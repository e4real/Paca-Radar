// M0 bring-up: CO5300 466x466 AMOLED via Arduino_GFX (QSPI) + LVGL.
// Pins come from config.h (confirmed against the Waveshare board definition and a
// working Arduino_GFX port for this exact panel). The panel runs off the always-on
// DC1 rail, so it lights up without configuring the AXP2101 PMIC.
// The actual UI is built by ui_boot_create() (shared with the native SDL sim).
#include "display.h"
#include "config.h"
#include "radar_view.h"
#include "ui.h"
#include "touch_cst9217.h"

#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <lvgl.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <string.h>

// --- Arduino_GFX panel -------------------------------------------------------
// Typed as Arduino_CO5300* (not Arduino_GFX*) so setBrightness() — declared on
// Arduino_OLED, not the GFX base — is reachable.
static Arduino_DataBus *s_bus = nullptr;
static Arduino_CO5300  *s_gfx = nullptr;

// --- LVGL plumbing -----------------------------------------------------------
#define LVGL_BUF_LINES 40    // partial draw-buffer height (lines); kept in fast internal RAM
static lv_disp_draw_buf_t s_draw_buf;
static lv_disp_drv_t      s_disp_drv;
static lv_indev_drv_t     s_indev_drv;
static lv_color_t        *s_buf1 = nullptr;
static lv_color_t        *s_buf2 = nullptr;

static volatile uint32_t s_frameCount = 0;   // rendered frames (last-flush), for FPS measurement
uint32_t display_frames() { return s_frameCount; }

static volatile uint8_t s_rot = 0;           // display rotation: 0/1/2/3 = 0°/90°/180°/270°
static lv_color_t *s_rotBuf = nullptr;       // PSRAM scratch for 90/270° transpose (see begin())

// Arbitrary whole-screen rotation. When s_uiRotDeg != 0 the flush stops doing the cheap
// 90-step transpose and instead composites every LVGL stripe into a persistent full-frame
// buffer (s_fb), then rotates that whole frame by s_uiRotDeg into s_rotFb and pushes it.
// Touch input is run through the inverse rotation, so taps, the detail card, text, and the
// nm readout all rotate together and stay interactive. Cost: a full-frame rotate + push per
// refresh (heavier than partial updates), which is why it's only engaged when nonzero.
static float        s_uiRotDeg = 0.0f;       // 0 = off
static float        s_uiCos    = 1.0f;       // cos(s_uiRotDeg), precomputed
static float        s_uiSin    = 0.0f;       // sin(s_uiRotDeg), precomputed
static lv_color_t  *s_fb       = nullptr;    // PSRAM: full unrotated frame (composited)
static lv_color_t  *s_rotFb    = nullptr;    // PSRAM: full rotated frame (pushed to panel)

// LVGL -> panel, applying the chosen rotation while pushing.
//   0°   : straight through.
//   180° : reverse the flat block in place — no scratch buffer.
//   90°/270° : the block transposes (w<->h), so it can't be reversed in place; copy it
//              rotated into a PSRAM scratch buffer. That buffer MUST live in PSRAM — an
//              internal-RAM one starves the mbedTLS handshake and kills the ADS-B feed.
static void flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *px) {
    const int w = (int)(area->x2 - area->x1 + 1);
    const int h = (int)(area->y2 - area->y1 + 1);

    // ---- arbitrary whole-screen rotation path -------------------------------
    if (s_uiRotDeg != 0.0f && s_fb && s_rotFb) {
        // 1) composite this dirty stripe into the persistent full (unrotated) frame
        for (int j = 0; j < h; ++j) {
            const int yy = area->y1 + j;
            if (yy < 0 || yy >= SCREEN_H) continue;
            int x1 = area->x1, x2 = area->x2;
            if (x1 < 0) x1 = 0;
            if (x2 > SCREEN_W - 1) x2 = SCREEN_W - 1;
            const int cnt = x2 - x1 + 1;
            if (cnt > 0)
                memcpy(&s_fb[yy * SCREEN_W + x1], &px[j * w + (x1 - area->x1)], (size_t)cnt * sizeof(lv_color_t));
        }
        // 2) once the frame is complete, rotate the whole thing and push it
        if (lv_disp_flush_is_last(drv)) {
            const float cx = (SCREEN_W - 1) * 0.5f, cy = (SCREEN_H - 1) * 0.5f;
            const float ca = s_uiCos, sa = s_uiSin;
            const lv_color_t black = lv_color_black();
            for (int y = 0; y < SCREEN_H; ++y) {
                // source coords for dest x=0, then step by (+ca,-sa) per dest x (affine, no per-pixel trig)
                float sx = cx + (0.0f - cx) * ca + ((float)y - cy) * sa;
                float sy = cy - (0.0f - cx) * sa + ((float)y - cy) * ca;
                lv_color_t *drow = &s_rotFb[y * SCREEN_W];
                for (int x = 0; x < SCREEN_W; ++x) {
                    const int ix = (int)(sx + 0.5f);
                    const int iy = (int)(sy + 0.5f);
                    drow[x] = (ix >= 0 && ix < SCREEN_W && iy >= 0 && iy < SCREEN_H)
                                  ? s_fb[iy * SCREEN_W + ix] : black;
                    sx += ca;
                    sy -= sa;
                }
            }
#if (LV_COLOR_16_SWAP != 0)
            s_gfx->draw16bitBeRGBBitmap(0, 0, (uint16_t *)s_rotFb, SCREEN_W, SCREEN_H);
#else
            s_gfx->draw16bitRGBBitmap(0, 0, (uint16_t *)s_rotFb, SCREEN_W, SCREEN_H);
#endif
            s_frameCount++;
        }
        lv_disp_flush_ready(drv);
        return;
    }

    // ---- original 0/90/180/270 path -----------------------------------------
    lv_color_t *out = px;
    int16_t  dx = area->x1, dy = area->y1;
    uint16_t dw = (uint16_t)w, dh = (uint16_t)h;

    switch (s_rot) {
        case 2:  // 180°
            for (int i = 0, j = w * h - 1; i < j; ++i, --j) { lv_color_t t = px[i]; px[i] = px[j]; px[j] = t; }
            dx = (int16_t)(SCREEN_W - 1 - area->x2);
            dy = (int16_t)(SCREEN_H - 1 - area->y2);
            break;
        case 1:  // 90° CW
            if (s_rotBuf) {
                for (int j = 0; j < h; ++j)
                    for (int i = 0; i < w; ++i)
                        s_rotBuf[i * h + (h - 1 - j)] = px[j * w + i];
                out = s_rotBuf; dw = (uint16_t)h; dh = (uint16_t)w;
                dx = (int16_t)(SCREEN_H - 1 - area->y2); dy = area->x1;
            }
            break;
        case 3:  // 270° CW
            if (s_rotBuf) {
                for (int j = 0; j < h; ++j)
                    for (int i = 0; i < w; ++i)
                        s_rotBuf[(w - 1 - i) * h + j] = px[j * w + i];
                out = s_rotBuf; dw = (uint16_t)h; dh = (uint16_t)w;
                dx = area->y1; dy = (int16_t)(SCREEN_W - 1 - area->x2);
            }
            break;
        default: break;  // 0°
    }
#if (LV_COLOR_16_SWAP != 0)
    s_gfx->draw16bitBeRGBBitmap(dx, dy, (uint16_t *)out, dw, dh);
#else
    s_gfx->draw16bitRGBBitmap(dx, dy, (uint16_t *)out, dw, dh);
#endif
    if (lv_disp_flush_is_last(drv)) s_frameCount++;
    lv_disp_flush_ready(drv);
}

// CO5300 (QSPI) requires 2-pixel-aligned flush windows: even start, odd end.
// Without this, partial-area updates (e.g. the radar sweep) tear / ghost / flicker.
static void rounder_cb(lv_disp_drv_t *drv, lv_area_t *area) {
    (void)drv;
    area->x1 &= ~1;
    area->y1 &= ~1;
    area->x2 |= 1;
    area->y2 |= 1;
}

// CST9217 touch -> LVGL pointer. LVGL keeps the last point on release.
static void touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    (void)drv;
    uint16_t x, y;
    if (touch_read(&x, &y)) {
        uint16_t lx = x, ly = y;                          // map physical touch -> logical (inverse rotation)
        if (s_uiRotDeg != 0.0f) {
            const float cx = (SCREEN_W - 1) * 0.5f, cy = (SCREEN_H - 1) * 0.5f;
            const float fx = (float)x - cx, fy = (float)y - cy;
            int mlx = (int)lroundf(cx + fx * s_uiCos + fy * s_uiSin);
            int mly = (int)lroundf(cy - fx * s_uiSin + fy * s_uiCos);
            if (mlx < 0) mlx = 0; if (mlx > SCREEN_W - 1) mlx = SCREEN_W - 1;
            if (mly < 0) mly = 0; if (mly > SCREEN_H - 1) mly = SCREEN_H - 1;
            lx = (uint16_t)mlx; ly = (uint16_t)mly;
        } else
        switch (s_rot) {
            case 1: lx = y;                            ly = (uint16_t)(SCREEN_H - 1 - x); break;
            case 2: lx = (uint16_t)(SCREEN_W - 1 - x); ly = (uint16_t)(SCREEN_H - 1 - y); break;
            case 3: lx = (uint16_t)(SCREEN_W - 1 - y); ly = x;                            break;
            default: break;
        }
        data->point.x = (lv_coord_t)lx;
        data->point.y = (lv_coord_t)ly;
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

namespace display {

bool begin() {
    Serial.println("[display] init CO5300 QSPI...");
    s_bus = new Arduino_ESP32QSPI(PIN_LCD_CS, PIN_LCD_SCLK,
                                  PIN_LCD_D0, PIN_LCD_D1, PIN_LCD_D2, PIN_LCD_D3);
    s_gfx = new Arduino_CO5300(s_bus, PIN_LCD_RST, 0 /*rotation*/,
                               SCREEN_W, SCREEN_H,
                               LCD_COL_OFFSET, LCD_ROW_OFFSET, 0, 0);
    if (!s_gfx->begin(LCD_QSPI_HZ)) {
        Serial.println("[display] gfx->begin() FAILED");
        return false;
    }
    s_gfx->fillScreen(RGB565_BLACK);
    s_gfx->setBrightness(BRIGHTNESS_DEFAULT);
    Serial.println("[display] panel up; init LVGL...");

    lv_init();

    // Draw scratch in INTERNAL DMA RAM: rendering anti-aliased graphics into PSRAM is
    // slow (that, not QSPI bandwidth, was the bottleneck). Keep the active buffer in fast
    // internal SRAM; single partial buffer to stay within the internal-RAM budget.
    const size_t buf_px = (size_t)SCREEN_W * LVGL_BUF_LINES;
    s_buf1 = (lv_color_t *)heap_caps_malloc(buf_px * sizeof(lv_color_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    s_buf2 = nullptr;
    if (!s_buf1) {
        Serial.println("[display] internal draw buffer failed; falling back to PSRAM");
        s_buf1 = (lv_color_t *)heap_caps_malloc(buf_px * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    }
    lv_disp_draw_buf_init(&s_draw_buf, s_buf1, s_buf2, buf_px);

    // Scratch target for 90/270° rotation (the block transposes, so it can't rotate in
    // place). Deliberately in PSRAM: an internal-RAM buffer here would eat the contiguous
    // block the TLS handshake needs and break the ADS-B feed. NULL is fine (rotation just
    // falls back to un-rotated for 90/270° if PSRAM is exhausted).
    s_rotBuf = (lv_color_t *)heap_caps_malloc(buf_px * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);

    // Full-frame buffers for arbitrary whole-screen rotation (PSRAM; ~434 KB each). One
    // holds the composited unrotated frame, the other the rotated frame we push. Allocated
    // up front (before WiFi/TLS) so the contiguous PSRAM the handshake needs is still free.
    // If either fails we just never enable arbitrary rotation (the flush guards on both).
    const size_t fb_px = (size_t)SCREEN_W * SCREEN_H;
    s_fb    = (lv_color_t *)heap_caps_malloc(fb_px * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    s_rotFb = (lv_color_t *)heap_caps_malloc(fb_px * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    if (s_fb) memset(s_fb, 0, fb_px * sizeof(lv_color_t));

    lv_disp_drv_init(&s_disp_drv);
    s_disp_drv.hor_res  = SCREEN_W;
    s_disp_drv.ver_res  = SCREEN_H;
    s_disp_drv.flush_cb = flush_cb;
    s_disp_drv.rounder_cb = rounder_cb;     // CO5300 needs 2-px-aligned windows
    s_disp_drv.draw_buf = &s_draw_buf;
    lv_disp_drv_register(&s_disp_drv);

    // Touch input (CST9217) -> LVGL pointer indev (drives tap-to-inspect + swipe).
    if (touch_begin()) {
        lv_indev_drv_init(&s_indev_drv);
        s_indev_drv.type = LV_INDEV_TYPE_POINTER;
        s_indev_drv.read_cb = touch_read_cb;
        lv_indev_drv_register(&s_indev_drv);
        Serial.println("[display] CST9217 touch registered");
    }

    Serial.printf("[display] PSRAM free: %u KB\n", (unsigned)(ESP.getFreePsram() / 1024));
    ui_create();                   // M3: radar/list/stats views + tap-to-inspect
    Serial.println("[display] LVGL ready");
    return true;
}

void loop() { lv_timer_handler(); }

void setBrightness(uint8_t v) { if (s_gfx) s_gfx->setBrightness(v); }

void setRotation(uint8_t quarters) {
    s_rot = (uint8_t)(quarters & 3);   // 0..3 = 0°/90°/180°/270°
    lv_obj_t *scr = lv_scr_act();
    if (scr) lv_obj_invalidate(scr);   // full repaint in the new orientation
}
uint8_t rotation() { return s_rot; }

void setUiRotation(float deg) {
    float d = fmodf(deg, 360.0f);
    if (d < 0.0f) d += 360.0f;
    s_uiRotDeg = d;
    const float a = d * (float)M_PI / 180.0f;
    s_uiCos = cosf(a);
    s_uiSin = sinf(a);
    if (d != 0.0f) s_rot = 0;          // arbitrary rotation owns the flush; drop the 90-step path
    if (s_fb) memset(s_fb, 0, (size_t)SCREEN_W * SCREEN_H * sizeof(lv_color_t));
    lv_obj_t *scr = lv_scr_act();
    if (scr) lv_obj_invalidate(scr);   // force a full repaint so the whole frame recomposites
}
float uiRotation() { return s_uiRotDeg; }

uint32_t inactiveMs() { return lv_disp_get_inactive_time(NULL); }

} // namespace display
