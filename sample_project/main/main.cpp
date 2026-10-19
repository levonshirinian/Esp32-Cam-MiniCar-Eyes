// ===========================================================================
// follow_line / ESP-IDF port — AI-Thinker ESP32-CAM (OV2640)
//
// Same detector as the S3-CAM PlatformIO build. Same UART protocol:
//     L <err> <aim> <conf> <turn> <rows>\n
// Same three-scan-row logic, same NVS keys. Only the board and the sensor
// underneath changed.
//
// WHAT DID NOT PORT, AND WHY:
//   * all raw-register pokes. 0x380E/0x380F VTS, 0x3500-02 exposure and
//     0x3A02/03 AE cap are OV3660/OV5640 registers. The OV2640 has an
//     entirely different map and those writes land in the wrong bank.
//     The driver API is used instead — see apply_sensor_cfg().
//   * cfgExpMax and cfgVtsPct. They existed to stop an OV3660 from trading
//     frame rate for brightness. The OV2640 doesn't do that at QVGA.
//   * the FL_SNAP / FL_STREAM / FL_BRINGUP / FL_THERM / FL_PINS / FL_IDLE
//     builds. This is the flight build.
// ===========================================================================
#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_clk_tree.h"

extern "C" {
#include "esp_psram.h"
#include "nvs_flash.h"
#include "esp_camera.h"
}

#include "board_pins.h"
#include "line.h"

static const char *TAG = "follow";

// ---- UART link to the minicore --------------------------------------------
// GPIO16 is the only cleanly free pad on this board; GPIO13 is free unless.
// you populate the SD slot. Change both here if your wiring differs.
#define PIN_TX        13
#define PIN_RX        14
#define LINK_UART     UART_NUM_1
#define LINK_BAUD     115200

// ---- pacing ---------------------------------------------------------------
// The S3-CAM used 55 ms. OV2640 QVGA grayscale at 20 MHz hits ~15-20 fps
// with 2 fb in PSRAM, so 55 ms is a safe default — lower it once you've
// confirmed the HUD's fps counter actually rises.
#define FRAME_MS      55

static LineFinder finder;
static bool cam_is_up = false;

// ---- config (mirrors the S3 NVS keys, minus the OV3660-only ones) ---------
static int cfg_vflip    = 0;
static int cfg_hmirror  = 1;
static int cfg_aec      = 0;    // >0 = manual exposure, else auto
static int cfg_ae_level = 0;    // -2..+2, auto only
static int cfg_gain     = 0;    // >0 = manual gain 0..30, else AGC
static int cfg_contrast = 2;
static int cfg_small    = 0;    // 1 = QQVGA, 0 = QVGA

static void load_cfg(void) {
    nvs_handle_t h;
    if (nvs_open("fline", NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no NVS cfg — using defaults");
        return;
    }
    int32_t v;
    if (nvs_get_i32(h, "vflip",  &v) == ESP_OK) cfg_vflip    = v;
    if (nvs_get_i32(h, "hmir",   &v) == ESP_OK) cfg_hmirror  = v;
    if (nvs_get_i32(h, "aec",    &v) == ESP_OK) cfg_aec      = v;
    if (nvs_get_i32(h, "aelvl",  &v) == ESP_OK) cfg_ae_level = v;
    if (nvs_get_i32(h, "gain",   &v) == ESP_OK) cfg_gain     = v;
    if (nvs_get_i32(h, "con",    &v) == ESP_OK) cfg_contrast = v;
    if (nvs_get_i32(h, "small",  &v) == ESP_OK) cfg_small    = v;
    nvs_close(h);
}

// ---------------------------------------------------------------------------
// Sensor tweaks — driver API only, no raw registers.
// ---------------------------------------------------------------------------
static void apply_sensor_cfg(void) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;

    s->set_vflip(s, cfg_vflip);
    s->set_hmirror(s, cfg_hmirror);

    // Let AGC actually reach for gain rather than lengthening exposure —
    // a thresholded tape detector can afford noise far better than it can
    // afford motion blur.
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, GAINCEILING_32X);

    if (cfg_aec > 0) {
        s->set_exposure_ctrl(s, 0);
        s->set_aec_value(s, cfg_aec);
    } else {
        s->set_exposure_ctrl(s, 1);
        s->set_ae_level(s, cfg_ae_level);
    }

    if (cfg_gain > 0) {
        s->set_gain_ctrl(s, 0);
        s->set_agc_gain(s, cfg_gain);
    }

    s->set_whitebal(s, 0);      // grayscale; a WB loop only adds wander
    s->set_brightness(s, 0);
    s->set_contrast(s, cfg_contrast);
}

// ---------------------------------------------------------------------------
// Camera bring-up. 20 MHz first (the OV2640's happy place), 10 MHz as a
// fallback for a marginal ribbon. Five retries because a warm reboot can
// leave the sensor wedged and only a fresh esp_camera_init clears it.
// ---------------------------------------------------------------------------
static bool camera_init_attempt(framesize_t fs, int xclk, pixformat_t pf) {
    camera_config_t c = {};
    c.ledc_channel = LEDC_CHANNEL_0;
    c.ledc_timer   = LEDC_TIMER_0;
    c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
    c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
    c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
    c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
    c.pin_xclk = XCLK_GPIO_NUM;   c.pin_pclk  = PCLK_GPIO_NUM;
    c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href  = HREF_GPIO_NUM;
    c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
    c.pin_pwdn = PWDN_GPIO_NUM;   c.pin_reset = RESET_GPIO_NUM;

    c.xclk_freq_hz = xclk;
    // FORMAT AND SIZE ARE COUPLED. Raw formats above QVGA write PSRAM faster
    // than it accepts, and the symptom is missing scanlines — the driver
    // README says so explicitly. QVGA is the ceiling for GRAYSCALE.
    c.pixel_format = pf;
    c.frame_size   = (pf != PIXFORMAT_JPEG && fs > FRAMESIZE_QVGA) ? FRAMESIZE_QVGA : fs;
    c.jpeg_quality = 12;
    c.fb_count     = 2;
    c.fb_location  = CAMERA_FB_IN_PSRAM;
    c.grab_mode    = CAMERA_GRAB_LATEST;

    for (int attempt = 1; attempt <= 5; attempt++) {
        esp_err_t err = esp_camera_init(&c);
        if (err == ESP_OK) {
            if (attempt > 1) ESP_LOGI(TAG, "camera init OK on attempt %d", attempt);
            sensor_t *sen = esp_camera_sensor_get();
            if (sen) {
                ESP_LOGI(TAG, "sensor PID 0x%04x VER 0x%02x", sen->id.PID, sen->id.VER);
                // OV2640 PID is 0x26. 0x3660 is an OV3660 — which would mean
                // you have the wrong module or the wrong board file.
            }
            apply_sensor_cfg();
            cam_is_up = true;
            return true;
        }
        ESP_LOGW(TAG, "camera init attempt %d failed: 0x%x (%s)",
                 attempt, err, esp_err_to_name(err));
        esp_camera_deinit();
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    return false;
}

static bool camera_init(void) {
    framesize_t fs = cfg_small ? FRAMESIZE_QQVGA : FRAMESIZE_QVGA;
    ESP_LOGI(TAG, "PSRAM %s, %u bytes",
             esp_psram_is_initialized() ? "OK" : "MISSING",
             (unsigned)esp_psram_get_size());
    if (!esp_psram_is_initialized()) {
        ESP_LOGW(TAG, "no PSRAM — camera may be unstable at QVGA");
    }
    if (camera_init_attempt(fs, 20000000, PIXFORMAT_GRAYSCALE)) return true;
    ESP_LOGW(TAG, "20 MHz XCLK failed — retrying at 10 MHz (marginal ribbon?)");
    return camera_init_attempt(fs, 10000000, PIXFORMAT_GRAYSCALE);
}

// ---------------------------------------------------------------------------
// AE LOCK — freeze exposure and gain once tracking is solid, unfreeze when
// blind for a while. The OV2640 supports the same "flip to manual and hold"
// trick (set_exposure_ctrl(0) / set_gain_ctrl(0)).
// ---------------------------------------------------------------------------
static bool     g_ae_locked = false, g_ae_lock_broken = false;
static uint32_t g_ae_good_since = 0, g_ae_bad_since = 0, g_ae_locked_at = 0;

static void ae_lock_tick(uint8_t conf) {
    if (cfg_aec > 0 || g_ae_lock_broken) return;   // manual, or off
    sensor_t *s = esp_camera_sensor_get(); if (!s) return;
    const uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (!g_ae_locked) {
        if (conf >= 70) {
            if (!g_ae_good_since) g_ae_good_since = now;
            if (now - g_ae_good_since >= 1500) {
                s->set_exposure_ctrl(s, 0);
                s->set_gain_ctrl(s, 0);
                g_ae_locked = true;
                g_ae_locked_at = now;
                g_ae_bad_since = 0;
                ESP_LOGI(TAG, "AE LOCKED at conf %u", conf);
            }
        } else {
            g_ae_good_since = 0;
        }
    } else {
        if (conf < 25) {
            if (!g_ae_bad_since) g_ae_bad_since = now;
            const bool just_locked = (now - g_ae_locked_at) < 1200;
            if (just_locked || now - g_ae_bad_since >= 4000) {
                s->set_exposure_ctrl(s, 1);
                s->set_gain_ctrl(s, 1);
                g_ae_locked = false; g_ae_good_since = 0; g_ae_bad_since = 0;
                if (just_locked) {
                    g_ae_lock_broken = true;
                    ESP_LOGW(TAG, "AE lock blanked the frame — reverting to auto");
                } else {
                    ESP_LOGI(TAG, "AE unlocked: blind 4 s, letting it re-adapt");
                }
            }
        } else {
            g_ae_bad_since = 0;
        }
    }
}

// ---- minicore UART --------------------------------------------------------
static void link_uart_init(void) {
    uart_config_t cfg = {};
    cfg.baud_rate = LINK_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity    = UART_PARITY_DISABLE;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(LINK_UART, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LINK_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(LINK_UART,
        PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// ---- app_main -------------------------------------------------------------
extern "C" void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    ESP_LOGI(TAG, "\n=== follow_line / ESP32-CAM (AI-Thinker) ===");

    uint32_t cpu_hz = 0;
    if (esp_clk_tree_src_get_freq_hz(SOC_MOD_CLK_CPU,
                                     ESP_CLK_TREE_SRC_FREQ_PRECISION_EXACT,
                                     &cpu_hz) != ESP_OK) {
        cpu_hz = (uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ * 1000000;
    }
    ESP_LOGI(TAG, "target %s, %d core(s), %u MHz",
             CONFIG_IDF_TARGET, portNUM_PROCESSORS, (unsigned)(cpu_hz / 1000000));

    load_cfg();
    link_uart_init();

    if (!camera_init()) {
        // Report the failure on the wire too, so the C3 can tell a dead
        // camera from a dead cable. Then park.
        ESP_LOGE(TAG, "camera init FAILED — reporting silence on the link");
        while (1) {
            uart_write_bytes(LINK_UART, "L 0 0 0 0 0\n", 12);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

    ESP_LOGI(TAG, "up — GRAYSCALE %s, vflip=%d hmirror=%d con=%d",
             cfg_small ? "QQVGA" : "QVGA",
             cfg_vflip, cfg_hmirror, cfg_contrast);

    uint32_t frames = 0;
    uint32_t last_log = xTaskGetTickCount() * portTICK_PERIOD_MS;

    for (;;) {
        const uint32_t t0 = xTaskGetTickCount() * portTICK_PERIOD_MS;

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            // A flight build that goes silent when the camera stops is
            // undiagnosable: no frame means no UART report and the C3 sees
            // only a dead link. Say so on the console.
            static uint32_t misses = 0, last_miss = 0;
            misses++;
            const uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last_miss >= 1000) {
                last_miss = now;
                ESP_LOGW(TAG, "NO FRAME from camera (%u misses)", misses);
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        finder.setFrame(fb->buf, fb->width, fb->height);
        LineResult r = finder.run();
        ae_lock_tick(r.conf);
        frames++;

        // Order matters: UART report goes out BEFORE anything else. The C3's
        // link timeout is the robot's fail-safe, nothing is allowed in front
        // of it.
        char buf[64];
        int n = snprintf(buf, sizeof(buf), "L %d %d %d %d %d\n",
                         (int)(r.err  * 1000),
                         (int)(r.aim  * 1000),
                         (int) r.conf,
                         (int)(r.turn * 1000),
                         (int) r.rowsOk);
        uart_write_bytes(LINK_UART, buf, n);

        esp_camera_fb_return(fb);

        // 1 Hz console log.
        const uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_log >= 1000) {
            ESP_LOGI(TAG,
                "err=%+.3f aim=%+.3f turn=%+.3f conf=%u rows n/m/f=%d%d%d fps %.1f",
                r.err, r.aim, r.turn, r.conf,
                r.near_.ok, r.mid.ok, r.far_.ok,
                frames * 1000.0f / (now - last_log));
            last_log = now;
            frames = 0;
        }

        const uint32_t dt = (xTaskGetTickCount() * portTICK_PERIOD_MS) - t0;
        if (dt < FRAME_MS) vTaskDelay(pdMS_TO_TICKS(FRAME_MS - dt));
    }
}