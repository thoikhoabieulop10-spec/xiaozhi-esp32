#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/lcd_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <driver/spi_common.h>

#if defined(LCD_TYPE_ILI9341_SERIAL)
#include "esp_lcd_ili9341.h"
#endif

#if defined(LCD_TYPE_GC9A01_SERIAL)
#include "esp_lcd_gc9a01.h"
static const gc9a01_lcd_init_cmd_t gc9107_lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfe, (uint8_t[]){0x00}, 0, 0},
    {0xef, (uint8_t[]){0x00}, 0, 0},
    {0xb0, (uint8_t[]){0xc0}, 1, 0},
    {0xb1, (uint8_t[]){0x80}, 1, 0},
    {0xb2, (uint8_t[]){0x27}, 1, 0},
    {0xb3, (uint8_t[]){0x13}, 1, 0},
    {0xb6, (uint8_t[]){0x19}, 1, 0},
    {0xb7, (uint8_t[]){0x05}, 1, 0},
    {0xac, (uint8_t[]){0xc8}, 1, 0},
    {0xab, (uint8_t[]){0x0f}, 1, 0},
    {0x3a, (uint8_t[]){0x05}, 1, 0},
    {0xb4, (uint8_t[]){0x04}, 1, 0},
    {0xa8, (uint8_t[]){0x08}, 1, 0},
    {0xb8, (uint8_t[]){0x08}, 1, 0},
    {0xea, (uint8_t[]){0x02}, 1, 0},
    {0xe8, (uint8_t[]){0x2A}, 1, 0},
    {0xe9, (uint8_t[]){0x47}, 1, 0},
    {0xe7, (uint8_t[]){0x5f}, 1, 0},
    {0xc6, (uint8_t[]){0x21}, 1, 0},
    {0xc7, (uint8_t[]){0x15}, 1, 0},
    {0xf0,
    (uint8_t[]){0x1D, 0x38, 0x09, 0x4D, 0x92, 0x2F, 0x35, 0x52, 0x1E, 0x0C,
                0x04, 0x12, 0x14, 0x1f},
    14, 0},
    {0xf1,
    (uint8_t[]){0x16, 0x40, 0x1C, 0x54, 0xA9, 0x2D, 0x2E, 0x56, 0x10, 0x0D,
                0x0C, 0x1A, 0x14, 0x1E},
    14, 0},
    {0xf4, (uint8_t[]){0x00, 0x00, 0xFF}, 3, 0},
    {0xba, (uint8_t[]){0xFF, 0xFF}, 2, 0},
};
#endif

// ============================================================================
//  "Nhu Y" UI: left button column (Noi / Am + / Am - / Reset), student
//  avatar header and chat bubbles, driven by the XPT2046 resistive touch.
// ============================================================================
#include "nhuy_assets.h"
#include "assets/lang_config.h"
#include "settings.h"
#include "lvgl_theme.h"

#include <esp_lvgl_port.h>
#include <esp_system.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <string>

#define TAG "CompactWifiBoardLCD"

// ---- XPT2046 touch wiring (shares SCK=GPIO21 and MOSI=GPIO47 with the LCD) ----
#define TOUCH_IRQ_PIN   GPIO_NUM_14   // T_IRQ
#define TOUCH_CS_PIN    GPIO_NUM_12   // T_CS
#define TOUCH_MISO_PIN  GPIO_NUM_13   // T_DO

// ---- UI geometry / colours ----
#define NY_SIDE_W     72
#define NY_HEADER_H   128
#define NY_MAX_MSG    16

static const char* const kRoleUser = "user";
static const char* const kRoleAssistant = "assistant";
static const char* const kRoleSystem = "system";

// --------------------------------------------------------------------------
//  Minimal XPT2046 driver (polled, PENIRQ gated)
// --------------------------------------------------------------------------
class Xpt2046 {
public:
    bool Init(spi_host_device_t host) {
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << TOUCH_IRQ_PIN;
        io.mode = GPIO_MODE_INPUT;
        io.pull_up_en = GPIO_PULLUP_ENABLE;
        io.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io);

        spi_device_interface_config_t dev = {};
        dev.mode = 0;
        dev.clock_speed_hz = 1 * 1000 * 1000;
        dev.spics_io_num = TOUCH_CS_PIN;
        dev.queue_size = 2;
        if (spi_bus_add_device(host, &dev, &dev_) != ESP_OK) {
            ESP_LOGE(TAG, "XPT2046: spi_bus_add_device failed");
            dev_ = nullptr;
            return false;
        }
        Read(0xD0);  // leave the chip in power-down with PENIRQ enabled
        return true;
    }

    bool Touched() const { return gpio_get_level(TOUCH_IRQ_PIN) == 0; }

    // Median-filtered raw reading. a = X channel, b = Y channel (12 bit).
    bool ReadRaw(int& a, int& b) {
        if (dev_ == nullptr || !Touched()) {
            return false;
        }
        const int n = 3;
        int xs[n], ys[n];
        for (int i = 0; i < n; i++) {
            xs[i] = Read(0xD0);
            ys[i] = Read(0x90);
        }
        int z1 = Read(0xB0);
        if (!Touched() || z1 < 20) {
            return false;
        }
        std::sort(xs, xs + n);
        std::sort(ys, ys + n);
        a = xs[n / 2];
        b = ys[n / 2];
        if (a < 50 || a > 4050 || b < 50 || b > 4050) {
            return false;
        }
        return true;
    }

private:
    spi_device_handle_t dev_ = nullptr;

    int Read(uint8_t cmd) {
        if (dev_ == nullptr) {
            return 0;
        }
        spi_transaction_t t = {};
        t.length = 24;
        t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        t.tx_data[0] = cmd;
        if (spi_device_transmit(dev_, &t) != ESP_OK) {
            return 0;
        }
        return ((((int)t.rx_data[1]) << 8) | t.rx_data[2]) >> 3;
    }
};

// --------------------------------------------------------------------------
//  Display with the custom "Nhu Y" layout
// --------------------------------------------------------------------------
class NhuYDisplay : public SpiLcdDisplay {
public:
    NhuYDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel, int width,
                int height, int offset_x, int offset_y, bool mirror_x, bool mirror_y,
                bool swap_xy)
        : SpiLcdDisplay(panel_io, panel, width, height, offset_x, offset_y, mirror_x, mirror_y,
                        swap_xy) {}

    virtual void SetupUI() override {
        if (setup_ui_called_) {
            return;
        }
        Display::SetupUI();
        DisplayLockGuard lock(this);

        auto theme = static_cast<LvglTheme*>(current_theme_);
        const lv_font_t* text_font = theme->text_font()->font();
        const lv_font_t* icon_font = theme->icon_font()->font();
        const int main_w = LV_HOR_RES - NY_SIDE_W;

        lv_obj_t* screen = lv_screen_active();
        lv_obj_set_style_text_font(screen, text_font, 0);
        lv_obj_set_style_text_color(screen, lv_color_hex(0x1E1E28), 0);
        lv_obj_set_style_bg_color(screen, lv_color_hex(0x8CC8FF), 0);
        lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
        lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

        /* ---------------- Left button column ---------------- */
        side_bar_ = lv_obj_create(screen);
        lv_obj_remove_style_all(side_bar_);
        lv_obj_set_size(side_bar_, NY_SIDE_W, LV_VER_RES);
        lv_obj_set_pos(side_bar_, 0, 0);
        lv_obj_set_style_bg_opa(side_bar_, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(side_bar_, lv_color_hex(0x1E4BA0), 0);
        lv_obj_remove_flag(side_bar_, LV_OBJ_FLAG_SCROLLABLE);

        mic_btn_ = MakeButton(side_bar_, &nhuy_icon_mic, (NY_SIDE_W - 60) / 2, 10, 60, 76,
                              lv_color_hex(0x3C8CFF), OnMic, LV_EVENT_PRESSED);
        MakeButton(side_bar_, &nhuy_icon_vup, (NY_SIDE_W - 60) / 2, 102, 60, 58,
                   lv_color_hex(0x3C8CFF), OnVolUp, LV_EVENT_PRESSED);

        volume_label_ = lv_label_create(side_bar_);
        lv_obj_set_width(volume_label_, NY_SIDE_W);
        lv_obj_set_pos(volume_label_, 0, 166);
        lv_obj_set_style_text_align(volume_label_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(volume_label_, lv_color_white(), 0);
        lv_label_set_text(volume_label_, "");

        MakeButton(side_bar_, &nhuy_icon_vdown, (NY_SIDE_W - 60) / 2, 196, 60, 58,
                   lv_color_hex(0x3C8CFF), OnVolDown, LV_EVENT_PRESSED);
        MakeButton(side_bar_, &nhuy_icon_home, (NY_SIDE_W - 60) / 2, 270, 60, 58,
                   lv_color_hex(0x1EAA5A), OnHome, LV_EVENT_PRESSED);
        MakeButton(side_bar_, &nhuy_icon_reset, (NY_SIDE_W - 60) / 2, LV_VER_RES - 72, 60, 62,
                   lv_color_hex(0xDC3C46), OnReset, LV_EVENT_CLICKED);

        /* ================= CHAT VIEW ================= */
        chat_view_ = MakePanel(screen, NY_SIDE_W, 0, main_w, LV_VER_RES, 0xFDF8EE);

        top_bar_ = MakePanel(chat_view_, 0, 0, main_w, NY_HEADER_H, 0x8CC8FF);

        avatar_ = lv_image_create(top_bar_);
        lv_image_set_src(avatar_, &nhuy_face_neutral);
        lv_obj_set_pos(avatar_, 2, NY_HEADER_H - nhuy_face_neutral.header.h);
        lv_obj_add_flag(avatar_, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(avatar_, OnBackHome, LV_EVENT_PRESSED, this);

        lv_obj_t* title = lv_image_create(top_bar_);
        lv_image_set_src(title, &nhuy_title);
        lv_obj_set_pos(title, 110, 20);

        MakeButton(top_bar_, &nhuy_btn_back_small, 110, 56, 104, 30, lv_color_hex(0x286EDC),
                   OnBackHome, LV_EVENT_PRESSED);

        network_label_ = MakeIcon(top_bar_, icon_font, 0x143278, -6);
        battery_label_ = MakeIcon(top_bar_, icon_font, 0x143278, -34);
        mute_label_ = MakeIcon(top_bar_, icon_font, 0xDC3C46, -62);

        lv_obj_t* divider = MakePanel(chat_view_, 0, NY_HEADER_H, main_w, 3, 0x286EDC);
        (void)divider;

        content_ = lv_obj_create(chat_view_);
        lv_obj_remove_style_all(content_);
        lv_obj_set_size(content_, main_w, LV_VER_RES - NY_HEADER_H - 3);
        lv_obj_set_pos(content_, 0, NY_HEADER_H + 3);
        lv_obj_set_style_bg_opa(content_, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(content_, lv_color_hex(0xFDF8EE), 0);
        lv_obj_set_style_pad_all(content_, 8, 0);
        lv_obj_set_style_pad_row(content_, 8, 0);
        lv_obj_set_flex_flow(content_, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(content_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START,
                              LV_FLEX_ALIGN_START);
        lv_obj_set_scroll_dir(content_, LV_DIR_VER);
        lv_obj_set_scrollbar_mode(content_, LV_SCROLLBAR_MODE_OFF);

        /* ================= HOME / MENU VIEW ================= */
        home_view_ = MakePanel(screen, NY_SIDE_W, 0, main_w, LV_VER_RES, 0x8CC8FF);

        big_face_ = lv_image_create(home_view_);
        lv_image_set_src(big_face_, &nhuy_big_neutral);
        lv_obj_add_flag(big_face_, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_ext_click_area(big_face_, 12);
        lv_obj_add_event_cb(big_face_, OnFaceTap, LV_EVENT_PRESSED, this);

        home_title_ = lv_image_create(home_view_);
        lv_image_set_src(home_title_, &nhuy_title);

        home_hint_ = lv_image_create(home_view_);
        lv_image_set_src(home_hint_, &nhuy_hint);

        home_info_ = lv_label_create(home_view_);
        lv_obj_set_width(home_info_, main_w - 20);
        lv_label_set_long_mode(home_info_, LV_LABEL_LONG_WRAP);
        lv_obj_set_style_text_align(home_info_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(home_info_, lv_color_hex(0x143278), 0);
        lv_label_set_text(home_info_, "");

        menu_box_ = MakePanel(home_view_, 0, 230, main_w, LV_VER_RES - 230, 0x8CC8FF);
        lv_obj_set_style_bg_opa(menu_box_, LV_OPA_TRANSP, 0);
        const int bx = (main_w - 228) / 2;
        MakeButton(menu_box_, &nhuy_btn_story, bx, 4, 228, 52, lv_color_hex(0x8C5AE6), OnStory,
                   LV_EVENT_PRESSED);
        MakeButton(menu_box_, &nhuy_btn_learn, bx, 64, 228, 52, lv_color_hex(0x1EAA5A), OnLearn,
                   LV_EVENT_PRESSED);
        MakeButton(menu_box_, &nhuy_btn_music, bx, 124, 228, 52, lv_color_hex(0xF08C28),
                   OnMusic, LV_EVENT_PRESSED);
        MakeButton(menu_box_, &nhuy_btn_back, bx, 184, 228, 52, lv_color_hex(0x5A6E8C), OnBackHome,
                   LV_EVENT_PRESSED);

        /* Status pill (moved between the views) */
        status_bar_ = lv_obj_create(home_view_);
        lv_obj_remove_style_all(status_bar_);
        lv_obj_set_style_radius(status_bar_, 14, 0);
        lv_obj_set_style_bg_opa(status_bar_, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(status_bar_, lv_color_white(), 0);
        lv_obj_remove_flag(status_bar_, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_remove_flag(status_bar_, LV_OBJ_FLAG_CLICKABLE);

        status_label_ = lv_label_create(status_bar_);
        lv_label_set_long_mode(status_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_style_text_align(status_label_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(status_label_, lv_color_hex(0x286EDC), 0);
        lv_label_set_text(status_label_, Lang::Strings::INITIALIZING);

        notification_label_ = lv_label_create(status_bar_);
        lv_label_set_long_mode(notification_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_style_text_align(notification_label_, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_style_text_color(notification_label_, lv_color_hex(0xE07814), 0);
        lv_label_set_text(notification_label_, "");
        lv_obj_add_flag(notification_label_, LV_OBJ_FLAG_HIDDEN);

        base_face_ = &nhuy_face_neutral;
        ShowHome();
        UpdateVolumeLabel();
        anim_timer_ = lv_timer_create(AnimTick, 150, this);
    }

    virtual void SetChatMessage(const char* role, const char* content) override {
        DisplayLockGuard lock(this);
        if (content_ == nullptr || role == nullptr) {
            return;
        }
        const char* role_tag = kRoleAssistant;
        if (strcmp(role, "user") == 0) {
            role_tag = kRoleUser;
        } else if (strcmp(role, "system") == 0) {
            role_tag = kRoleSystem;
        }

        if (role_tag == kRoleSystem) {
            // System lines (activation code, errors...) are also shown on the home screen.
            if (home_info_ != nullptr) {
                lv_label_set_text(home_info_, content != nullptr ? content : "");
            }
            if (system_row_ != nullptr) {
                if (lv_obj_is_valid(system_row_)) {
                    lv_obj_delete(system_row_);
                }
                system_row_ = nullptr;
            }
        }
        if (content == nullptr || content[0] == '\0') {
            return;
        }
        if (role_tag == kRoleUser && IsNoiseTranscript(content)) {
            ESP_LOGW(TAG, "Hidden noise transcript: %s", content);
            return;  // e.g. "Lala School": speech recognition hearing background noise
        }
        if (role_tag != kRoleSystem && view_ != kViewChat) {
            ShowChat();
        }

        while (lv_obj_get_child_cnt(content_) >= NY_MAX_MSG) {
            lv_obj_t* first = lv_obj_get_child(content_, 0);
            if (first == system_row_) {
                system_row_ = nullptr;
            }
            lv_obj_delete(first);
        }

        const int main_w = LV_HOR_RES - NY_SIDE_W;
        const int max_text_w = main_w * 80 / 100 - 20;

        lv_obj_t* row = lv_obj_create(content_);
        lv_obj_remove_style_all(row);
        lv_obj_set_width(row, lv_pct(100));
        lv_obj_set_height(row, LV_SIZE_CONTENT);
        lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_remove_flag(row, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_user_data(row, (void*)role_tag);

        lv_obj_t* bubble = lv_obj_create(row);
        lv_obj_remove_style_all(bubble);
        lv_obj_remove_flag(bubble, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_remove_flag(bubble, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_radius(bubble, 12, 0);
        lv_obj_set_style_bg_opa(bubble, LV_OPA_COVER, 0);
        lv_obj_set_style_pad_hor(bubble, 10, 0);
        lv_obj_set_style_pad_ver(bubble, 6, 0);

        lv_obj_t* label = lv_label_create(bubble);
        lv_label_set_text(label, content);
        lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(label, LV_SIZE_CONTENT);
        lv_obj_update_layout(label);
        if (lv_obj_get_width(label) > max_text_w) {
            lv_obj_set_width(label, max_text_w);
        }
        lv_obj_set_size(bubble, LV_SIZE_CONTENT, LV_SIZE_CONTENT);

        if (role_tag == kRoleUser) {
            lv_obj_set_style_bg_color(bubble, lv_color_hex(0x2882F0), 0);
            lv_obj_set_style_text_color(label, lv_color_white(), 0);
            lv_obj_align(bubble, LV_ALIGN_TOP_RIGHT, 0, 0);
        } else if (role_tag == kRoleAssistant) {
            lv_obj_set_style_bg_color(bubble, lv_color_hex(0xFFF0C8), 0);
            lv_obj_set_style_border_width(bubble, 1, 0);
            lv_obj_set_style_border_color(bubble, lv_color_hex(0xF0B450), 0);
            lv_obj_set_style_text_color(label, lv_color_hex(0x1E1E28), 0);
            lv_obj_align(bubble, LV_ALIGN_TOP_LEFT, 0, 0);
        } else {
            lv_obj_set_style_bg_color(bubble, lv_color_hex(0xE4EAF4), 0);
            lv_obj_set_style_text_color(label, lv_color_hex(0x46506A), 0);
            lv_obj_align(bubble, LV_ALIGN_TOP_MID, 0, 0);
            system_row_ = row;
        }

        lv_obj_update_layout(content_);
        lv_obj_scroll_to_view_recursive(row, LV_ANIM_ON);
    }

    virtual void RemoveChatMessage(const char* role, const char* content) override {
        DisplayLockGuard lock(this);
        if (content_ == nullptr || role == nullptr || content == nullptr) {
            return;
        }
        uint32_t count = lv_obj_get_child_cnt(content_);
        for (uint32_t i = 0; i < count; i++) {
            lv_obj_t* row = lv_obj_get_child(content_, i);
            const char* tag = static_cast<const char*>(lv_obj_get_user_data(row));
            if (tag == nullptr || strcmp(tag, role) != 0 || lv_obj_get_child_cnt(row) == 0) {
                continue;
            }
            lv_obj_t* bubble = lv_obj_get_child(row, 0);
            if (lv_obj_get_child_cnt(bubble) == 0) {
                continue;
            }
            lv_obj_t* label = lv_obj_get_child(bubble, 0);
            if (strcmp(lv_label_get_text(label), content) != 0) {
                continue;
            }
            if (row == system_row_) {
                system_row_ = nullptr;
            }
            lv_obj_delete(row);
            return;
        }
    }

    virtual void ClearChatMessages() override {
        DisplayLockGuard lock(this);
        if (content_ == nullptr) {
            return;
        }
        lv_obj_clean(content_);
        system_row_ = nullptr;
    }

    virtual bool SetEmotion(const char* emotion) override {
        const lv_image_dsc_t* face = MapEmotion(emotion);
        DisplayLockGuard lock(this);
        base_face_ = face;
        ApplyFace(face);
        return true;
    }

    virtual void SetTheme(Theme* theme) override {
        DisplayLockGuard lock(this);
        auto lvgl_theme = static_cast<LvglTheme*>(theme);
        if (lvgl_theme != nullptr && setup_ui_called_) {
            if (lvgl_theme->text_font() != nullptr) {
                lv_obj_set_style_text_font(lv_screen_active(), lvgl_theme->text_font()->font(), 0);
            }
            if (lvgl_theme->icon_font() != nullptr) {
                const lv_font_t* icon_font = lvgl_theme->icon_font()->font();
                if (network_label_ != nullptr) lv_obj_set_style_text_font(network_label_, icon_font, 0);
                if (battery_label_ != nullptr) lv_obj_set_style_text_font(battery_label_, icon_font, 0);
                if (mute_label_ != nullptr) lv_obj_set_style_text_font(mute_label_, icon_font, 0);
            }
        }
        if (theme != nullptr) {
            Display::SetTheme(theme);
        }
    }

private:
    enum View { kViewHome, kViewMenu, kViewChat };

    View view_ = kViewHome;
    lv_obj_t* chat_view_ = nullptr;
    lv_obj_t* home_view_ = nullptr;
    lv_obj_t* menu_box_ = nullptr;
    lv_obj_t* big_face_ = nullptr;
    lv_obj_t* home_title_ = nullptr;
    lv_obj_t* home_hint_ = nullptr;
    lv_obj_t* home_info_ = nullptr;
    lv_obj_t* mic_btn_ = nullptr;
    lv_obj_t* volume_label_ = nullptr;
    lv_obj_t* avatar_ = nullptr;
    lv_obj_t* system_row_ = nullptr;
    lv_timer_t* anim_timer_ = nullptr;
    const lv_image_dsc_t* base_face_ = nullptr;
    const lv_image_dsc_t* shown_face_ = nullptr;
    const lv_image_dsc_t* shown_big_ = nullptr;
    std::string pending_prompt_;
    uint32_t stop_ticks_ = 0;  // > 0 while the HOME button is stopping the conversation
    int last_story_ = -1;
    int last_song_ = -1;
    uint32_t pending_ticks_ = 0;
    uint32_t tick_ = 0;
    uint32_t idle_ticks_ = 0;
    uint32_t poke_ticks_ = 0;
    int last_state_ = -1;

    /* ---------- small widget helpers ---------- */
    lv_obj_t* MakePanel(lv_obj_t* parent, int x, int y, int w, int h, uint32_t color) {
        lv_obj_t* p = lv_obj_create(parent);
        lv_obj_remove_style_all(p);
        lv_obj_set_size(p, w, h);
        lv_obj_set_pos(p, x, y);
        lv_obj_set_style_bg_opa(p, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(p, lv_color_hex(color), 0);
        lv_obj_remove_flag(p, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_remove_flag(p, LV_OBJ_FLAG_CLICKABLE);
        return p;
    }

    lv_obj_t* MakeIcon(lv_obj_t* parent, const lv_font_t* font, uint32_t color, int x_ofs) {
        lv_obj_t* l = lv_label_create(parent);
        lv_label_set_text(l, "");
        lv_obj_set_style_text_font(l, font, 0);
        lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
        lv_obj_align(l, LV_ALIGN_TOP_RIGHT, x_ofs, 3);
        return l;
    }

    lv_obj_t* MakeButton(lv_obj_t* parent, const lv_image_dsc_t* icon, int x, int y, int w, int h,
                         lv_color_t color, lv_event_cb_t cb, lv_event_code_t code) {
        lv_obj_t* btn = lv_button_create(parent);
        lv_obj_remove_style_all(btn);
        lv_obj_set_size(btn, w, h);
        lv_obj_set_pos(btn, x, y);
        lv_obj_set_style_radius(btn, 14, 0);
        lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(btn, color, 0);
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x0E1E46), LV_STATE_PRESSED);
        lv_obj_set_style_border_width(btn, 2, 0);
        lv_obj_set_style_border_color(btn, lv_color_white(), 0);
        lv_obj_set_style_border_opa(btn, LV_OPA_40, 0);
        lv_obj_remove_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_ext_click_area(btn, 4);

        lv_obj_t* img = lv_image_create(btn);
        lv_image_set_src(img, icon);
        lv_obj_center(img);
        lv_obj_remove_flag(img, LV_OBJ_FLAG_CLICKABLE);

        lv_obj_add_event_cb(btn, cb, code, this);
        return btn;
    }

    void PlaceStatus(lv_obj_t* parent, int x, int y, int w) {
        if (status_bar_ == nullptr) return;
        lv_obj_set_parent(status_bar_, parent);
        lv_obj_set_size(status_bar_, w, 28);
        lv_obj_set_pos(status_bar_, x, y);
        lv_obj_set_width(status_label_, w - 14);
        lv_obj_set_width(notification_label_, w - 14);
        lv_obj_center(status_label_);
        lv_obj_center(notification_label_);
        lv_obj_move_foreground(status_bar_);
    }

    /* ---------- views ---------- */
    void ShowHome() {
        const int main_w = LV_HOR_RES - NY_SIDE_W;
        view_ = kViewHome;
        lv_obj_add_flag(chat_view_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(home_view_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(menu_box_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(big_face_, (main_w - (int)nhuy_big_neutral.header.w) / 2, 84);
        lv_obj_remove_flag(home_title_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(home_hint_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(home_info_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(home_title_, (main_w - (int)nhuy_title.header.w) / 2 + 22, 250);
        lv_obj_set_pos(home_hint_, (main_w - (int)nhuy_hint.header.w) / 2, 292);
        PlaceStatus(home_view_, 24, 332, main_w - 48);
        lv_obj_set_pos(home_info_, 10, 376);
        idle_ticks_ = 0;
        shown_big_ = nullptr;
        ApplyFace(base_face_);
    }

    void ShowMenu() {
        const int main_w = LV_HOR_RES - NY_SIDE_W;
        view_ = kViewMenu;
        lv_obj_add_flag(chat_view_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(home_view_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(menu_box_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_set_pos(big_face_, (main_w - (int)nhuy_big_neutral.header.w) / 2, 6);
        lv_obj_add_flag(home_title_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(home_hint_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(home_info_, LV_OBJ_FLAG_HIDDEN);
        PlaceStatus(home_view_, 24, 190, main_w - 48);
        idle_ticks_ = 0;
        poke_ticks_ = 10;
        ApplyFace(base_face_);
    }

    void ShowChat() {
        const int main_w = LV_HOR_RES - NY_SIDE_W;
        view_ = kViewChat;
        lv_obj_add_flag(home_view_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_remove_flag(chat_view_, LV_OBJ_FLAG_HIDDEN);
        PlaceStatus(top_bar_, 108, NY_HEADER_H - 34, main_w - 112);
        idle_ticks_ = 0;
        shown_face_ = nullptr;
        ApplyFace(base_face_);
    }

    /* ---------- noise filter ---------- */
    // Speech recognition sometimes turns silence/noise into phrases like "Lala School".
    static bool IsNoiseTranscript(const char* text) {
        std::string t;
        for (const char* p = text; *p; p++) {
            char c = *p;
            if (c >= 'A' && c <= 'Z') c = c - 'A' + 'a';
            if (c == ' ' || c == '.' || c == ',' || c == '!' || c == '-') continue;
            t += c;
        }
        static const char* const noise[] = {"lalaschool", "amara.org", "amaraorg", "subtitle",
                                            "字幕", "谢谢观看", "请不吝点赞", "订阅"};
        for (auto n : noise) {
            if (t.find(n) != std::string::npos) return true;
        }
        return false;
    }

    /* ---------- avatar ---------- */
    static const lv_image_dsc_t* BigOf(const lv_image_dsc_t* small) {
        if (small == &nhuy_face_neutral) return &nhuy_big_neutral;
        if (small == &nhuy_face_blink) return &nhuy_big_blink;
        if (small == &nhuy_face_happy) return &nhuy_big_happy;
        if (small == &nhuy_face_happy_talk) return &nhuy_big_happy_talk;
        if (small == &nhuy_face_talk || small == &nhuy_face_surprised) return &nhuy_big_talk;
        if (small == &nhuy_face_listen) return &nhuy_big_listen;
        return nullptr;
    }

    void ApplyFace(const lv_image_dsc_t* face) {
        if (face == nullptr) return;
        if (view_ == kViewChat) {
            if (avatar_ != nullptr && face != shown_face_) {
                lv_image_set_src(avatar_, face);
                shown_face_ = face;
            }
        } else if (big_face_ != nullptr && face != shown_big_) {
            const lv_image_dsc_t* big = BigOf(face);
            if (big != nullptr) {
                lv_image_set_src(big_face_, big);
                lv_image_set_scale(big_face_, LV_SCALE_NONE);
            } else {
                // No large version: draw the small frame enlarged to the same size.
                lv_image_set_src(big_face_, face);
                lv_image_set_pivot(big_face_, 0, 0);
                lv_image_set_scale(big_face_, LV_SCALE_NONE * nhuy_big_neutral.header.w /
                                                  nhuy_face_neutral.header.w);
            }
            shown_big_ = face;
        }
    }

    static const lv_image_dsc_t* MapEmotion(const char* e) {
        if (e == nullptr) return &nhuy_face_neutral;
        static const char* const happy[] = {"happy", "laughing", "funny", "loving", "winking",
                                            "delicious", "kissy", "silly", "cool", "confident"};
        static const char* const sad[] = {"sad", "crying", "angry"};
        static const char* const surprised[] = {"surprised", "shocked"};
        static const char* const think[] = {"thinking", "confused"};
        for (auto s : happy) if (strcmp(e, s) == 0) return &nhuy_face_happy;
        for (auto s : sad) if (strcmp(e, s) == 0) return &nhuy_face_sad;
        for (auto s : surprised) if (strcmp(e, s) == 0) return &nhuy_face_surprised;
        for (auto s : think) if (strcmp(e, s) == 0) return &nhuy_face_think;
        if (strcmp(e, "sleepy") == 0) return &nhuy_face_sleepy;
        return &nhuy_face_neutral;
    }

    /* ---------- volume ---------- */
    void UpdateVolumeLabel() {
        if (volume_label_ == nullptr) return;
        auto codec = Board::GetInstance().GetAudioCodec();
        if (codec == nullptr) return;
        lv_label_set_text_fmt(volume_label_, "%d%%", codec->output_volume());
    }

    void ChangeVolume(int delta) {
        auto codec = Board::GetInstance().GetAudioCodec();
        if (codec == nullptr) return;
        int v = codec->output_volume() + delta;
        if (v < 0) v = 0;
        if (v > 100) v = 100;
        codec->SetOutputVolume(v);
        UpdateVolumeLabel();
    }

    /* ---------- modes ---------- */
    void StartMode(const std::string& prompt) {
        ShowChat();
        pending_prompt_ = prompt;
        pending_ticks_ = 0;
        RunPending();
    }

    // Invoke the pending prompt; if a conversation is running, WakeWordInvoke first stops it
    // and we try again on a later tick until the device is idle.
    void RunPending() {
        if (pending_prompt_.empty()) return;
        auto& app = Application::GetInstance();
        DeviceState state = app.GetDeviceState();
        if (state == kDeviceStateIdle) {
            app.WakeWordInvoke(pending_prompt_);
            pending_prompt_.clear();
        } else if (pending_ticks_ % 8 == 0 &&
                   (state == kDeviceStateSpeaking || state == kDeviceStateListening)) {
            app.WakeWordInvoke(pending_prompt_);  // stops speaking / closes listening
        }
        if (++pending_ticks_ > 80) {
            pending_prompt_.clear();  // give up after ~12 s
        }
    }

    /* ---------- HOME: stop everything and go to the home face ---------- */
    void StopAndGoHome() {
        pending_prompt_.clear();
        stop_ticks_ = 1;
        RunStop(Application::GetInstance().GetDeviceState());
        ClearChatMessagesUnlocked();
        ShowHome();
    }

    // Speaking (answer or music) -> abort; listening -> close the audio channel.
    // Repeated every ~0.6 s until the device is idle (max ~6 s).
    void RunStop(DeviceState state) {
        auto& app = Application::GetInstance();
        if (state == kDeviceStateIdle || stop_ticks_ > 40) {
            stop_ticks_ = 0;
            return;
        }
        if ((stop_ticks_ - 1) % 4 == 0) {
            if (state == kDeviceStateSpeaking) {
                app.Schedule([]() { Application::GetInstance().AbortSpeaking(kAbortReasonNone); });
            } else if (state == kDeviceStateListening) {
                app.ToggleChatState();
            }
        }
        stop_ticks_++;
    }

    void ClearChatMessagesUnlocked() {
        if (content_ != nullptr) {
            lv_obj_clean(content_);
        }
        system_row_ = nullptr;
    }

    /* ---------- animation tick (LVGL task, every 150 ms) ---------- */
    void Animate() {
        tick_++;
        DeviceState state = Application::GetInstance().GetDeviceState();
        if (stop_ticks_ > 0) {
            RunStop(state);
        } else if (!pending_prompt_.empty()) {
            RunPending();
        }

        const lv_image_dsc_t* face = base_face_;
        if (state == kDeviceStateSpeaking) {
            bool open = (tick_ % 2) == 0;
            if (base_face_ == &nhuy_face_happy) {
                face = open ? &nhuy_face_happy : &nhuy_face_happy_talk;
            } else if (base_face_ == &nhuy_face_neutral || base_face_ == &nhuy_face_surprised) {
                face = open ? &nhuy_face_talk : &nhuy_face_neutral;
            }
        } else if (state == kDeviceStateListening) {
            face = &nhuy_face_listen;
        } else if (state == kDeviceStateConnecting) {
            face = &nhuy_face_think;
        } else if (poke_ticks_ > 0) {
            face = &nhuy_face_happy;
        } else if (base_face_ == &nhuy_face_neutral && (tick_ % 27) == 0) {
            face = &nhuy_face_blink;
        }
        if (poke_ticks_ > 0) poke_ticks_--;
        ApplyFace(face);

        // Back to the home face after 60 s without any conversation.
        if (view_ != kViewHome && state == kDeviceStateIdle && pending_prompt_.empty()) {
            if (++idle_ticks_ > 400) {
                ShowHome();
            }
        } else {
            idle_ticks_ = 0;
        }

        if ((int)state != last_state_) {
            last_state_ = (int)state;
            lv_color_t c = lv_color_hex(0x3C8CFF);
            if (state == kDeviceStateListening) c = lv_color_hex(0x1EB45A);
            else if (state == kDeviceStateSpeaking) c = lv_color_hex(0xF08C28);
            else if (state == kDeviceStateConnecting) c = lv_color_hex(0x8C64DC);
            if (mic_btn_ != nullptr) lv_obj_set_style_bg_color(mic_btn_, c, 0);
            UpdateVolumeLabel();
        }
    }

    /* ---------- event callbacks ---------- */
    static NhuYDisplay* Self(lv_event_t* e) {
        return static_cast<NhuYDisplay*>(lv_event_get_user_data(e));
    }
    static void AnimTick(lv_timer_t* t) {
        static_cast<NhuYDisplay*>(lv_timer_get_user_data(t))->Animate();
    }
    static void OnMic(lv_event_t* e) {
        auto self = Self(e);
        if (self->view_ != kViewChat) self->ShowChat();
        Application::GetInstance().ToggleChatState();
    }
    static void OnVolUp(lv_event_t* e) { Self(e)->ChangeVolume(10); }
    static void OnVolDown(lv_event_t* e) { Self(e)->ChangeVolume(-10); }
    static void OnReset(lv_event_t* e) {
        (void)e;
        ESP_LOGW(TAG, "Reset button pressed - restarting");
        esp_restart();
    }
    static void OnFaceTap(lv_event_t* e) {
        auto self = Self(e);
        if (self->view_ == kViewHome) self->ShowMenu();
        else self->poke_ticks_ = 10;
    }
    static void OnBackHome(lv_event_t* e) { Self(e)->ShowHome(); }
    static void OnHome(lv_event_t* e) { Self(e)->StopAndGoHome(); }
    static int PickRandom(int count, int& last) {
        int n = (int)(esp_random() % (uint32_t)count);
        if (n == last) n = (n + 1) % count;
        last = n;
        return n;
    }
    static void OnStory(lv_event_t* e) {
        static const char* const stories[] = {
            "The Little Red Kite", "The Hungry Cat", "Ben's First Day at School",
            "The Ant and the Leaf", "The Magic Pencil", "The Lazy Rooster", "The Lost Puppy",
            "The Tortoise and the Rabbit", "A Gift for Mom", "The Brave Little Fish",
            "The Robot Who Wanted to Dance", "The Moon and the Lantern", "The Honest Woodcutter",
            "The Seed That Waited", "The Elephant Who Was Afraid of Mice", "The Rainy Day Picnic",
            "The Star Who Fell Down", "The Boy Who Cried Wolf", "The Clever Crow",
            "The Kind Snowman"};
        auto self = Self(e);
        int n = PickRandom(sizeof(stories) / sizeof(stories[0]), self->last_story_);
        self->StartMode(std::string("Như Ý ơi, kể cho mình nghe Truyện số ") +
                        std::to_string(n + 1) + ": \"" + stories[n] +
                        "\" trong thư viện truyện tiếng Anh nhé. Đọc bằng tiếng Anh thật chậm "
                        "và rõ, rồi nói ý nghĩa bằng tiếng Việt, dạy 3 từ vựng và hỏi mình "
                        "câu hỏi của truyện.");
    }
    static void OnLearn(lv_event_t* e) {
        Self(e)->StartMode(
            "Như Ý ơi, mình muốn học tiếng Anh. Dạy mình 3 từ vựng mới kèm câu ví dụ đơn giản, "
            "rồi mời mình đọc lại từng từ nhé.");
    }
    static void OnMusic(lv_event_t* e) {
        static const char* const songs[] = {
            "Twinkle Twinkle Little Star", "Old MacDonald Had a Farm", "The Wheels on the Bus",
            "Row Row Row Your Boat", "If You're Happy and You Know It",
            "Head Shoulders Knees and Toes", "London Bridge Is Falling Down",
            "Mary Had a Little Lamb", "Baby Shark - Pinkfong", "You Are My Sunshine",
            "Happy Birthday to You", "Jingle Bells", "We Wish You a Merry Christmas",
            "Count on Me - Bruno Mars", "Happy - Pharrell Williams", "Let It Go - Idina Menzel",
            "Try Everything - Shakira", "A Whole New World - Aladdin",
            "Yellow Submarine - The Beatles", "What a Wonderful World - Louis Armstrong"};
        auto self = Self(e);
        int n = PickRandom(sizeof(songs) / sizeof(songs[0]), self->last_song_);
        self->StartMode(std::string("Phát nhạc: ") + songs[n] +
                        ". Như Ý ơi, hãy dùng ngay công cụ phát nhạc để phát bài hát tiếng Anh \"" +
                        songs[n] + "\" cho mình nghe. Đây là yêu cầu nghe nhạc, không phải kể "
                        "chuyện, đừng kể chuyện.");
    }
};

// --------------------------------------------------------------------------
//  Board
// --------------------------------------------------------------------------
class CompactWifiBoardLCD : public WifiBoard {
private:
    Button boot_button_;
    LcdDisplay* display_ = nullptr;
    Xpt2046 touch_;
    bool touch_ok_ = false;
    bool cal_swap_ = false;
    int cal_x0_ = 0, cal_x1_ = 4095, cal_y0_ = 0, cal_y1_ = 4095;
    lv_point_t last_point_ = {0, 0};

    static constexpr int kCalMargin = 30;

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_PIN;
        buscfg.miso_io_num = TOUCH_MISO_PIN;
        buscfg.sclk_io_num = DISPLAY_CLK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeLcdDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = DISPLAY_SPI_MODE;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = DISPLAY_RST_PIN;
        panel_config.rgb_ele_order = DISPLAY_RGB_ORDER;
        panel_config.bits_per_pixel = 16;
#if defined(LCD_TYPE_ILI9341_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(panel_io, &panel_config, &panel));
#elif defined(LCD_TYPE_GC9A01_SERIAL)
        ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_io, &panel_config, &panel));
        gc9a01_vendor_config_t gc9107_vendor_config = {
            .init_cmds = gc9107_lcd_init_cmds,
            .init_cmds_size = sizeof(gc9107_lcd_init_cmds) / sizeof(gc9a01_lcd_init_cmd_t),
        };
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
#endif

        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
#ifdef LCD_TYPE_GC9A01_SERIAL
        panel_config.vendor_config = &gc9107_vendor_config;
#endif
        display_ = new NhuYDisplay(panel_io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                   DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X,
                                   DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    // MCP test tool: control a lamp
    void InitializeTools() {
        static LampController lamp(LAMP_GPIO);
    }

    /* ---------------- Touch ---------------- */
    bool LoadCalibration() {
        Settings s("touch", false);
        if (s.GetInt("ok", 0) != 1) {
            return false;
        }
        cal_swap_ = s.GetInt("sw", 0) != 0;
        cal_x0_ = s.GetInt("x0", 0);
        cal_x1_ = s.GetInt("x1", 4095);
        cal_y0_ = s.GetInt("y0", 0);
        cal_y1_ = s.GetInt("y1", 4095);
        return std::abs(cal_x1_ - cal_x0_) > 300 && std::abs(cal_y1_ - cal_y0_) > 300;
    }

    void SaveCalibration() {
        Settings s("touch", true);
        s.SetInt("sw", cal_swap_ ? 1 : 0);
        s.SetInt("x0", cal_x0_);
        s.SetInt("x1", cal_x1_);
        s.SetInt("y0", cal_y0_);
        s.SetInt("y1", cal_y1_);
        s.SetInt("ok", 1);
    }

    // Waits for one touch and returns its median raw value. False on timeout.
    bool CaptureTouch(int& a, int& b, int timeout_ms) {
        int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
        while (touch_.Touched()) {
            if (esp_timer_get_time() > deadline) return false;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        vTaskDelay(pdMS_TO_TICKS(250));
        while (!touch_.Touched()) {
            if (esp_timer_get_time() > deadline) return false;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        vTaskDelay(pdMS_TO_TICKS(60));
        int sa[16], sb[16], n = 0;
        int64_t end = esp_timer_get_time() + 700 * 1000;
        while (n < 16 && esp_timer_get_time() < end) {
            int ra, rb;
            if (touch_.ReadRaw(ra, rb)) {
                sa[n] = ra;
                sb[n] = rb;
                n++;
            }
            vTaskDelay(pdMS_TO_TICKS(15));
        }
        if (n < 4) return false;
        std::sort(sa, sa + n);
        std::sort(sb, sb + n);
        a = sa[n / 2];
        b = sb[n / 2];
        return true;
    }

    bool RunCalibration() {
        const int W = DISPLAY_WIDTH, H = DISPLAY_HEIGHT, M = kCalMargin;
        const int px[3] = {M, W - M, M};
        const int py[3] = {M, M, H - M};

        lvgl_port_lock(0);
        lv_obj_t* panel = lv_obj_create(lv_layer_top());
        lv_obj_remove_style_all(panel);
        lv_obj_set_size(panel, W, H);
        lv_obj_set_style_bg_opa(panel, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(panel, lv_color_white(), 0);
        lv_obj_t* txt = lv_image_create(panel);
        lv_image_set_src(txt, &nhuy_calib_text);
        lv_obj_align(txt, LV_ALIGN_CENTER, 0, 0);
        lv_obj_t* hl = lv_obj_create(panel);
        lv_obj_remove_style_all(hl);
        lv_obj_set_size(hl, 31, 3);
        lv_obj_set_style_bg_opa(hl, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(hl, lv_color_hex(0xDC1E28), 0);
        lv_obj_t* vl = lv_obj_create(panel);
        lv_obj_remove_style_all(vl);
        lv_obj_set_size(vl, 3, 31);
        lv_obj_set_style_bg_opa(vl, LV_OPA_COVER, 0);
        lv_obj_set_style_bg_color(vl, lv_color_hex(0xDC1E28), 0);
        lvgl_port_unlock();

        bool ok = false;
        for (int attempt = 0; attempt < 2 && !ok; attempt++) {
            int ra[3], rb[3];
            bool got_all = true;
            for (int i = 0; i < 3; i++) {
                lvgl_port_lock(0);
                lv_obj_set_pos(hl, px[i] - 15, py[i] - 1);
                lv_obj_set_pos(vl, px[i] - 1, py[i] - 15);
                lvgl_port_unlock();
                if (!CaptureTouch(ra[i], rb[i], 30000)) {
                    got_all = false;
                    break;
                }
                ESP_LOGI(TAG, "Touch calib point %d (%d,%d): raw a=%d b=%d", i, px[i], py[i],
                         ra[i], rb[i]);
            }
            if (!got_all) {
                break;  // nobody touched: give up for this boot
            }
            int da = std::abs(ra[1] - ra[0]);
            int db = std::abs(rb[1] - rb[0]);
            cal_swap_ = db > da;
            cal_x0_ = cal_swap_ ? rb[0] : ra[0];
            cal_x1_ = cal_swap_ ? rb[1] : ra[1];
            cal_y0_ = cal_swap_ ? ra[0] : rb[0];
            cal_y1_ = cal_swap_ ? ra[2] : rb[2];
            ok = std::abs(cal_x1_ - cal_x0_) > 300 && std::abs(cal_y1_ - cal_y0_) > 300;
            ESP_LOGI(TAG, "Touch calib %s: swap=%d x0=%d x1=%d y0=%d y1=%d", ok ? "OK" : "BAD",
                     cal_swap_, cal_x0_, cal_x1_, cal_y0_, cal_y1_);
        }
        while (touch_.Touched()) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        lvgl_port_lock(0);
        lv_obj_delete(panel);
        lvgl_port_unlock();

        if (ok) {
            SaveCalibration();
        }
        return ok;
    }

    bool MapPoint(int a, int b, lv_point_t& p) {
        int rx = cal_swap_ ? b : a;
        int ry = cal_swap_ ? a : b;
        const int W = DISPLAY_WIDTH, H = DISPLAY_HEIGHT, M = kCalMargin;
        int x = M + (int)((long)(rx - cal_x0_) * (W - 2 * M) / (cal_x1_ - cal_x0_));
        int y = M + (int)((long)(ry - cal_y0_) * (H - 2 * M) / (cal_y1_ - cal_y0_));
        p.x = std::clamp(x, 0, W - 1);
        p.y = std::clamp(y, 0, H - 1);
        return true;
    }

    static void TouchRead(lv_indev_t* indev, lv_indev_data_t* data) {
        auto self = static_cast<CompactWifiBoardLCD*>(lv_indev_get_user_data(indev));
        int a, b;
        if (self->touch_.ReadRaw(a, b)) {
            self->MapPoint(a, b, self->last_point_);
            data->state = LV_INDEV_STATE_PRESSED;
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
        }
        data->point = self->last_point_;
    }

    void InitializeTouch() {
        if (!touch_.Init(SPI3_HOST)) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        bool have_cal = LoadCalibration();
        // No calibration yet, or finger held on the screen at power-up: calibrate.
        if (!have_cal || touch_.Touched()) {
            have_cal = RunCalibration() || have_cal;
        }
        if (!have_cal) {
            ESP_LOGW(TAG, "Touch not calibrated - on-screen buttons disabled this boot");
            return;
        }
        lvgl_port_lock(0);
        lv_indev_t* indev = lv_indev_create();
        lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
        lv_indev_set_read_cb(indev, TouchRead);
        lv_indev_set_user_data(indev, this);
        lv_indev_set_disp(indev, lv_display_get_default());
        lv_timer_set_period(lv_indev_get_read_timer(indev), 10);  // poll touch every 10 ms
        lvgl_port_unlock();
        touch_ok_ = true;
        ESP_LOGI(TAG, "Touch ready");
    }

public:
    CompactWifiBoardLCD() :
        boot_button_(BOOT_BUTTON_GPIO) {
        InitializeSpi();
        InitializeLcdDisplay();
        InitializeButtons();
        InitializeTools();
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            GetBacklight()->RestoreBrightness();
        }
        InitializeTouch();
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC) {
            static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
            return &backlight;
        }
        return nullptr;
    }
};

DECLARE_BOARD(CompactWifiBoardLCD);
