#include "io/cytron_md13s.hpp"

#include "driver/ledc.h"
#include "esp_log.h"

namespace io::cytron_md13s {

namespace {
constexpr const char* kTag = "cytron_md13s";
constexpr gpio_num_t kPwmPin = GPIO_NUM_48;
constexpr ledc_timer_t kTimer = LEDC_TIMER_1;
constexpr ledc_channel_t kChannel = LEDC_CHANNEL_1;
constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_bit_t kDutyResolution = LEDC_TIMER_10_BIT;  // 0-1023
constexpr uint32_t kPwmFrequencyHz = 20000;
constexpr uint32_t kMaxDuty = (1u << kDutyResolution) - 1u;

bool g_initialized = false;
uint8_t g_speed_percent = 0;
}  // namespace

void init() {
    ledc_timer_config_t timer_cfg = {};
    timer_cfg.speed_mode = kSpeedMode;
    timer_cfg.duty_resolution = kDutyResolution;
    timer_cfg.timer_num = kTimer;
    timer_cfg.freq_hz = kPwmFrequencyHz;
    timer_cfg.clk_cfg = LEDC_AUTO_CLK;
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return;
    }

    ledc_channel_config_t channel_cfg = {};
    channel_cfg.gpio_num = kPwmPin;
    channel_cfg.speed_mode = kSpeedMode;
    channel_cfg.channel = kChannel;
    channel_cfg.timer_sel = kTimer;
    channel_cfg.duty = 0;  // safe default: stopped
    channel_cfg.hpoint = 0;
    err = ledc_channel_config(&channel_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "ledc_channel_config failed: %s", esp_err_to_name(err));
        return;
    }

    g_initialized = true;
    g_speed_percent = 0;
}

bool set_speed_percent(uint8_t percent) {
    if (!g_initialized || percent > 100) {
        return false;
    }
    const uint32_t duty = (static_cast<uint32_t>(percent) * kMaxDuty + 50u) / 100u;
    esp_err_t err = ledc_set_duty(kSpeedMode, kChannel, duty);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "ledc_set_duty failed: %s", esp_err_to_name(err));
        return false;
    }
    err = ledc_update_duty(kSpeedMode, kChannel);
    if (err != ESP_OK) {
        ESP_LOGE(kTag, "ledc_update_duty failed: %s", esp_err_to_name(err));
        return false;
    }
    g_speed_percent = percent;
    ESP_LOGI(kTag, "MD13S PWM -> %u%% (GPIO48)", static_cast<unsigned>(percent));
    return true;
}

uint8_t get_speed_percent() {
    return g_speed_percent;
}

}  // namespace io::cytron_md13s
