#include "config/nvs_store.hpp"

#include <array>
#include <cctype>
#include <cstdlib>
#include <cstdio>
#include <mutex>
#include <string>

#include "config/nvs_write_queue.hpp"
#include "esp_log.h"
#include "nvs.h"

namespace config::nvs_store {

namespace {
constexpr const char* kTag = "nvs_store";
constexpr const char* kNvsNamespace = "channels";
constexpr const char* kMd13sPwmKey = "md13s_pwm";

std::array<std::string, kChannelCount> g_names;
int g_md13s_pwm_percent = 0;
std::mutex g_mutex;

std::string default_name(int channel) {
    return "R" + std::to_string(channel);
}

std::string normalize_name(int channel, const std::string& value) {
    std::string normalized;
    normalized.reserve(kChannelNameMaxChars);
    for (unsigned char character : value) {
        if (character < 32 || character > 126) {
            continue;
        }
        normalized.push_back(static_cast<char>(character));
    }

    size_t first = 0;
    while (first < normalized.size() && std::isspace(static_cast<unsigned char>(normalized[first]))) {
        ++first;
    }
    size_t last = normalized.size();
    while (last > first && std::isspace(static_cast<unsigned char>(normalized[last - 1]))) {
        --last;
    }
    normalized = normalized.substr(first, last - first);
    if (normalized.size() > kChannelNameMaxChars) {
        normalized.resize(kChannelNameMaxChars);
        while (!normalized.empty() && std::isspace(static_cast<unsigned char>(normalized.back()))) {
            normalized.pop_back();
        }
    }
    return normalized.empty() ? default_name(channel) : normalized;
}

}  // namespace

void init() {
    for (int channel = 1; channel <= kChannelCount; ++channel) {
        g_names[channel - 1] = default_name(channel);
    }

    nvs_write_queue::init();

    nvs_handle_t handle;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
    if (ESP_OK != err) {
        ESP_LOGE(kTag, "nvs_open failed (%s), using default channel names", esp_err_to_name(err));
        return;
    }

    for (int channel = 1; channel <= kChannelCount; ++channel) {
        char key[8];
        char stored[kChannelNameMaxChars + 1] = {};
        std::snprintf(key, sizeof(key), "name%d", channel);
        size_t required_size = sizeof(stored);
        err = nvs_get_str(handle, key, stored, &required_size);
        if (ESP_OK == err) {
            g_names[channel - 1] = normalize_name(channel, stored);
        } else if (ESP_ERR_NVS_NOT_FOUND != err) {
            ESP_LOGW(kTag, "Unable to load channel %d name (%s), using default", channel, esp_err_to_name(err));
        }
    }

    char pwm_value[4] = {};
    size_t pwm_size = sizeof(pwm_value);
    err = nvs_get_str(handle, kMd13sPwmKey, pwm_value, &pwm_size);
    if (ESP_OK == err) {
        int parsed = std::atoi(pwm_value);
        if (parsed >= 0 && parsed <= 100) {
            g_md13s_pwm_percent = parsed;
        } else {
            ESP_LOGW(kTag, "Invalid stored MD13S PWM percent \"%s\", using 0", pwm_value);
            g_md13s_pwm_percent = 0;
        }
    } else if (ESP_ERR_NVS_NOT_FOUND != err) {
        ESP_LOGW(kTag, "Unable to load MD13S PWM percent (%s), using 0", esp_err_to_name(err));
    }
    nvs_close(handle);
}

std::string get_channel_name(int channel) {
    if (channel < 1 || channel > kChannelCount) {
        return {};
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_names[channel - 1];
}

bool set_channel_name(int channel, const std::string& value) {
    if (channel < 1 || channel > kChannelCount) {
        return false;
    }
    const std::string name = normalize_name(channel, value);

    char key[8];
    std::snprintf(key, sizeof(key), "name%d", channel);
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!nvs_write_queue::enqueue_string(kNvsNamespace, key, name)) {
        ESP_LOGE(kTag, "Channel %d name was not queued for persistence", channel);
        return false;
    }
    g_names[channel - 1] = name;
    return true;
}

int get_md13s_pwm_percent() {
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_md13s_pwm_percent;
}

bool set_md13s_pwm_percent(int percent) {
    if (percent < 0 || percent > 100) {
        return false;
    }

    const std::string value = std::to_string(percent);
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!nvs_write_queue::enqueue_string(kNvsNamespace, kMd13sPwmKey, value)) {
        ESP_LOGE(kTag, "MD13S PWM percent was not queued for persistence");
        return false;
    }
    g_md13s_pwm_percent = percent;
    return true;
}

}  // namespace config::nvs_store
