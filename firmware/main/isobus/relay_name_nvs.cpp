#include "isobus/relay_name_nvs.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <string>

#include "esp_log.h"
#include "nvs.h"

namespace iso::vt_app {
namespace {

constexpr const char *kTag = "relay_name_nvs";
constexpr const char *kNamespace = "relay_name";
constexpr std::array<const char *, RelayNameNVSRepository::kChannelCount + 1> kKeys = {
	"",
	"ch1",
	"ch2",
	"ch3",
	"ch4",
	"ch5",
	"ch6",
	"ch7",
	"ch8",
};

bool is_safe_char(char c)
{
	const unsigned char uc = static_cast<unsigned char>(c);
	return std::isalnum(uc) || (c == '_') || (c == '-') || (c == ' ');
}

} // namespace

std::string RelayNameNVSRepository::default_name(int channel)
{
	if ((channel < 1) || (channel > kChannelCount))
	{
		return "R?";
	}
	return "R" + std::to_string(channel);
}

std::string RelayNameNVSRepository::sanitize_name(const std::string &value, int channel)
{
	std::string sanitized;
	sanitized.reserve(value.size());
	for (const char c : value)
	{
		if (is_safe_char(c))
		{
			sanitized.push_back(c);
		}
	}

	auto firstNonSpace = sanitized.find_first_not_of(' ');
	if (firstNonSpace == std::string::npos)
	{
		sanitized.clear();
	}
	else
	{
		const auto lastNonSpace = sanitized.find_last_not_of(' ');
		sanitized = sanitized.substr(firstNonSpace, (lastNonSpace - firstNonSpace) + 1);
	}

	if (sanitized.empty())
	{
		sanitized = default_name(channel);
	}

	if (sanitized.size() > kMaxRelayNameChars)
	{
		sanitized.resize(kMaxRelayNameChars);
	}

	return sanitized;
}

bool RelayNameNVSRepository::load(std::array<std::string, kChannelCount + 1> &names) const
{
	for (int channel = 1; channel <= kChannelCount; ++channel)
	{
		names[channel] = default_name(channel);
	}

	nvs_handle_t handle;
	esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(kTag, "nvs_open failed: %s", esp_err_to_name(err));
		return false;
	}

	for (int channel = 1; channel <= kChannelCount; ++channel)
	{
		size_t required = 0;
		err = nvs_get_str(handle, kKeys[channel], nullptr, &required);
		if (err == ESP_ERR_NVS_NOT_FOUND)
		{
			continue;
		}
		if (err != ESP_OK)
		{
			ESP_LOGE(kTag, "nvs_get_str size failed for channel %d: %s", channel, esp_err_to_name(err));
			nvs_close(handle);
			return false;
		}
		if (required == 0)
		{
			continue;
		}

		std::string value(required, '\0');
		err = nvs_get_str(handle, kKeys[channel], value.data(), &required);
		if (err != ESP_OK)
		{
			ESP_LOGE(kTag, "nvs_get_str failed for channel %d: %s", channel, esp_err_to_name(err));
			nvs_close(handle);
			return false;
		}
		if (!value.empty() && value.back() == '\0')
		{
			value.pop_back();
		}
		names[channel] = sanitize_name(value, channel);
	}

	nvs_close(handle);
	return true;
}

bool RelayNameNVSRepository::store(int channel, const std::string &name) const
{
	if ((channel < 1) || (channel > kChannelCount))
	{
		return false;
	}

	nvs_handle_t handle;
	esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(kTag, "nvs_open failed: %s", esp_err_to_name(err));
		return false;
	}

	const std::string sanitized = sanitize_name(name, channel);
	err = nvs_set_str(handle, kKeys[channel], sanitized.c_str());
	if (err == ESP_OK)
	{
		err = nvs_commit(handle);
	}
	nvs_close(handle);

	if (err != ESP_OK)
	{
		ESP_LOGE(kTag, "failed to store relay name for channel %d: %s", channel, esp_err_to_name(err));
		return false;
	}
	return true;
}

bool RelayNameNVSRepository::clear_all() const
{
	nvs_handle_t handle;
	esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(kTag, "nvs_open failed: %s", esp_err_to_name(err));
		return false;
	}

	for (int channel = 1; channel <= kChannelCount; ++channel)
	{
		esp_err_t eraseErr = nvs_erase_key(handle, kKeys[channel]);
		if ((eraseErr != ESP_OK) && (eraseErr != ESP_ERR_NVS_NOT_FOUND))
		{
			ESP_LOGE(kTag, "failed to erase key for channel %d: %s", channel, esp_err_to_name(eraseErr));
			nvs_close(handle);
			return false;
		}
	}

	err = nvs_commit(handle);
	nvs_close(handle);

	if (err != ESP_OK)
	{
		ESP_LOGE(kTag, "nvs_commit failed while clearing relay names: %s", esp_err_to_name(err));
		return false;
	}
	return true;
}

} // namespace iso::vt_app
