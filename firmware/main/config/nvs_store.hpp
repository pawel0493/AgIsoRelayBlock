#pragma once

#include <cstddef>
#include <string>

namespace config::nvs_store {

constexpr int kChannelCount = 8;
constexpr std::size_t kChannelNameMaxChars = 8;

void init();
std::string get_channel_name(int channel);
bool set_channel_name(int channel, const std::string& name);

}  // namespace config::nvs_store
