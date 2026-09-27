#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace iso::vt_app {

class RelayNameNVSRepository final
{
public:
	static constexpr int kChannelCount = 8;
	static constexpr std::size_t kMaxRelayNameChars = 3;

	bool load(std::array<std::string, kChannelCount + 1> &names) const;
	bool store(int channel, const std::string &name) const;
	bool clear_all() const;

	static std::string default_name(int channel);
	static std::string sanitize_name(const std::string &value, int channel);
};

} // namespace iso::vt_app
