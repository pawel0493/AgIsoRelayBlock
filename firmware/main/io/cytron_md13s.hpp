#pragma once

#include <cstdint>

namespace io::cytron_md13s {

void init();
bool set_speed_percent(uint8_t percent);
uint8_t get_speed_percent();

}  // namespace io::cytron_md13s
