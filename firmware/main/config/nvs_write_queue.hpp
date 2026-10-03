#pragma once

#include <string>

namespace config::nvs_write_queue {

void init();
bool enqueue_string(const std::string& nvs_namespace, const std::string& key,
                    const std::string& value);

}  // namespace config::nvs_write_queue
