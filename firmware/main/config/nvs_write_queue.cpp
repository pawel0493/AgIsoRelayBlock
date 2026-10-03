#include "config/nvs_write_queue.hpp"

#include <cstdio>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"

namespace config::nvs_write_queue {

namespace {
constexpr const char* kTag = "nvs_write_queue";
constexpr std::size_t kQueueDepth = 8;
constexpr std::size_t kTaskStackSizeBytes = 6 * 1024;
constexpr std::size_t kNvsNameMaxChars = 15;
constexpr std::size_t kStringMaxChars = 63;

struct WriteRequest {
    char nvs_namespace[kNvsNameMaxChars + 1];
    char key[kNvsNameMaxChars + 1];
    char value[kStringMaxChars + 1];
};

QueueHandle_t g_queue = nullptr;
StaticTask_t* g_task_control_block = nullptr;
StackType_t* g_task_stack = nullptr;

void write_request(const WriteRequest& request) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(request.nvs_namespace, NVS_READWRITE, &handle);
    if (ESP_OK != err) {
        ESP_LOGE(kTag, "nvs_open failed for namespace %s (%s)",
                 request.nvs_namespace, esp_err_to_name(err));
        return;
    }

    err = nvs_set_str(handle, request.key, request.value);
    if (ESP_OK == err) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (ESP_OK != err) {
        ESP_LOGE(kTag, "Failed to persist %s/%s (%s)", request.nvs_namespace,
                 request.key, esp_err_to_name(err));
    }
}

void worker_task(void*) {
    WriteRequest request{};
    while (xQueueReceive(g_queue, &request, portMAX_DELAY) == pdTRUE) {
        write_request(request);
    }
}

}  // namespace

void init() {
    if (g_queue != nullptr) {
        return;
    }

    constexpr std::size_t stack_depth_words =
        kTaskStackSizeBytes / sizeof(StackType_t);
    g_task_stack = static_cast<StackType_t*>(
        heap_caps_malloc(stack_depth_words * sizeof(StackType_t),
                         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    g_task_control_block = static_cast<StaticTask_t*>(
        heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    g_queue = xQueueCreate(kQueueDepth, sizeof(WriteRequest));
    if (g_task_stack == nullptr || g_task_control_block == nullptr ||
        g_queue == nullptr) {
        ESP_LOGE(kTag, "Failed to allocate internal-RAM NVS worker resources");
        if (g_queue != nullptr) {
            vQueueDelete(g_queue);
            g_queue = nullptr;
        }
        heap_caps_free(g_task_control_block);
        g_task_control_block = nullptr;
        heap_caps_free(g_task_stack);
        g_task_stack = nullptr;
        return;
    }

    TaskHandle_t task = xTaskCreateStaticPinnedToCore(
        worker_task, "nvs_write", stack_depth_words, nullptr,
        tskIDLE_PRIORITY + 1, g_task_stack, g_task_control_block,
        tskNO_AFFINITY);
    if (task == nullptr) {
        ESP_LOGE(kTag, "Failed to create internal-RAM NVS worker task");
        vQueueDelete(g_queue);
        g_queue = nullptr;
        heap_caps_free(g_task_control_block);
        g_task_control_block = nullptr;
        heap_caps_free(g_task_stack);
        g_task_stack = nullptr;
    }
}

bool enqueue_string(const std::string& nvs_namespace, const std::string& key,
                    const std::string& value) {
    if (g_queue == nullptr || nvs_namespace.empty() || key.empty() ||
        nvs_namespace.size() > kNvsNameMaxChars ||
        key.size() > kNvsNameMaxChars || value.size() > kStringMaxChars) {
        return false;
    }

    WriteRequest request{};
    std::snprintf(request.nvs_namespace, sizeof(request.nvs_namespace), "%s",
                  nvs_namespace.c_str());
    std::snprintf(request.key, sizeof(request.key), "%s", key.c_str());
    std::snprintf(request.value, sizeof(request.value), "%s", value.c_str());
    if (xQueueSend(g_queue, &request, 0) != pdTRUE) {
        ESP_LOGE(kTag, "NVS write queue is full");
        return false;
    }
    return true;
}

}  // namespace config::nvs_write_queue
