#include "isobus/aux_assignment_nvs.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"

namespace iso::vt_app {
namespace {

constexpr const char *kTag = "aux_pref_nvs";
constexpr const char *kNamespace = "aux_pref";
constexpr const char *kBlobKey = "records";
constexpr std::uint32_t kBlobMagic = 0x50585541; // "AUXP" LE
constexpr std::uint16_t kBlobVersion = 1;
constexpr std::size_t kRecordSize = 8 + 8 + 2 + 2 + 2 + 1;
constexpr std::size_t kWorkerQueueDepth = 4;
constexpr std::size_t kWorkerStackSizeBytes = 6 * 1024;
constexpr TickType_t kWorkerStopTimeout = pdMS_TO_TICKS(1000);

void append_u16(std::vector<std::uint8_t> &buffer, std::uint16_t value)
{
	buffer.push_back(static_cast<std::uint8_t>(value));
	buffer.push_back(static_cast<std::uint8_t>(value >> 8));
}

void append_u32(std::vector<std::uint8_t> &buffer, std::uint32_t value)
{
	buffer.push_back(static_cast<std::uint8_t>(value));
	buffer.push_back(static_cast<std::uint8_t>(value >> 8));
	buffer.push_back(static_cast<std::uint8_t>(value >> 16));
	buffer.push_back(static_cast<std::uint8_t>(value >> 24));
}

void append_u64(std::vector<std::uint8_t> &buffer, std::uint64_t value)
{
	buffer.push_back(static_cast<std::uint8_t>(value));
	buffer.push_back(static_cast<std::uint8_t>(value >> 8));
	buffer.push_back(static_cast<std::uint8_t>(value >> 16));
	buffer.push_back(static_cast<std::uint8_t>(value >> 24));
	buffer.push_back(static_cast<std::uint8_t>(value >> 32));
	buffer.push_back(static_cast<std::uint8_t>(value >> 40));
	buffer.push_back(static_cast<std::uint8_t>(value >> 48));
	buffer.push_back(static_cast<std::uint8_t>(value >> 56));
}

bool read_u16(const std::vector<std::uint8_t> &buffer, std::size_t &offset, std::uint16_t &value)
{
	if ((offset + 2) > buffer.size())
	{
		return false;
	}
	value = static_cast<std::uint16_t>(buffer[offset]) |
	        (static_cast<std::uint16_t>(buffer[offset + 1]) << 8);
	offset += 2;
	return true;
}

bool read_u32(const std::vector<std::uint8_t> &buffer, std::size_t &offset, std::uint32_t &value)
{
	if ((offset + 4) > buffer.size())
	{
		return false;
	}
	value = static_cast<std::uint32_t>(buffer[offset]) |
	        (static_cast<std::uint32_t>(buffer[offset + 1]) << 8) |
	        (static_cast<std::uint32_t>(buffer[offset + 2]) << 16) |
	        (static_cast<std::uint32_t>(buffer[offset + 3]) << 24);
	offset += 4;
	return true;
}

bool read_u64(const std::vector<std::uint8_t> &buffer, std::size_t &offset, std::uint64_t &value)
{
	if ((offset + 8) > buffer.size())
	{
		return false;
	}
	value = static_cast<std::uint64_t>(buffer[offset]) |
	        (static_cast<std::uint64_t>(buffer[offset + 1]) << 8) |
	        (static_cast<std::uint64_t>(buffer[offset + 2]) << 16) |
	        (static_cast<std::uint64_t>(buffer[offset + 3]) << 24) |
	        (static_cast<std::uint64_t>(buffer[offset + 4]) << 32) |
	        (static_cast<std::uint64_t>(buffer[offset + 5]) << 40) |
	        (static_cast<std::uint64_t>(buffer[offset + 6]) << 48) |
	        (static_cast<std::uint64_t>(buffer[offset + 7]) << 56);
	offset += 8;
	return true;
}

} // namespace

AuxiliaryPreferredAssignmentNVSRepository::AuxiliaryPreferredAssignmentNVSRepository()
{
	workerStackDepthWords = kWorkerStackSizeBytes / sizeof(StackType_t);
	workerTaskStack = heap_caps_malloc(workerStackDepthWords * sizeof(StackType_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	workerTaskControlBlock = heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	workerQueue = xQueueCreate(kWorkerQueueDepth, sizeof(WorkerRequest *));
	workerExitSignal = xSemaphoreCreateBinary();
	if ((nullptr == workerTaskStack) || (nullptr == workerTaskControlBlock) || (nullptr == workerQueue) || (nullptr == workerExitSignal))
	{
		ESP_LOGE(kTag, "failed to create NVS worker resources");
		if (nullptr != workerQueue)
		{
			vQueueDelete(static_cast<QueueHandle_t>(workerQueue));
			workerQueue = nullptr;
		}
		if (nullptr != workerTaskControlBlock)
		{
			heap_caps_free(workerTaskControlBlock);
			workerTaskControlBlock = nullptr;
		}
		if (nullptr != workerTaskStack)
		{
			heap_caps_free(workerTaskStack);
			workerTaskStack = nullptr;
		}
		if (nullptr != workerExitSignal)
		{
			vSemaphoreDelete(workerExitSignal);
			workerExitSignal = nullptr;
		}
		return;
	}

	workerTask = xTaskCreateStaticPinnedToCore(worker_task_entry,
	                                           "aux_pref_nvs",
	                                           static_cast<uint32_t>(workerStackDepthWords),
	                                           this,
	                                           tskIDLE_PRIORITY + 1,
	                                           static_cast<StackType_t *>(workerTaskStack),
	                                           static_cast<StaticTask_t *>(workerTaskControlBlock),
	                                           tskNO_AFFINITY);
	if (nullptr == workerTask)
	{
		ESP_LOGE(kTag, "failed to create NVS worker task");
		if (nullptr != workerQueue)
		{
			vQueueDelete(static_cast<QueueHandle_t>(workerQueue));
			workerQueue = nullptr;
		}
		if (nullptr != workerTaskControlBlock)
		{
			heap_caps_free(workerTaskControlBlock);
			workerTaskControlBlock = nullptr;
		}
		if (nullptr != workerTaskStack)
		{
			heap_caps_free(workerTaskStack);
			workerTaskStack = nullptr;
		}
		if (nullptr != workerExitSignal)
		{
			vSemaphoreDelete(workerExitSignal);
			workerExitSignal = nullptr;
		}
	}
}

AuxiliaryPreferredAssignmentNVSRepository::~AuxiliaryPreferredAssignmentNVSRepository()
{

	const auto queueHandle = static_cast<QueueHandle_t>(workerQueue);
	const auto taskHandle = static_cast<TaskHandle_t>(workerTask);
	const auto currentTask = xTaskGetCurrentTaskHandle();
	bool workerStopAcknowledged = false;
	if ((nullptr != queueHandle) && (nullptr != taskHandle) && (currentTask != taskHandle))
	{
		if (nullptr != workerExitSignal)
		{
			(void)xSemaphoreTake(workerExitSignal, 0);
		}

		WorkerRequest stopRequest{
			WorkerCommand::StopWorker,
			nullptr,
			xSemaphoreCreateBinary(),
			false
		};
		if (nullptr != stopRequest.completionSignal)
		{
			auto *requestPointer = &stopRequest;
			if (xQueueSend(queueHandle, &requestPointer, kWorkerStopTimeout) == pdTRUE)
			{
				workerStopAcknowledged = (xSemaphoreTake(stopRequest.completionSignal, kWorkerStopTimeout) == pdTRUE);
			}
			vSemaphoreDelete(stopRequest.completionSignal);
		}
	}
	if ((nullptr != workerExitSignal) && (nullptr != taskHandle) && (currentTask != taskHandle))
	{
		(void)xSemaphoreTake(workerExitSignal, kWorkerStopTimeout);
	}

	if ((nullptr != taskHandle) && (currentTask != taskHandle) && (!workerStopAcknowledged))
	{
		vTaskDelete(taskHandle);
	}
	if (nullptr != queueHandle)
	{
		vQueueDelete(queueHandle);
	}
	if (nullptr != workerTaskControlBlock)
	{
		heap_caps_free(workerTaskControlBlock);
	}
	if (nullptr != workerTaskStack)
	{
		heap_caps_free(workerTaskStack);
	}
	if (nullptr != workerExitSignal)
	{
		vSemaphoreDelete(workerExitSignal);
	}

	workerTask = nullptr;
	workerQueue = nullptr;
	workerTaskControlBlock = nullptr;
	workerTaskStack = nullptr;
	workerExitSignal = nullptr;
}

void AuxiliaryPreferredAssignmentNVSRepository::worker_task_entry(void *context)
{
	auto *self = static_cast<AuxiliaryPreferredAssignmentNVSRepository *>(context);
	if (nullptr != self)
	{
		self->worker_task();
		if (nullptr != self->workerExitSignal)
		{
			xSemaphoreGive(self->workerExitSignal);
		}
	}
	vTaskDelete(nullptr);
}

void AuxiliaryPreferredAssignmentNVSRepository::worker_task()
{
	WorkerRequest *request = nullptr;
	while (xQueueReceive(static_cast<QueueHandle_t>(workerQueue), &request, portMAX_DELAY) == pdTRUE)
	{
		if (nullptr == request)
		{
			continue;
		}

		switch (request->command)
		{
			case WorkerCommand::LoadRecords:
				request->success = (nullptr != request->records) && load_all_records_from_nvs(*request->records);
				break;

			case WorkerCommand::SaveSnapshot:
				save_snapshot();
				request->success = true;
				break;

			case WorkerCommand::StopWorker:
				request->success = true;
				break;
		}

		if (nullptr != request->completionSignal)
		{
			xSemaphoreGive(request->completionSignal);
		}

		if (request->command == WorkerCommand::StopWorker)
		{
			break;
		}
	}
}

// Runs in the worker task: take a copy of the latest cache (so several quick
// changes collapse into one flash write) and write it without holding the lock.
void AuxiliaryPreferredAssignmentNVSRepository::save_snapshot()
{
	std::vector<PersistedAssignmentRecord> snapshot;
	{
		std::lock_guard<std::mutex> lock(operationMutex);
		snapshot = cache;
		savePending = false;
	}
	const bool ok = save_all_records_to_nvs(snapshot);
	ESP_LOGI(kTag, "saved %u preferred assignment record(s) to NVS -> %s", static_cast<unsigned>(snapshot.size()), ok ? "OK" : "FAILED");
}

bool AuxiliaryPreferredAssignmentNVSRepository::dispatch_load(std::vector<PersistedAssignmentRecord> &records)
{
	if ((nullptr == workerQueue) || (nullptr == workerTask))
	{
		ESP_LOGE(kTag, "NVS worker is not available");
		return false;
	}

	if (xTaskGetCurrentTaskHandle() == static_cast<TaskHandle_t>(workerTask))
	{
		return load_all_records_from_nvs(records);
	}

	WorkerRequest request{ WorkerCommand::LoadRecords, &records, xSemaphoreCreateBinary(), false };
	if (nullptr == request.completionSignal)
	{
		ESP_LOGE(kTag, "failed to create request synchronization primitive");
		return false;
	}

	auto *requestPointer = &request;
	if (xQueueSend(static_cast<QueueHandle_t>(workerQueue), &requestPointer, portMAX_DELAY) != pdTRUE)
	{
		ESP_LOGE(kTag, "failed to enqueue NVS worker request");
		vSemaphoreDelete(request.completionSignal);
		return false;
	}

	(void)xSemaphoreTake(request.completionSignal, portMAX_DELAY);
	vSemaphoreDelete(request.completionSignal);
	return request.success;
}

// Loads the NVS blob into the RAM cache once (through the worker task, which
// keeps flash access off the VT thread's PSRAM stack). The lock is not held
// while waiting for the worker.
bool AuxiliaryPreferredAssignmentNVSRepository::ensure_loaded()
{
	{
		std::lock_guard<std::mutex> lock(operationMutex);
		if (cacheLoaded)
		{
			return true;
		}
	}
	std::vector<PersistedAssignmentRecord> records;
	if (!dispatch_load(records))
	{
		return false;
	}
	std::lock_guard<std::mutex> lock(operationMutex);
	if (!cacheLoaded)
	{
		cache = std::move(records);
		cacheLoaded = true;
		ESP_LOGI(kTag, "loaded %u preferred assignment record(s) from NVS", static_cast<unsigned>(cache.size()));
	}
	return true;
}

// Caller holds operationMutex. At most one save request is ever queued.
void AuxiliaryPreferredAssignmentNVSRepository::schedule_save_locked()
{
	if (savePending)
	{
		return;
	}
	if ((nullptr == workerQueue) || (nullptr == workerTask))
	{
		ESP_LOGE(kTag, "NVS worker is not available, assignment change is kept in RAM only");
		return;
	}
	auto *requestPointer = &saveRequest;
	if (xQueueSend(static_cast<QueueHandle_t>(workerQueue), &requestPointer, 0) == pdTRUE)
	{
		savePending = true;
	}
	else
	{
		ESP_LOGE(kTag, "NVS worker queue is full, assignment change is kept in RAM only");
	}
}

std::vector<std::uint8_t> AuxiliaryPreferredAssignmentNVSRepository::serialize(const std::vector<PersistedAssignmentRecord> &records)
{
	std::vector<std::uint8_t> blob;
	blob.reserve(8 + (records.size() * kRecordSize));
	append_u32(blob, kBlobMagic);
	append_u16(blob, kBlobVersion);
	append_u16(blob, static_cast<std::uint16_t>(records.size()));
	for (const auto &record : records)
	{
		append_u64(blob, record.virtualTerminalName);
		append_u64(blob, record.auxiliaryInputDeviceName);
		append_u16(blob, record.modelIdentificationCode);
		append_u16(blob, record.functionObjectID);
		append_u16(blob, record.inputObjectID);
		blob.push_back(record.functionType);
	}
	return blob;
}

bool AuxiliaryPreferredAssignmentNVSRepository::deserialize(const std::vector<std::uint8_t> &blob, std::vector<PersistedAssignmentRecord> &records)
{
	records.clear();
	if (blob.empty())
	{
		return true;
	}
	std::size_t offset = 0;
	std::uint32_t magic = 0;
	std::uint16_t version = 0;
	std::uint16_t count = 0;
	if ((!read_u32(blob, offset, magic)) ||
	    (!read_u16(blob, offset, version)) ||
	    (!read_u16(blob, offset, count)))
	{
		return false;
	}
	if ((magic != kBlobMagic) || (version != kBlobVersion))
	{
		return false;
	}

	const std::size_t expectedBytes = offset + (static_cast<std::size_t>(count) * kRecordSize);
	if (expectedBytes != blob.size())
	{
		return false;
	}

	records.reserve(count);
	for (std::uint16_t i = 0; i < count; ++i)
	{
		PersistedAssignmentRecord record;
		if ((!read_u64(blob, offset, record.virtualTerminalName)) ||
		    (!read_u64(blob, offset, record.auxiliaryInputDeviceName)) ||
		    (!read_u16(blob, offset, record.modelIdentificationCode)) ||
		    (!read_u16(blob, offset, record.functionObjectID)) ||
		    (!read_u16(blob, offset, record.inputObjectID)))
		{
			return false;
		}
		if (offset >= blob.size())
		{
			return false;
		}
		record.functionType = blob[offset++];
		records.push_back(record);
	}
	return (offset == blob.size());
}

bool AuxiliaryPreferredAssignmentNVSRepository::load_all_records_from_nvs(std::vector<PersistedAssignmentRecord> &records)
{
	records.clear();

	nvs_handle_t handle;
	esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(kTag, "nvs_open failed: %s", esp_err_to_name(err));
		return false;
	}

	size_t requiredSize = 0;
	err = nvs_get_blob(handle, kBlobKey, nullptr, &requiredSize);
	if (err == ESP_ERR_NVS_NOT_FOUND)
	{
		nvs_close(handle);
		return true;
	}
	if (err != ESP_OK)
	{
		ESP_LOGE(kTag, "nvs_get_blob size failed: %s", esp_err_to_name(err));
		nvs_close(handle);
		return false;
	}

	std::vector<std::uint8_t> blob(requiredSize);
	err = nvs_get_blob(handle, kBlobKey, blob.data(), &requiredSize);
	nvs_close(handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(kTag, "nvs_get_blob data failed: %s", esp_err_to_name(err));
		return false;
	}

	if (!deserialize(blob, records))
	{
		ESP_LOGW(kTag, "preferred assignment blob is invalid, ignoring stored data");
		records.clear();
	}
	return true;
}

bool AuxiliaryPreferredAssignmentNVSRepository::save_all_records_to_nvs(const std::vector<PersistedAssignmentRecord> &records)
{
	nvs_handle_t handle;
	esp_err_t err = nvs_open(kNamespace, NVS_READWRITE, &handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(kTag, "nvs_open failed: %s", esp_err_to_name(err));
		return false;
	}

	if (records.empty())
	{
		err = nvs_erase_key(handle, kBlobKey);
		if ((err != ESP_OK) && (err != ESP_ERR_NVS_NOT_FOUND))
		{
			ESP_LOGE(kTag, "nvs_erase_key failed: %s", esp_err_to_name(err));
		}
		else
		{
			err = nvs_commit(handle);
			if (err != ESP_OK)
			{
				ESP_LOGE(kTag, "nvs_commit failed: %s", esp_err_to_name(err));
			}
		}
	}
	else
	{
		const auto blob = serialize(records);
		err = nvs_set_blob(handle, kBlobKey, blob.data(), blob.size());
		if (err != ESP_OK)
		{
			ESP_LOGE(kTag, "nvs_set_blob failed: %s (%u bytes)", esp_err_to_name(err), static_cast<unsigned>(blob.size()));
		}
		else
		{
			err = nvs_commit(handle);
			if (err != ESP_OK)
			{
				ESP_LOGE(kTag, "nvs_commit failed: %s", esp_err_to_name(err));
			}
		}
	}
	nvs_close(handle);

	if (err != ESP_OK)
	{
		ESP_LOGE(kTag, "failed to save preferred assignments: %s", esp_err_to_name(err));
		return false;
	}
	return true;
}

std::uint64_t AuxiliaryPreferredAssignmentNVSRepository::stable_vt_key(std::uint64_t virtualTerminalName)
{
	// NAME layout: bits 32-34 ECU instance, 35-39 function instance,
	// 56-59 device class instance, 63 arbitrary address capable.
	constexpr std::uint64_t kVolatileBits = (0xFFULL << 32) | (0x0FULL << 56) | (1ULL << 63);
	return virtualTerminalName & ~kVolatileBits;
}

std::vector<AuxiliaryPreferredAssignmentNVSRepository::PersistedAssignmentRecord> AuxiliaryPreferredAssignmentNVSRepository::list(std::uint64_t virtualTerminalName)
{
	std::vector<PersistedAssignmentRecord> result;
	if (!ensure_loaded())
	{
		return result;
	}
	std::lock_guard<std::mutex> lock(operationMutex);
	for (const auto &record : cache)
	{
		if (stable_vt_key(record.virtualTerminalName) == stable_vt_key(virtualTerminalName))
		{
			result.push_back(record);
		}
	}
	return result;
}

std::vector<isobus::VirtualTerminalClient::PreferredAuxiliaryAssignment> AuxiliaryPreferredAssignmentNVSRepository::load(std::uint64_t virtualTerminalName)
{
	std::vector<isobus::VirtualTerminalClient::PreferredAuxiliaryAssignment> result;
	for (const auto &record : list(virtualTerminalName))
	{
		result.push_back({
		  record.auxiliaryInputDeviceName,
		  record.modelIdentificationCode,
		  {
		    record.functionObjectID,
		    record.inputObjectID,
		    static_cast<isobus::VirtualTerminalClient::AuxiliaryTypeTwoFunctionType>(record.functionType)
		  }
		});
	}
	ESP_LOGI(kTag, "load: %u record(s) for VT NAME 0x%016llx (matched on stable key 0x%016llx)", static_cast<unsigned>(result.size()),
	         static_cast<unsigned long long>(virtualTerminalName), static_cast<unsigned long long>(stable_vt_key(virtualTerminalName)));
	return result;
}

bool AuxiliaryPreferredAssignmentNVSRepository::store(std::uint64_t virtualTerminalName, const isobus::VirtualTerminalClient::PreferredAuxiliaryAssignment &assignment)
{
	if (!ensure_loaded())
	{
		return false;
	}

	PersistedAssignmentRecord updatedRecord{
		virtualTerminalName,
		assignment.auxiliaryInputDeviceName,
		assignment.modelIdentificationCode,
		assignment.function.functionObjectID,
		assignment.function.inputObjectID,
		static_cast<std::uint8_t>(assignment.function.functionType)
	};

	std::lock_guard<std::mutex> lock(operationMutex);
	const auto key = stable_vt_key(virtualTerminalName);
	auto existing = std::find_if(cache.begin(), cache.end(), [key, &assignment](const PersistedAssignmentRecord &record) {
		return (stable_vt_key(record.virtualTerminalName) == key) &&
		       (record.functionObjectID == assignment.function.functionObjectID);
	});
	if (existing != cache.end())
	{
		*existing = updatedRecord;
	}
	else
	{
		cache.push_back(updatedRecord);
	}
	schedule_save_locked();
	return true;
}

bool AuxiliaryPreferredAssignmentNVSRepository::remove(std::uint64_t virtualTerminalName, std::uint16_t functionObjectID)
{
	if (!ensure_loaded())
	{
		return false;
	}

	std::lock_guard<std::mutex> lock(operationMutex);
	const auto key = stable_vt_key(virtualTerminalName);
	const auto before = cache.size();
	cache.erase(std::remove_if(cache.begin(), cache.end(), [key, functionObjectID](const PersistedAssignmentRecord &record) {
		return (stable_vt_key(record.virtualTerminalName) == key) &&
		       (record.functionObjectID == functionObjectID);
	}),
	            cache.end());
	if (before != cache.size())
	{
		schedule_save_locked();
	}
	return true;
}

bool AuxiliaryPreferredAssignmentNVSRepository::clear(std::uint64_t virtualTerminalName)
{
	if (!ensure_loaded())
	{
		return false;
	}

	std::lock_guard<std::mutex> lock(operationMutex);
	const auto key = stable_vt_key(virtualTerminalName);
	const auto before = cache.size();
	cache.erase(std::remove_if(cache.begin(), cache.end(), [key](const PersistedAssignmentRecord &record) {
		return stable_vt_key(record.virtualTerminalName) == key;
	}),
	            cache.end());
	if (before != cache.size())
	{
		schedule_save_locked();
	}
	return true;
}

} // namespace iso::vt_app
