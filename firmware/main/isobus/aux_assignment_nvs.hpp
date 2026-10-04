#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>
#include <mutex>
#include "isobus/isobus/isobus_virtual_terminal_client.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace iso::vt_app {

class AuxiliaryPreferredAssignmentNVSRepository final : public isobus::VirtualTerminalClient::AuxiliaryPreferredAssignmentRepository
{
public:
	AuxiliaryPreferredAssignmentNVSRepository();
	~AuxiliaryPreferredAssignmentNVSRepository() override;

	struct PersistedAssignmentRecord
	{
		std::uint64_t virtualTerminalName = 0;
		std::uint64_t auxiliaryInputDeviceName = 0;
		std::uint16_t modelIdentificationCode = 0;
		std::uint16_t functionObjectID = 0;
		std::uint16_t inputObjectID = 0;
		std::uint8_t functionType = 0;
	};

	// Reduces a VT NAME to its fields that stay the same between power cycles
	// (identity number, manufacturer, function, device class, industry group);
	// instance fields and the arbitrary-address-capable bit may differ per
	// start-up or per VT unit configuration and are ignored when matching.
	static std::uint64_t stable_vt_key(std::uint64_t virtualTerminalName);

	std::vector<isobus::VirtualTerminalClient::PreferredAuxiliaryAssignment> load(std::uint64_t virtualTerminalName) override;
	bool store(std::uint64_t virtualTerminalName, const isobus::VirtualTerminalClient::PreferredAuxiliaryAssignment &assignment) override;
	bool remove(std::uint64_t virtualTerminalName, std::uint16_t functionObjectID) override;
	bool clear(std::uint64_t virtualTerminalName) override;

	// Diagnostics (GET /api/aux): the stored records matching a VT NAME.
	std::vector<PersistedAssignmentRecord> list(std::uint64_t virtualTerminalName);

private:
	enum class WorkerCommand : std::uint8_t
	{
		LoadRecords,
		SaveSnapshot,
		StopWorker
	};

	struct WorkerRequest
	{
		WorkerCommand command;
		std::vector<PersistedAssignmentRecord> *records = nullptr;
		SemaphoreHandle_t completionSignal = nullptr;
		bool success = false;
	};

	static void worker_task_entry(void *context);
	void worker_task();
	bool dispatch_load(std::vector<PersistedAssignmentRecord> &records);
	bool ensure_loaded();
	void schedule_save_locked();
	void save_snapshot();
	static bool load_all_records_from_nvs(std::vector<PersistedAssignmentRecord> &records);
	static bool save_all_records_to_nvs(const std::vector<PersistedAssignmentRecord> &records);
	static std::vector<std::uint8_t> serialize(const std::vector<PersistedAssignmentRecord> &records);
	static bool deserialize(const std::vector<std::uint8_t> &blob, std::vector<PersistedAssignmentRecord> &records);

	void *workerQueue = nullptr;
	void *workerTask = nullptr;
	void *workerTaskControlBlock = nullptr;
	void *workerTaskStack = nullptr;
	SemaphoreHandle_t workerExitSignal = nullptr;
	std::size_t workerStackDepthWords = 0;
	// Guards cache, cacheLoaded and savePending. Never held across NVS access
	// or while waiting for the worker, so the VT/CAN thread only does RAM
	// work; the flash write happens asynchronously in the worker task.
	std::mutex operationMutex;
	std::vector<PersistedAssignmentRecord> cache;
	bool cacheLoaded = false;
	bool savePending = false;
	WorkerRequest saveRequest{ WorkerCommand::SaveSnapshot, nullptr, nullptr, false };
};

} // namespace iso::vt_app
