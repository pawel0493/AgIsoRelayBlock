#pragma once

#include <cstddef>
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

	std::vector<isobus::VirtualTerminalClient::PreferredAuxiliaryAssignment> load(std::uint64_t virtualTerminalName) override;
	bool store(std::uint64_t virtualTerminalName, const isobus::VirtualTerminalClient::PreferredAuxiliaryAssignment &assignment) override;
	bool remove(std::uint64_t virtualTerminalName, std::uint16_t functionObjectID) override;
	bool clear(std::uint64_t virtualTerminalName) override;

private:
	struct PersistedAssignmentRecord
	{
		std::uint64_t virtualTerminalName = 0;
		std::uint64_t auxiliaryInputDeviceName = 0;
		std::uint16_t modelIdentificationCode = 0;
		std::uint16_t functionObjectID = 0;
		std::uint16_t inputObjectID = 0;
		std::uint8_t functionType = 0;
	};

	enum class WorkerCommand : std::uint8_t
	{
		LoadRecords,
		SaveRecords
	};

	struct WorkerRequest
	{
		WorkerCommand command;
		std::vector<PersistedAssignmentRecord> *records = nullptr;
		const std::vector<PersistedAssignmentRecord> *recordsToSave = nullptr;
		SemaphoreHandle_t completionSignal = nullptr;
		bool success = false;
	};

	static void worker_task_entry(void *context);
	void worker_task();
	bool dispatch_request(WorkerRequest &request);
	void cleanup_worker_resources();
	bool load_all_records(std::vector<PersistedAssignmentRecord> &records);
	bool save_all_records(const std::vector<PersistedAssignmentRecord> &records);
	static bool load_all_records_from_nvs(std::vector<PersistedAssignmentRecord> &records);
	static bool save_all_records_to_nvs(const std::vector<PersistedAssignmentRecord> &records);
	static std::vector<std::uint8_t> serialize(const std::vector<PersistedAssignmentRecord> &records);
	static bool deserialize(const std::vector<std::uint8_t> &blob, std::vector<PersistedAssignmentRecord> &records);

	void *workerQueue = nullptr;
	void *workerTask = nullptr;
	void *workerTaskControlBlock = nullptr;
	void *workerTaskStack = nullptr;
	std::size_t workerStackDepthWords = 0;
	std::mutex operationMutex;
};

} // namespace iso::vt_app
