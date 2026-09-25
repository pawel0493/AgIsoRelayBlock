#pragma once

#include "isobus/isobus/isobus_virtual_terminal_client.hpp"

namespace iso::vt_app {

class AuxiliaryPreferredAssignmentNVSRepository final : public isobus::VirtualTerminalClient::AuxiliaryPreferredAssignmentRepository
{
public:
	AuxiliaryPreferredAssignmentNVSRepository() = default;

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

	static bool load_all_records(std::vector<PersistedAssignmentRecord> &records);
	static bool save_all_records(const std::vector<PersistedAssignmentRecord> &records);
	static std::vector<std::uint8_t> serialize(const std::vector<PersistedAssignmentRecord> &records);
	static bool deserialize(const std::vector<std::uint8_t> &blob, std::vector<PersistedAssignmentRecord> &records);
};

} // namespace iso::vt_app
