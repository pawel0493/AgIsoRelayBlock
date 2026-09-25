#include "isobus/aux_assignment_nvs.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "esp_log.h"
#include "nvs.h"

namespace iso::vt_app {
namespace {

constexpr const char *kTag = "aux_pref_nvs";
constexpr const char *kNamespace = "aux_pref";
constexpr const char *kBlobKey = "records";
constexpr std::uint32_t kBlobMagic = 0x50585541; // "AUXP" LE
constexpr std::uint16_t kBlobVersion = 1;
constexpr std::size_t kRecordSize = 8 + 8 + 2 + 2 + 2 + 1;

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

bool AuxiliaryPreferredAssignmentNVSRepository::load_all_records(std::vector<PersistedAssignmentRecord> &records)
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

bool AuxiliaryPreferredAssignmentNVSRepository::save_all_records(const std::vector<PersistedAssignmentRecord> &records)
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
		if ((err == ESP_OK) || (err == ESP_ERR_NVS_NOT_FOUND))
		{
			err = nvs_commit(handle);
		}
	}
	else
	{
		const auto blob = serialize(records);
		err = nvs_set_blob(handle, kBlobKey, blob.data(), blob.size());
		if (err == ESP_OK)
		{
			err = nvs_commit(handle);
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

std::vector<isobus::VirtualTerminalClient::PreferredAuxiliaryAssignment> AuxiliaryPreferredAssignmentNVSRepository::load(std::uint64_t virtualTerminalName)
{
	std::vector<PersistedAssignmentRecord> records;
	if (!load_all_records(records))
	{
		return {};
	}

	std::vector<isobus::VirtualTerminalClient::PreferredAuxiliaryAssignment> result;
	for (const auto &record : records)
	{
		if (record.virtualTerminalName != virtualTerminalName)
		{
			continue;
		}
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
	return result;
}

bool AuxiliaryPreferredAssignmentNVSRepository::store(std::uint64_t virtualTerminalName, const isobus::VirtualTerminalClient::PreferredAuxiliaryAssignment &assignment)
{
	std::vector<PersistedAssignmentRecord> records;
	if (!load_all_records(records))
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

	auto existing = std::find_if(records.begin(), records.end(), [virtualTerminalName, &assignment](const PersistedAssignmentRecord &record) {
		return (record.virtualTerminalName == virtualTerminalName) &&
		       (record.functionObjectID == assignment.function.functionObjectID);
	});
	if (existing != records.end())
	{
		*existing = updatedRecord;
	}
	else
	{
		records.push_back(updatedRecord);
	}

	return save_all_records(records);
}

bool AuxiliaryPreferredAssignmentNVSRepository::remove(std::uint64_t virtualTerminalName, std::uint16_t functionObjectID)
{
	std::vector<PersistedAssignmentRecord> records;
	if (!load_all_records(records))
	{
		return false;
	}

	const auto before = records.size();
	records.erase(std::remove_if(records.begin(), records.end(), [virtualTerminalName, functionObjectID](const PersistedAssignmentRecord &record) {
		return (record.virtualTerminalName == virtualTerminalName) &&
		       (record.functionObjectID == functionObjectID);
	}),
	              records.end());

	if (before == records.size())
	{
		return true;
	}

	return save_all_records(records);
}

bool AuxiliaryPreferredAssignmentNVSRepository::clear(std::uint64_t virtualTerminalName)
{
	std::vector<PersistedAssignmentRecord> records;
	if (!load_all_records(records))
	{
		return false;
	}

	const auto before = records.size();
	records.erase(std::remove_if(records.begin(), records.end(), [virtualTerminalName](const PersistedAssignmentRecord &record) {
		return record.virtualTerminalName == virtualTerminalName;
	}),
	              records.end());

	if (before == records.size())
	{
		return true;
	}

	return save_all_records(records);
}

} // namespace iso::vt_app
