/**
 * @brief FTP plugin
 * @file ftp.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <chrono>
#include <cerrno>
#include <condition_variable>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <std_srvs/Empty.h>
#include <mavros/FileEntry.h>
#include <mavros/FileList.h>
#include <mavros/FileOpen.h>
#include <mavros/FileClose.h>
#include <mavros/FileRead.h>
#include <mavros/FileWrite.h>
#include <mavros/FileRemove.h>
#include <mavros/FileMakeDir.h>
#include <mavros/FileRemoveDir.h>
#include <mavros/FileTruncate.h>
#include <mavros/FileRename.h>
#include <mavros/FileChecksum.h>

// enable debugging messages
//#define FTP_LL_DEBUG

namespace mavplugin {

/**
 * @brief FTP Request message abstraction class
 *
 * @note This class not portable, and works on little-endian machines only.
 */
class FTPRequest {
public:
	/// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload.
	/// We pad the structure ourselves to 32 bit alignment to avoid usage of any pack pragmas.
	struct PayloadHeader {
		uint16_t	seqNumber;	///< sequence number for message
		uint8_t		session;	///< Session id for read and write commands
		uint8_t		opcode;		///< Command opcode
		uint8_t		size;		///< Size of data
		uint8_t		req_opcode;	///< Request opcode returned in kRspAck, kRspNak message
		uint8_t		padding[2];	///< 32 bit aligment padding
		uint32_t	offset;		///< Offsets for List and Read commands
		uint8_t		data[];		///< command data, varies by Opcode
	};

	/// @brief Command opcodes
	enum Opcode : uint8_t {
		kCmdNone,		///< ignored, always acked
		kCmdTerminateSession,	///< Terminates open Read session
		kCmdResetSessions,	///< Terminates all open Read sessions
		kCmdListDirectory,	///< List files in <path> from <offset>
		kCmdOpenFileRO,		///< Opens file at <path> for reading, returns <session>
		kCmdReadFile,		///< Reads <size> bytes from <offset> in <session>
		kCmdCreateFile,		///< Creates file at <path> for writing, returns <session>
		kCmdWriteFile,		///< Writes <size> bytes to <offset> in <session>
		kCmdRemoveFile,		///< Remove file at <path>
		kCmdCreateDirectory,	///< Creates directory at <path>
		kCmdRemoveDirectory,	///< Removes Directory at <path>, must be empty
		kCmdOpenFileWO,		///< Opens file at <path> for writing, returns <session>
		kCmdTruncateFile,	///< Truncate file at <path> to <offset> length
		kCmdRename,		///< Rename <path1> to <path2>
		kCmdCalcFileCRC32,	///< Calculate CRC32 for file at <path>

		kRspAck = 128,		///< Ack response
		kRspNak			///< Nak response
	};

	/// @brief Error codes returned in Nak response.
	enum ErrorCode : uint8_t {
		kErrNone,
		kErrFail,			///< Unknown failure
		kErrFailErrno,			///< Command failed, errno sent back in PayloadHeader.data[1]
		kErrInvalidDataSize,		///< PayloadHeader.size is invalid
		kErrInvalidSession,		///< Session is not currently open
		kErrNoSessionsAvailable,	///< All available Sessions in use
		kErrEOF,			///< Offset past end of file for List and Read commands
		kErrUnknownCommand		///< Unknown command opcode
	};

	static const char	DIRENT_FILE = 'F';
	static const char	DIRENT_DIR = 'D';
	static const char	DIRENT_SKIP = 'S';
	static const uint8_t	DATA_MAXSZ = MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(PayloadHeader);

	uint8_t *raw_payload();

	inline PayloadHeader *header();

	uint8_t *data();

	char *data_c();

	uint32_t *data_u32();

	/**
	 * @brief Copy string to payload
	 *
	 * @param[in] s  payload string
	 * @note this function allow null termination inside string
	 *       it used to send multiple strings in one message
	 */
	void set_data_string(std::string &s);

	uint8_t get_target_system_id();

	/**
	 * @brief Decode and check target system
	 */
	bool decode(UAS *uas, const mavlink_message_t *msg);

	/**
	 * @brief Encode and send message
	 */
	void send(UAS *uas, uint16_t seqNumber);

	FTPRequest();

	explicit FTPRequest(Opcode op, uint8_t session = 0);

private:
	mavlink_file_transfer_protocol_t message;
};


/**
 * @brief FTP plugin.
 */
class FTPPlugin : public MavRosPlugin {
public:
	FTPPlugin();

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater);

	std::string const get_name() const;

	const message_map get_rx_handlers();

private:
	UAS *uas;
	ros::NodeHandle ftp_nh;
	ros::ServiceServer list_srv;
	ros::ServiceServer open_srv;
	ros::ServiceServer close_srv;
	ros::ServiceServer read_srv;
	ros::ServiceServer write_srv;
	ros::ServiceServer mkdir_srv;
	ros::ServiceServer rmdir_srv;
	ros::ServiceServer remove_srv;
	ros::ServiceServer rename_srv;
	ros::ServiceServer truncate_srv;
	ros::ServiceServer reset_srv;
	ros::ServiceServer checksum_srv;

	//! This type used in servicies to store 'data' fileds.
	typedef std::vector<uint8_t> V_FileData;

	enum OpState {
		OP_IDLE,
		OP_ACK,
		OP_LIST,
		OP_OPEN,
		OP_READ,
		OP_WRITE,
		OP_CHECKSUM
	};

	OpState op_state;
	uint16_t last_send_seqnr;	//!< seqNumber for send.
	uint32_t active_session;	//!< session id of current operation

	std::mutex cond_mutex;
	std::condition_variable cond;	//!< wait condvar
	bool is_error;			//!< error signaling flag (timeout/proto error)
	int r_errno;			//!< store errno from server

	// FTP:List
	uint32_t list_offset;
	std::string list_path;
	std::vector<mavros::FileEntry> list_entries;

	// FTP:Open / FTP:Close
	std::string open_path;
	size_t open_size;
	std::map<std::string, uint32_t> session_file_map;

	// FTP:Read
	size_t read_size;
	uint32_t read_offset;
	V_FileData read_buffer;

	// FTP:Write
	uint32_t write_offset;
	V_FileData write_buffer;
	V_FileData::iterator write_it;

	// FTP:CalcCRC32
	uint32_t checksum_crc32;

	// Timeouts,
	// computed as x4 time that needed for transmission of
	// one message at 57600 baud rate
	static constexpr int LIST_TIMEOUT_MS = 5000;
	static constexpr int OPEN_TIMEOUT_MS = 200;
	static constexpr int CHUNK_TIMEOUT_MS = 200;

	//! Maximum difference between allocated space and used
	static constexpr size_t MAX_RESERVE_DIFF = 0x10000;

	//! @todo exchange speed calculation
	//! @todo diagnostics

	/* -*- message handler -*- */

	void handle_file_transfer_protocol(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid);

	void handle_req_ack(FTPRequest &req);

	void handle_req_nack(FTPRequest &req);

	void handle_ack_list(FTPRequest &req);

	void handle_ack_open(FTPRequest &req);

	void handle_ack_read(FTPRequest &req);

	void handle_ack_write(FTPRequest &req);

	void handle_ack_checksum(FTPRequest &req);

	/* -*- send helpers -*- */

	/**
	 * @brief Go to IDLE mode
	 *
	 * @param is_error_ mark that caused in error case
	 * @param r_errno_ set r_errno in error case
	 */
	void go_idle(bool is_error_, int r_errno_ = 0);

	void send_reset();

	/// Send any command with string payload (usually file/dir path)
	inline void send_any_path_command(FTPRequest::Opcode op, const std::string debug_msg, std::string &path, uint32_t offset);

	void send_list_command();

	void send_open_ro_command();

	void send_open_wo_command();

	void send_create_command();

	void send_terminate_command(uint32_t session);

	void send_read_command();

	void send_write_command(const size_t bytes_to_copy);

	void send_remove_command(std::string &path);

	bool send_rename_command(std::string &old_path, std::string &new_path);

	void send_truncate_command(std::string &path, size_t length);

	void send_create_dir_command(std::string &path);

	void send_remove_dir_command(std::string &path);

	void send_calc_file_crc32_command(std::string &path);

	/* -*- helpers -*- */

	void add_dirent(const char *ptr, size_t slen);

	void list_directory_end();

	void list_directory(std::string &path);

	bool open_file(std::string &path, int mode);

	bool close_file(std::string &path);

	void read_file_end();

	bool read_file(std::string &path, size_t off, size_t len);

	void write_file_end();

	bool write_file(std::string &path, size_t off, V_FileData &data);

	void remove_file(std::string &path);

	bool rename_(std::string &old_path, std::string &new_path);

	void truncate_file(std::string &path, size_t length);

	void create_directory(std::string &path);

	void remove_directory(std::string &path);

	void checksum_crc32_file(std::string &path);

	static constexpr int compute_rw_timeout(size_t len);

	size_t write_bytes_to_copy();

	bool wait_completion(const int msecs);

	/* -*- service callbacks -*- */

	/**
	 * Service handler common header code.
	 */
#define SERVICE_IDLE_CHECK()				\
	if (op_state != OP_IDLE) {			\
		ROS_ERROR_NAMED("ftp", "FTP: Busy");	\
		return false;				\
	}

	bool list_cb(mavros::FileList::Request &req,
			mavros::FileList::Response &res);

	bool open_cb(mavros::FileOpen::Request &req,
			mavros::FileOpen::Response &res);

	bool close_cb(mavros::FileClose::Request &req,
			mavros::FileClose::Response &res);

	bool read_cb(mavros::FileRead::Request &req,
			mavros::FileRead::Response &res);

	bool write_cb(mavros::FileWrite::Request &req,
			mavros::FileWrite::Response &res);

	bool remove_cb(mavros::FileRemove::Request &req,
			mavros::FileRemove::Response &res);

	bool rename_cb(mavros::FileRename::Request &req,
			mavros::FileRename::Response &res);

	bool truncate_cb(mavros::FileTruncate::Request &req,
			mavros::FileTruncate::Response &res);

	bool mkdir_cb(mavros::FileMakeDir::Request &req,
			mavros::FileMakeDir::Response &res);

	bool rmdir_cb(mavros::FileRemoveDir::Request &req,
			mavros::FileRemoveDir::Response &res);

	bool checksum_cb(mavros::FileChecksum::Request &req,
			mavros::FileChecksum::Response &res);

#undef SERVICE_IDLE_CHECK

	/**
	 * @brief Reset communication on both sides.
	 * @note This call break other calls, so use carefully.
	 */
	bool reset_cb(std_srvs::Empty::Request &req,
			std_srvs::Empty::Response &res);
};

}; // namespace mavplugin

