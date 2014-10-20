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
#include <mavros/ftp.h>
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

	uint8_t *FTPRequest::raw_payload() {
		return message.payload;
	}

	inline FTPRequest::PayloadHeader *FTPRequest::header() {
		return reinterpret_cast<PayloadHeader *>(message.payload);
	}

	uint8_t *FTPRequest::data() {
		return header()->data;
	}

	char *FTPRequest::data_c() {
		return reinterpret_cast<char *>(header()->data);
	}

	uint32_t *FTPRequest::data_u32() {
		return reinterpret_cast<uint32_t *>(header()->data);
	}

	/**
	 * @brief Copy string to payload
	 *
	 * @param[in] s  payload string
	 * @note this function allow null termination inside string
	 *       it used to send multiple strings in one message
	 */
	void FTPRequest::set_data_string(std::string &s) {
		size_t sz = (s.size() < DATA_MAXSZ - 1)? s.size() : DATA_MAXSZ - 1;

		memcpy(data_c(), s.c_str(), sz);
		data_c()[sz] = '\0';
		header()->size = sz;
	}

	uint8_t FTPRequest::get_target_system_id() {
		return message.target_system;
	}

	/**
	 * @brief Decode and check target system
	 */
	bool FTPRequest::decode(UAS *uas, const mavlink_message_t *msg) {
		mavlink_msg_file_transfer_protocol_decode(msg, &message);

#ifdef FTP_LL_DEBUG
		auto hdr = header();
		ROS_DEBUG_NAMED("ftp", "FTP:rm: SEQ(%u) SESS(%u) OPCODE(%u) RQOP(%u) SZ(%u) OFF(%u)",
				hdr->seqNumber, hdr->session, hdr->opcode, hdr->req_opcode, hdr->size, hdr->offset);
#endif

		return UAS_FCU(uas)->get_system_id() == message.target_system;
	}

	/**
	 * @brief Encode and send message
	 */
	void FTPRequest::send(UAS *uas, uint16_t seqNumber) {
		mavlink_message_t msg;

		auto hdr = header();
		hdr->seqNumber = seqNumber;
		hdr->req_opcode = kCmdNone;

#ifdef FTP_LL_DEBUG
		ROS_DEBUG_NAMED("ftp", "FTP:sm: SEQ(%u) SESS(%u) OPCODE(%u) SZ(%u) OFF(%u)",
				hdr->seqNumber, hdr->session, hdr->opcode, hdr->size, hdr->offset);
#endif

		mavlink_msg_file_transfer_protocol_pack_chan(UAS_PACK_CHAN(uas), &msg,
				0, // target_network
				UAS_PACK_TGT(uas),
				raw_payload());
		UAS_FCU(uas)->send_message(&msg);
	}

	FTPRequest::FTPRequest() :
		message{}
	{ }

	FTPRequest::FTPRequest(Opcode op, uint8_t session) :
		message{}
	{
		header()->session = session;
		header()->opcode = op;
	}

/**
 * @brief FTP plugin.
 */
	FTPPlugin::FTPPlugin() :
		uas(nullptr),
		op_state(OP_IDLE),
		last_send_seqnr(0),
		active_session(0),
		is_error(false),
		list_offset(0),
		read_offset(0),
		open_size(0),
		read_size(0),
		read_buffer{}
	{ }

	void FTPPlugin::initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;

		ftp_nh = ros::NodeHandle(nh, "ftp");

		list_srv = ftp_nh.advertiseService("list", &FTPPlugin::list_cb, this);
		open_srv = ftp_nh.advertiseService("open", &FTPPlugin::open_cb, this);
		close_srv = ftp_nh.advertiseService("close", &FTPPlugin::close_cb, this);
		read_srv = ftp_nh.advertiseService("read", &FTPPlugin::read_cb, this);
		write_srv = ftp_nh.advertiseService("write", &FTPPlugin::write_cb, this);
		mkdir_srv = ftp_nh.advertiseService("mkdir", &FTPPlugin::mkdir_cb, this);
		rmdir_srv = ftp_nh.advertiseService("rmdir", &FTPPlugin::rmdir_cb, this);
		remove_srv = ftp_nh.advertiseService("remove", &FTPPlugin::remove_cb, this);
		truncate_srv = ftp_nh.advertiseService("truncate", &FTPPlugin::truncate_cb, this);
		reset_srv = ftp_nh.advertiseService("reset", &FTPPlugin::reset_cb, this);
		rename_srv = ftp_nh.advertiseService("rename", &FTPPlugin::rename_cb, this);
		checksum_srv = ftp_nh.advertiseService("checksum", &FTPPlugin::checksum_cb, this);
	}

	std::string const FTPPlugin::get_name() const {
		return "FTP";
	}

	const MavRosPlugin::message_map FTPPlugin::get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL, &FTPPlugin::handle_file_transfer_protocol),
		};
	}

	//! @todo exchange speed calculation
	//! @todo diagnostics

	/* -*- message handler -*- */

	void FTPPlugin::handle_file_transfer_protocol(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		FTPRequest req;
		if (!req.decode(uas, msg)) {
			ROS_DEBUG_NAMED("ftp", "FTP: Wrong System Id, MY %u, TGT %u",
					UAS_FCU(uas)->get_system_id(), req.get_target_system_id());
			return;
		}

		const uint16_t incoming_seqnr = req.header()->seqNumber;
		const uint16_t expected_seqnr = last_send_seqnr + 1;
		if (incoming_seqnr != expected_seqnr) {
			ROS_WARN_NAMED("ftp", "FTP: Lost sync! seqnr: %u != %u",
					incoming_seqnr, expected_seqnr);
			go_idle(true, EILSEQ);
			return;
		}

		last_send_seqnr = incoming_seqnr;

		// logic from QGCUASFileManager.cc
		if (req.header()->opcode == FTPRequest::kRspAck)
			handle_req_ack(req);
		else if (req.header()->opcode == FTPRequest::kRspNak)
			handle_req_nack(req);
		else {
			ROS_ERROR_NAMED("ftp", "FTP: Unknown request response: %u", req.header()->opcode);
			go_idle(true, EBADRQC);
		}
	}

	void FTPPlugin::handle_req_ack(FTPRequest &req) {
		switch (op_state) {
		case OP_IDLE:		send_reset();			break;
		case OP_ACK:		go_idle(false);			break;
		case OP_LIST:		handle_ack_list(req);		break;
		case OP_OPEN:		handle_ack_open(req);		break;
		case OP_READ:		handle_ack_read(req);		break;
		case OP_WRITE:		handle_ack_write(req);		break;
		case OP_CHECKSUM:	handle_ack_checksum(req);	break;
		default:
			ROS_ERROR_NAMED("ftp", "FTP: wrong op_state");
			go_idle(true, EBADRQC);
		}
	}

	void FTPPlugin::handle_req_nack(FTPRequest &req) {
		auto hdr = req.header();
		auto error_code = static_cast<FTPRequest::ErrorCode>(req.data()[0]);
		OpState prev_op = op_state;

		ROS_ASSERT(hdr->size == 1 || (error_code == FTPRequest::kErrFailErrno && hdr->size == 2));

		op_state = OP_IDLE;
		if (error_code == FTPRequest::kErrFailErrno)
			r_errno = req.data()[1];
		// translate other protocol errors to errno
		else if (error_code == FTPRequest::kErrFail)
			r_errno = EFAULT;
		else if (error_code == FTPRequest::kErrInvalidDataSize)
			r_errno = EMSGSIZE;
		else if (error_code == FTPRequest::kErrInvalidSession)
			r_errno = EBADFD;
		else if (error_code == FTPRequest::kErrNoSessionsAvailable)
			r_errno = EMFILE;
		else if (error_code == FTPRequest::kErrUnknownCommand)
			r_errno = ENOSYS;

		if (prev_op == OP_LIST && error_code == FTPRequest::kErrEOF) {
			/* dir list done */
			list_directory_end();
			return;
		}
		else if (prev_op == OP_READ && error_code == FTPRequest::kErrEOF) {
			/* read done */
			read_file_end();
			return;
		}

		ROS_ERROR_NAMED("ftp", "FTP: NAK: %u Opcode: %u State: %u Errno: %d (%s)",
				error_code, hdr->req_opcode, prev_op, r_errno, strerror(r_errno));
		go_idle(true);
	}

	void FTPPlugin::handle_ack_list(FTPRequest &req) {
		auto hdr = req.header();

		ROS_DEBUG_NAMED("ftp", "FTP:m: ACK List SZ(%u) OFF(%u)", hdr->size, hdr->offset);
		if (hdr->offset != list_offset) {
			ROS_ERROR_NAMED("ftp", "FTP: Wring list offset, req %u, ret %u",
					list_offset, hdr->offset);
			go_idle(true, EBADE);
			return;
		}

		uint8_t off = 0;
		uint32_t n_list_entries = 0;

		while (off < hdr->size) {
			const char *ptr = req.data_c() + off;
			const size_t bytes_left = hdr->size - off;

			size_t slen = strnlen(ptr, bytes_left);
			if ((ptr[0] == FTPRequest::DIRENT_SKIP && slen > 1) ||
					(ptr[0] != FTPRequest::DIRENT_SKIP && slen < 2)) {
				ROS_ERROR_NAMED("ftp", "FTP: Incorrect list entry: %s", ptr);
				go_idle(true, ERANGE);
				return;
			}
			else if (slen == bytes_left) {
				ROS_ERROR_NAMED("ftp", "FTP: Missing NULL termination in list entry");
				go_idle(true, EOVERFLOW);
				return;
			}

			if (ptr[0] == FTPRequest::DIRENT_FILE ||
					ptr[0] == FTPRequest::DIRENT_DIR) {
				add_dirent(ptr, slen);
			}
			else if (ptr[0] == FTPRequest::DIRENT_SKIP) {
				// do nothing
			}
			else {
				ROS_WARN_NAMED("ftp", "FTP: Unknown list entry: %s", ptr);
			}

			off += slen + 1;
			n_list_entries++;
		}

		if (hdr->size == 0) {
			// dir empty, we are done
			list_directory_end();
		}
		else {
			ROS_ASSERT_MSG(n_list_entries > 0, "FTP:List don't parse entries");
			// Possibly more to come, try get more
			list_offset += n_list_entries;
			send_list_command();
		}
	}

	void FTPPlugin::handle_ack_open(FTPRequest &req) {
		auto hdr = req.header();

		ROS_DEBUG_NAMED("ftp", "FTP:m: ACK Open OPCODE(%u)", hdr->req_opcode);
		ROS_ASSERT(hdr->size == sizeof(uint32_t));
		open_size = *req.data_u32();

		ROS_INFO_NAMED("ftp", "FTP:Open %s: success, session %u, size %zu",
				open_path.c_str(), hdr->session, open_size);
		session_file_map.insert(std::make_pair(open_path, hdr->session));
		go_idle(false);
	}

	void FTPPlugin::handle_ack_read(FTPRequest &req) {
		auto hdr = req.header();

		ROS_DEBUG_NAMED("ftp", "FTP:m: ACK Read SZ(%u)", hdr->size);
		if (hdr->session != active_session) {
			ROS_ERROR_NAMED("ftp", "FTP:Read unexpected session");
			go_idle(true, EBADSLT);
			return;
		}

		if (hdr->offset != read_offset) {
			ROS_ERROR_NAMED("ftp", "FTP:Read different offset");
			go_idle(true, EBADE);
			return;
		}

		// kCmdReadFile return cunks of DATA_MAXSZ or smaller (last chunk)
		// We requested specific amount of data, that can be smaller,
		// but not larger.
		const size_t bytes_left = read_size - read_buffer.size();
		const size_t bytes_to_copy = std::min<size_t>(bytes_left, hdr->size);

		read_buffer.insert(read_buffer.end(), req.data(), req.data() + bytes_to_copy);

		if (bytes_to_copy == FTPRequest::DATA_MAXSZ) {
			// Possibly more data
			read_offset += bytes_to_copy;
			send_read_command();
		}
		else
			read_file_end();
	}

	void FTPPlugin::handle_ack_write(FTPRequest &req) {
		auto hdr = req.header();

		ROS_DEBUG_NAMED("ftp", "FTP:m: ACK Write SZ(%u)", hdr->size);
		if (hdr->session != active_session) {
			ROS_ERROR_NAMED("ftp", "FTP:Write unexpected session");
			go_idle(true, EBADSLT);
			return;
		}

		if (hdr->offset != write_offset) {
			ROS_ERROR_NAMED("ftp", "FTP:Write different offset");
			go_idle(true, EBADE);
			return;
		}

		ROS_ASSERT(hdr->size == sizeof(uint32_t));
		const size_t bytes_written = *req.data_u32();

		// check that reported size not out of range
		const size_t bytes_left_before_advance = std::distance(write_it, write_buffer.end());
		ROS_ASSERT_MSG(bytes_written <= bytes_left_before_advance, "Bad write size");
		ROS_ASSERT(bytes_written != 0);

		// move iterator to written size
		std::advance(write_it, bytes_written);

		const size_t bytes_to_copy = write_bytes_to_copy();
		if (bytes_to_copy > 0) {
			// More data to write
			write_offset += bytes_written;
			send_write_command(bytes_to_copy);
		}
		else
			write_file_end();
	}

	void FTPPlugin::handle_ack_checksum(FTPRequest &req) {
		auto hdr = req.header();

		ROS_DEBUG_NAMED("ftp", "FTP:m: ACK CalcFileCRC32 OPCODE(%u)", hdr->req_opcode);
		ROS_ASSERT(hdr->size == sizeof(uint32_t));
		checksum_crc32 = *req.data_u32();

		ROS_DEBUG_NAMED("ftp", "FTP:Checksum: success, crc32: 0x%08x", checksum_crc32);
		go_idle(false);
	}

	/* -*- send helpers -*- */

	/**
	 * @brief Go to IDLE mode
	 *
	 * @param is_error_ mark that caused in error case
	 * @param r_errno_ set r_errno in error case
	 */
	void FTPPlugin::go_idle(bool is_error_, int r_errno_) {
		op_state = OP_IDLE;
		is_error = is_error_;
		if (is_error && r_errno_ != 0)	r_errno = r_errno_;
		else if (!is_error)		r_errno = 0;
		cond.notify_all();
	}

	void FTPPlugin::send_reset() {
		ROS_DEBUG_NAMED("ftp", "FTP:m: kCmdResetSessions");
		if (session_file_map.size() > 0) {
			ROS_WARN_NAMED("ftp", "FTP: Reset closes %zu sessons",
					session_file_map.size());
			session_file_map.clear();
		}

		op_state = OP_ACK;
		FTPRequest req(FTPRequest::kCmdResetSessions);
		req.send(uas, last_send_seqnr);
	}

	/// Send any command with string payload (usually file/dir path)
	inline void FTPPlugin::send_any_path_command(FTPRequest::Opcode op, const std::string debug_msg, std::string &path, uint32_t offset) {
		ROS_DEBUG_STREAM_NAMED("ftp", "FTP:m: " << debug_msg << path << " off: " << offset);
		FTPRequest req(op);
		req.header()->offset = offset;
		req.set_data_string(path);
		req.send(uas, last_send_seqnr);
	}

	void FTPPlugin::send_list_command() {
		send_any_path_command(FTPRequest::kCmdListDirectory, "kCmdListDirectory: ", list_path, list_offset);
	}

	void FTPPlugin::send_open_ro_command() {
		send_any_path_command(FTPRequest::kCmdOpenFileRO, "kCmdOpenFileRO: ", open_path, 0);
	}

	void FTPPlugin::send_open_wo_command() {
		send_any_path_command(FTPRequest::kCmdOpenFileWO, "kCmdOpenFileWO: ", open_path, 0);
	}

	void FTPPlugin::send_create_command() {
		send_any_path_command(FTPRequest::kCmdCreateFile, "kCmdCreateFile: ", open_path, 0);
	}

	void FTPPlugin::send_terminate_command(uint32_t session) {
		ROS_DEBUG_STREAM_NAMED("ftp", "FTP:m: kCmdTerminateSession: " << session);
		FTPRequest req(FTPRequest::kCmdTerminateSession, session);
		req.header()->offset = 0;
		req.header()->size = 0;
		req.send(uas, last_send_seqnr);
	}

	void FTPPlugin::send_read_command() {
		// read operation always try read DATA_MAXSZ block (hdr->size ignored)
		ROS_DEBUG_STREAM_NAMED("ftp", "FTP:m: kCmdReadFile: " << active_session << " off: " << read_offset);
		FTPRequest req(FTPRequest::kCmdReadFile, active_session);
		req.header()->offset = read_offset;
		req.header()->size = 0 /* FTPRequest::DATA_MAXSZ */;
		req.send(uas, last_send_seqnr);
	}

	void FTPPlugin::send_write_command(const size_t bytes_to_copy) {
		// write chunk from write_buffer [write_it..bytes_to_copy]
		ROS_DEBUG_STREAM_NAMED("ftp", "FTP:m: kCmdWriteFile: " << active_session << " off: " << write_offset << " sz: " << bytes_to_copy);
		FTPRequest req(FTPRequest::kCmdWriteFile, active_session);
		req.header()->offset = write_offset;
		req.header()->size = bytes_to_copy;
		std::copy(write_it, write_it + bytes_to_copy, req.data());
		req.send(uas, last_send_seqnr);
	}

	void FTPPlugin::send_remove_command(std::string &path) {
		send_any_path_command(FTPRequest::kCmdRemoveFile, "kCmdRemoveFile: ", path, 0);
	}

	bool FTPPlugin::send_rename_command(std::string &old_path, std::string &new_path) {
		std::ostringstream os;
		os << old_path;
		os << '\0';
		os << new_path;

		std::string paths = os.str();
		if (paths.size() >= FTPRequest::DATA_MAXSZ) {
			ROS_ERROR_NAMED("ftp", "FTP: rename file paths is too long: %zu", paths.size());
			r_errno = ENAMETOOLONG;
			return false;
		}

		send_any_path_command(FTPRequest::kCmdRename, "kCmdRename: ", paths, 0);
		return true;
	}

	void FTPPlugin::send_truncate_command(std::string &path, size_t length) {
		send_any_path_command(FTPRequest::kCmdTruncateFile, "kCmdTruncateFile: ", path, length);
	}

	void FTPPlugin::send_create_dir_command(std::string &path) {
		send_any_path_command(FTPRequest::kCmdCreateDirectory, "kCmdCreateDirectory: ", path, 0);
	}

	void FTPPlugin::send_remove_dir_command(std::string &path) {
		send_any_path_command(FTPRequest::kCmdRemoveDirectory, "kCmdRemoveDirectory: ", path, 0);
	}

	void FTPPlugin::send_calc_file_crc32_command(std::string &path) {
		send_any_path_command(FTPRequest::kCmdCalcFileCRC32, "kCmdCalcFileCRC32: ", path, 0);
	}

	/* -*- helpers -*- */

	void FTPPlugin::add_dirent(const char *ptr, size_t slen) {
		mavros::FileEntry ent;
		ent.size = 0;

		if (ptr[0] == FTPRequest::DIRENT_DIR) {
			ent.name.assign(ptr + 1, slen - 1);
			ent.type = mavros::FileEntry::TYPE_DIRECTORY;

			ROS_DEBUG_STREAM_NAMED("ftp", "FTP:List Dir: " << ent.name);
		}
		else {
			// ptr[0] == FTPRequest::DIRENT_FILE
			std::string name_size(ptr + 1, slen - 1);

			auto sep_it = std::find(name_size.begin(), name_size.end(), '\t');
			ent.name.assign(name_size.begin(), sep_it);
			ent.type = mavros::FileEntry::TYPE_FILE;

			if (sep_it != name_size.end()) {
				name_size.erase(name_size.begin(), sep_it + 1);
				if (name_size.size() != 0)
					//ent.size = std::stoi(name_size);
					try {
						ent.size = boost::lexical_cast<int>(name_size);
					} catch( boost::bad_lexical_cast const& ) {
						std::cout << "Error: input string was not valid" << std::endl;
					}
			}

			ROS_DEBUG_STREAM_NAMED("ftp", "FTP:List File: " << ent.name << " SZ: " << ent.size);
		}

		list_entries.push_back(ent);
	}

	void FTPPlugin::list_directory_end() {
		ROS_DEBUG_NAMED("ftp", "FTP:List done");
		go_idle(false);
	}

	void FTPPlugin::list_directory(std::string &path) {
		list_offset = 0;
		list_path = path;
		list_entries.clear();
		op_state = OP_LIST;

		send_list_command();
	}

	bool FTPPlugin::open_file(std::string &path, int mode) {
		open_path = path;
		open_size = 0;
		op_state = OP_OPEN;

		if (mode == mavros::FileOpenRequest::MODE_READ)
			send_open_ro_command();
		else if (mode == mavros::FileOpenRequest::MODE_WRITE)
			send_open_wo_command();
		else if (mode == mavros::FileOpenRequest::MODE_CREATE)
			send_create_command();
		else {
			ROS_ERROR_NAMED("ftp", "FTP: Unsupported open mode: %d", mode);
			op_state = OP_IDLE;
			r_errno = EINVAL;
			return false;
		}

		return true;
	}

	bool FTPPlugin::close_file(std::string &path) {
		auto it = session_file_map.find(path);
		if (it == session_file_map.end()) {
			ROS_ERROR_NAMED("ftp", "FTP:Close %s: not opened", path.c_str());
			r_errno = EBADF;
			return false;
		}

		op_state = OP_ACK;
		send_terminate_command(it->second);
		session_file_map.erase(it);
		return true;
	}

	void FTPPlugin::read_file_end() {
		ROS_DEBUG_NAMED("ftp", "FTP:Read done");
		go_idle(false);
	}

	bool FTPPlugin::read_file(std::string &path, size_t off, size_t len) {
		auto it = session_file_map.find(path);
		if (it == session_file_map.end()) {
			ROS_ERROR_NAMED("ftp", "FTP:Read %s: not opened", path.c_str());
			r_errno = EBADF;
			return false;
		}

		op_state = OP_READ;
		active_session = it->second;
		read_size = len;
		read_offset = off;
		read_buffer.clear();
		if (read_buffer.capacity() < len ||
				read_buffer.capacity() > len + MAX_RESERVE_DIFF) {
			// reserve memory
			read_buffer.reserve(len);
		}

		send_read_command();
		return true;
	}

	void FTPPlugin::write_file_end() {
		ROS_DEBUG_NAMED("ftp", "FTP:Write done");
		go_idle(false);
	}

	bool FTPPlugin::write_file(std::string &path, size_t off, V_FileData &data) {
		auto it = session_file_map.find(path);
		if (it == session_file_map.end()) {
			ROS_ERROR_NAMED("ftp", "FTP:Write %s: not opened", path.c_str());
			r_errno = EBADF;
			return false;
		}

		op_state = OP_WRITE;
		active_session = it->second;
		write_offset = off;
		write_buffer = std::move(data);
		write_it = write_buffer.begin();

		send_write_command(write_bytes_to_copy());
		return true;
	}

	void FTPPlugin::remove_file(std::string &path) {
		op_state = OP_ACK;
		send_remove_command(path);
	}

	bool FTPPlugin::rename_(std::string &old_path, std::string &new_path) {
		op_state = OP_ACK;
		return send_rename_command(old_path, new_path);
	}

	void FTPPlugin::truncate_file(std::string &path, size_t length) {
		op_state = OP_ACK;
		send_truncate_command(path, length);
	}

	void FTPPlugin::create_directory(std::string &path) {
		op_state = OP_ACK;
		send_create_dir_command(path);
	}

	void FTPPlugin::remove_directory(std::string &path) {
		op_state = OP_ACK;
		send_remove_dir_command(path);
	}

	void FTPPlugin::checksum_crc32_file(std::string &path) {
		op_state = OP_CHECKSUM;
		checksum_crc32 = 0;
		send_calc_file_crc32_command(path);
	}

	constexpr int FTPPlugin::compute_rw_timeout(size_t len) {
		return CHUNK_TIMEOUT_MS * (len / FTPRequest::DATA_MAXSZ + 1);
	}

	size_t FTPPlugin::write_bytes_to_copy() {
		return std::min<size_t>(std::distance(write_it, write_buffer.end()),
				FTPRequest::DATA_MAXSZ);
	}

	bool FTPPlugin::wait_completion(const int msecs) {
		std::unique_lock<std::mutex> lock(cond_mutex);

		bool is_timedout = cond.wait_for(lock, std::chrono::milliseconds(msecs))
			== std::cv_status::timeout;

		if (is_timedout) {
			// If timeout occurs don't forget to reset state
			op_state = OP_IDLE;
			r_errno = ETIMEDOUT;
			return false;
		}
		else
			// if go_idle() occurs before timeout
			return !is_error;
	}

	/* -*- service callbacks -*- */

	/**
	 * Service handler common header code.
	 */
#define SERVICE_IDLE_CHECK()				\
	if (op_state != OP_IDLE) {			\
		ROS_ERROR_NAMED("ftp", "FTP: Busy");	\
		return false;				\
	}

	bool FTPPlugin::list_cb(mavros::FileList::Request &req,
			mavros::FileList::Response &res) {
		SERVICE_IDLE_CHECK();

		list_directory(req.dir_path);
		res.success = wait_completion(LIST_TIMEOUT_MS);
		res.r_errno = r_errno;
		if (res.success) {
			res.list = std::move(list_entries);
			list_entries.clear();	// not shure that it's needed
		}

		return true;
	}

	bool FTPPlugin::open_cb(mavros::FileOpen::Request &req,
			mavros::FileOpen::Response &res) {
		SERVICE_IDLE_CHECK();

		// only one session per file
		auto it = session_file_map.find(req.file_path);
		if (it != session_file_map.end()) {
			ROS_ERROR_NAMED("ftp", "FTP: File %s: already opened",
					req.file_path.c_str());
			return false;
		}

		res.success = open_file(req.file_path, req.mode);
		if (res.success) {
			res.success = wait_completion(OPEN_TIMEOUT_MS);
			res.size = open_size;
		}
		res.r_errno = r_errno;

		return true;
	}

	bool FTPPlugin::close_cb(mavros::FileClose::Request &req,
			mavros::FileClose::Response &res) {
		SERVICE_IDLE_CHECK();

		res.success = close_file(req.file_path);
		if (res.success) {
			res.success = wait_completion(OPEN_TIMEOUT_MS);
		}
		res.r_errno = r_errno;

		return true;
	}

	bool FTPPlugin::read_cb(mavros::FileRead::Request &req,
			mavros::FileRead::Response &res) {
		SERVICE_IDLE_CHECK();

		res.success = read_file(req.file_path, req.offset, req.size);
		if (res.success)
			res.success = wait_completion(compute_rw_timeout(req.size));
		if (res.success) {
			res.data = std::move(read_buffer);
			read_buffer.clear();	// same as for list_entries
		}
		res.r_errno = r_errno;

		return true;
	}

	bool FTPPlugin::write_cb(mavros::FileWrite::Request &req,
			mavros::FileWrite::Response &res) {
		SERVICE_IDLE_CHECK();

		const size_t data_size = req.data.size();
		res.success = write_file(req.file_path, req.offset, req.data);
		if (res.success) {
			res.success = wait_completion(compute_rw_timeout(data_size));
		}
		write_buffer.clear();
		res.r_errno = r_errno;

		return true;
	}

	bool FTPPlugin::remove_cb(mavros::FileRemove::Request &req,
			mavros::FileRemove::Response &res) {
		SERVICE_IDLE_CHECK();

		remove_file(req.file_path);
		res.success = wait_completion(OPEN_TIMEOUT_MS);
		res.r_errno = r_errno;

		return true;
	}

	bool FTPPlugin::rename_cb(mavros::FileRename::Request &req,
			mavros::FileRename::Response &res) {
		SERVICE_IDLE_CHECK();

		res.success = rename_(req.old_path, req.new_path);
		if (res.success) {
			res.success = wait_completion(OPEN_TIMEOUT_MS);
		}
		res.r_errno = r_errno;

		return true;
	}


	bool FTPPlugin::truncate_cb(mavros::FileTruncate::Request &req,
			mavros::FileTruncate::Response &res) {
		SERVICE_IDLE_CHECK();

		// Note: emulated truncate() can take a while
		truncate_file(req.file_path, req.length);
		res.success = wait_completion(LIST_TIMEOUT_MS * 5);
		res.r_errno = r_errno;

		return true;
	}

	bool FTPPlugin::mkdir_cb(mavros::FileMakeDir::Request &req,
			mavros::FileMakeDir::Response &res) {
		SERVICE_IDLE_CHECK();

		create_directory(req.dir_path);
		res.success = wait_completion(OPEN_TIMEOUT_MS);
		res.r_errno = r_errno;

		return true;
	}

	bool FTPPlugin::rmdir_cb(mavros::FileRemoveDir::Request &req,
			mavros::FileRemoveDir::Response &res) {
		SERVICE_IDLE_CHECK();

		remove_directory(req.dir_path);
		res.success = wait_completion(OPEN_TIMEOUT_MS);
		res.r_errno = r_errno;

		return true;
	}

	bool FTPPlugin::checksum_cb(mavros::FileChecksum::Request &req,
			mavros::FileChecksum::Response &res) {
		SERVICE_IDLE_CHECK();

		checksum_crc32_file(req.file_path);
		res.success = wait_completion(LIST_TIMEOUT_MS);
		res.crc32 = checksum_crc32;
		res.r_errno = r_errno;

		return true;
	}

#undef SERVICE_IDLE_CHECK

	/**
	 * @brief Reset communication on both sides.
	 * @note This call break other calls, so use carefully.
	 */
	bool FTPPlugin::reset_cb(std_srvs::Empty::Request &req,
			std_srvs::Empty::Response &res) {
		send_reset();
		return true;
	}

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::FTPPlugin, mavplugin::MavRosPlugin)

