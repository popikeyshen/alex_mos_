/*
  Alexmos Serial controlled mount backend class
*/
#pragma once

#include "AP_Mount.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_AHRS/AP_AHRS.h>
#include "AP_Mount_Backend.h"



//definition of the commands id for the Alexmos Serial Protocol
#define CMD_READ_PARAMS 'R'
#define CMD_WRITE_PARAMS 'W'
#define CMD_REALTIME_DATA 'D'
#define CMD_BOARD_INFO 'V'
#define CMD_CALIB_ACC 'A'
#define CMD_CALIB_GYRO 'g'
#define CMD_CALIB_EXT_GAIN 'G'
#define CMD_USE_DEFAULTS 'F'
#define CMD_CALIB_POLES 'P'
#define CMD_RESET 'r'
#define CMD_HELPER_DATA 'H'
#define CMD_CALIB_OFFSET 'O'
#define CMD_CALIB_BAT 'B'
#define CMD_MOTORS_ON 'M'
#define CMD_MOTORS_OFF 'm'
#define CMD_CONTROL 'C'
#define CMD_TRIGGER_PIN 'T'
#define CMD_EXECUTE_MENU 'E'
#define CMD_GET_ANGLES 'I'
#define CMD_CONFIRM 'C'
// Board v3.x only
#define CMD_BOARD_INFO_3 20
#define CMD_READ_PARAMS_3 21
#define CMD_WRITE_PARAMS_3 22
#define CMD_REALTIME_DATA_3 23
#define CMD_SELECT_IMU_3 24
#define CMD_READ_PROFILE_NAMES 28
#define CMD_WRITE_PROFILE_NAMES 29
#define CMD_QUEUE_PARAMS_INFO_3 30
#define CMD_SET_PARAMS_3 31
#define CMD_SAVE_PARAMS_3 32
#define CMD_READ_PARAMS_EXT 33
#define CMD_WRITE_PARAMS_EXT 34
#define CMD_AUTO_PID 35
#define CMD_SERVO_OUT 36
#define CMD_ERROR 255

#define AP_MOUNT_ALEXMOS_MODE_NO_CONTROL 0
#define AP_MOUNT_ALEXMOS_MODE_SPEED 1
#define AP_MOUNT_ALEXMOS_MODE_ANGLE 2
#define AP_MOUNT_ALEXMOS_MODE_SPEED_ANGLE 3
#define AP_MOUNT_ALEXMOS_MODE_RC 4

#define AP_MOUNT_ALEXMOS_SPEED 30 // degree/s2

#define VALUE_TO_DEGREE(d) ((float)((d * 720) >> 15))
#define DEGREE_TO_VALUE(d) ((int16_t)((float)(d)*(1.0f/0.02197265625f)))
#define DEGREE_PER_SEC_TO_VALUE(d) ((int16_t)((float)(d)*(1.0f/0.1220740379f)))

class AP_Mount_Alexmos : public AP_Mount_Backend
{
public:
    //constructor
    AP_Mount_Alexmos(AP_Mount &frontend, AP_Mount::mount_state &state, uint8_t instance):
        AP_Mount_Backend(frontend, state, instance),
        _port(nullptr),
        _initialised(false),
        _board_version(0),
        _current_firmware_version(0.0f),
        _firmware_beta_version(0),
        _gimbal_3axis(false),
        _gimbal_bat_monitoring(false),
        _current_angle(0,0,0),
        _param_read_once(false),
        _checksum(0),
        _step(0),
        _command_id(0),
        _payload_length(0),
        _payload_counter(0),
        _last_command_confirmed(false)
    {}

	int time1000 = 0;
    // init - performs any required initialisation for this instance
    virtual void init(const AP_SerialManager& serial_manager);

    // update mount position - should be called periodically
    virtual void update();

    // has_pan_control - returns true if this mount can control it's pan (required for multicopters)
    virtual bool has_pan_control() const;

    // set_mode - sets mount's mode
    virtual void set_mode(enum MAV_MOUNT_MODE mode) ;

    // status_msg - called to allow mounts to send their status to GCS via MAVLink
    virtual void status_msg(mavlink_channel_t chan) ;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	const uint8_t
		MASTER_ID = 0x01, // this controller
		SLAVE_ID = 0xFF;  // gimbal

						  // ---------------------------------------------------------------------------------- INLINE AND MACRO ---

	template<class T> inline void reverseByteOrderInPlace(T &data)
	{
		uint8_t *firstByte = reinterpret_cast<uint8_t *>(&data);
		uint8_t *lastByte = reinterpret_cast<uint8_t *>(&data) + sizeof(T) - 1;
		while (firstByte < lastByte)
		{
			*firstByte ^= *lastByte; // exchange byte values
			*lastByte ^= *firstByte;
			*firstByte ^= *lastByte;
			++firstByte;             // go next bytes pair
			--lastByte;
		}
	}


	struct Bytes
	{
	public:
		Bytes() : ptr(nullptr), len(0)
		{ }
		static Bytes from(uint8_t *addr, int16_t size)
		{
			return Bytes(addr, size);
		}
		template<class T> static const Bytes from(const T &data)
		{
			return Bytes(reinterpret_cast<uint8_t *>(&const_cast<T&>(data)), sizeof(T));
		}
		template<class T> static Bytes from(T &data)
		{
			return Bytes(reinterpret_cast<uint8_t *>(&data), sizeof(T));
		}
	public:
		uint8_t *const ptr;
		const int16_t len;
	private:
		Bytes(uint8_t *addr, int16_t size) : ptr(addr), len(size)
		{
			//ASSERT_M(ptr != 0, "(Bytes)");
			//ASSERT_M(len >= 0, "(Bytes)");
		}
	};




	/* To control the gimbal we should transmit some messages in correct format,
	receive response, check correctness and integrity and analize values in arguments field

	The structure of message is:
	- IR (receiver identity)
	- IT (transmitter identity - our master controller)
	- CC (message class code)
	- MC (message code)
	- A (arguments - command parameters, requested values)
	- MSB (most significant byte of CRC16 checksum)
	- LSB (least significant byte of CRC16 checksum)
	*/

	// makes structures definition fast and easy


#define DEF_INTPAIR(NAME,T,firstElem,U,secondElem) struct NAME {\
      NAME() {} \
      NAME(T firstElem##_, U secondElem##_) \
        : firstElem(firstElem##_)   \
        , secondElem(secondElem##_) \
      {} \
      bool operator ==(const NAME & rhs) const { return firstElem == rhs.firstElem && secondElem == rhs.secondElem; } \
      bool operator !=(const NAME & rhs) const { return !(*this == rhs); } \
      T firstElem;  \
      U secondElem;  \
    }

	// typedefs were structures used for implementation of strong typing in earlier code revisions
	typedef uint8_t ReceiverID;
	typedef uint8_t TransmitterID;
	typedef uint8_t Gimbal_MessageCode;
	typedef uint8_t Administration_MessageCode;
	typedef uint8_t LaserRangeFinder_MessageCode;
	typedef uint8_t VideoTracker_MessageCode;
	typedef uint8_t VideoRecorder_MessageCode;
	typedef uint8_t VideoOutput_MessageCode;
	typedef uint8_t GeoLocationAndPositioning_MessageCode;
	typedef uint8_t Camera_MessageCode;
	typedef uint8_t DeviceID;

	// enum class (c++11) -- strongly typed enum, does not emit member names to the outer scope


	enum class ClassCode : uint8_t // class of messages forwarded to the gimbal
	{
		ADMINISTRATION = 0x00,
		GIMBAL = 0x01,
		CAMERA_A = 0x02,
		LASER_RANGE_FINDER = 0x03,
		VIDEO_TRACKER = 0x04,
		VIDEO_RECORDER = 0x05,
		VIDEO_OUTPUT = 0x06,
		GEO_LOCATION_POSITIONING = 0x07,
		CAMERA_B = 0x12
	};

	/* Following namespaces contain message codes for each message class
	and also some necessary argument types.
	This code revision uses Gimbal, VideoOutput and Camera classes only.
	*/

	typedef uint32_t MediaIdentity;
	DEF_INTPAIR(MediaAndDeviceIdentity, MediaIdentity, MI, DeviceID, DI);

	static const Administration_MessageCode
		ADMIN_GET_IMPLEMENTED_MESSAGES = 0x00,
		ADMIN_GET_VERSION = 0x01,
		ADMIN_GET_STATUS = 0x02,
		ADMIN_PING = 0x03,
		ADMIN_GET_MEDIA_ACCESS_IDENTITY = 0x04,
		ADMIN_SET_DEVICE_IDENTITY = 0x05,
		ADMIN_GET_DEVICE_IDENTITY = 0x06,
		ADMIN_GET_MESSAGE_CLASSES = 0x07,
		ADMIN_IMPLEMENTED_MESSAGES = 0x80,
		ADMIN_VERSION = 0x81,
		ADMIN_STATUS = 0x82,
		ADMIN_PING_REPLY = 0x83,
		ADMIN_MEDIA_IDENTITY = 0x84,
		ADMIN_MESSAGE_CLASSES = 0x85;



	enum class Gimbal_Mode : uint8_t
	{
		JOYSTICK_SPEED_CONTROL = 0x0,
		STOW = 0x1,
		GEO_POSITIONING = 0x2, /*platform dependent*/
		VIDEO_TRACKING = 0x3,  /*platform dependent*/
		ENCODER_POSITIONING = 0x4,
		INITIALISE = 0x5,
		FIXED_YAW_AND_PITCH = 0x6 /*platform dependent*/
	};
	DEF_INTPAIR(Position, int16_t, azimuth, int16_t, elevation);
	DEF_INTPAIR(Rate, int16_t, azimuth, int16_t, elevation);

	static const Gimbal_MessageCode
		GIMBAL_GET_IMPLEMENTED_MESSAGES = 0x00,
		GIMBAL_GET_VERSION = 0x01,
		GIMBAL_GET_STATUS = 0x02,
		GIMBAL_SET_MODE = 0x03,
		GIMBAL_GET_MODE = 0x04,
		GIMBAL_SET_RATE = 0x05,
		GIMBAL_GET_RATE = 0x06,
		GIMBAL_SET_POSITION = 0x07,
		GIMBAL_GET_POSITION = 0x08,
		GIMBAL_SET_GYRO_BIAS = 0x09,
		GIMBAL_GET_GYRO_BIAS = 0x0A,
		GIMBAL_GET_TEMPERATURES = 0x0B,
		GIMBAL_SET_BIAS_ESTIMATE_MODE = 0x0C,
		GIMBAL_GET_BIAS_ESTIMATE_MODE = 0x0D,
		GIMBAL_SET_ENCODER_ZERO_POINT = 0x0E,
		GIMBAL_GET_ENCODER_ZERO_POINT = 0x0F,
		GIMBAL_SET_INCREMENT_ANGLES = 0x10,
		GIMBAL_SET_INCREMENT_PIXELS = 0x11,
		GIMBAL_SET_RATE_AND_GET_POSITION = 0x40,
		GIMBAL_IMPLEMENTED_MESSAGES = 0x80,
		GIMBAL_VERSION = 0x81,
		GIMBAL_STATUS = 0x82,
		GIMBAL_MODE = 0x83,
		GIMBAL_RATE = 0x84,
		GIMBAL_POSITION = 0x85,
		GYRO_BIAS = 0x86,
		GIMBAL_TEMPERATURES = 0x87,
		GIMBAL_BIAS_ESTIMATE_MODE = 0x88,
		GIMBAL_ENCODER_ZERO_POINT = 0x89,
		GIMBAL_INCREMENT_ANGLES = 0x8A,
		GIMBAL_RATE_AND_POSITION = 0xC0;


	static const LaserRangeFinder_MessageCode
		LASER_GET_MESSAGES = 0x00,
		LASER_GET_VERSION = 0x01,
		LASER_GET_STATUS = 0x02,
		LASER_SET_MODE = 0x03,
		LASER_GET_DISTANCE = 0x04,
		LASER_GET_MODE = 0x05,
		LASER_IMPLEMENTED_MESSAGES = 0x80,
		LASER_VERSION = 0x81,
		LASER_STATUS = 0x82,
		LASER_MODE = 0x83,
		LASER_DISTANCE = 0x84;


	static const VideoTracker_MessageCode
		VIDEOTR_GET_IMPLEMENTED_MESSAGES = 0x00,
		VIDEOTR_GET_VERSION = 0x01,
		VIDEOTR_GET_STATUS = 0x02,
		VIDEOTR_SET_THRESHOLDS = 0x03,
		VIDEOTR_GET_THRESHOLDS = 0x04,
		VIDEOTR_IMPLEMENTED_MESSAGES = 0x80,
		VIDEOTR_VERSION = 0x81,
		VIDEOTR_STATUS = 0x82,
		VIDEOTR_THRESHOLDS = 0x83;


	static const VideoRecorder_MessageCode
		VIDEOR_GET_IMPLEMENTED_MESSAGES = 0x00,
		VIDEOR_GET_VERSION = 0x01,
		VIDEOR_GET_STATUS = 0x02,
		VIDEOR_SET_MODE = 0x03,
		VIDEOR_GET_MODE = 0x04,
		VIDEOR_GET_SOURCES = 0x05,
		VIDEOR_GET_DESTINATIONS = 0x06,
		VIDEOR_GET_COMPATIBLE_SOURCES = 0x07,
		VIDEOR_GET_COMPATIBLE_DESTINATIONS = 0x08,
		VIDEOR_SET_CHANNEL_CONFIGURATION = 0x09,
		VIDEOR_GET_CHANNEL_CONFIGURATION = 0x0A,
		VIDEOR_IMPLEMENTED_MESSAGES = 0x80,
		VIDEOR_VERSION = 0x81,
		VIDEOR_STATUS = 0x82,
		VIDEOR_MODE = 0x83,
		VIDEOR_SOURCES = 0x84,
		VIDEOR_DESTINATIONS = 0x85,
		VIDEOR_COMPATIBLE_SOURCES = 0x86,
		VIDEOR_COMPATIBLE_DESTINATIONS = 0x87,
		VIDEOR_CHANNEL_CONFIGURATION = 0x88;


	typedef uint8_t VideoSource;

	static const VideoOutput_MessageCode
		VO_GET_IMPLEMENTED_MESSAGES = 0x00,
		VO_GET_VERSION = 0x01,
		VO_SET_OVERLAY_ENABLE = 0x02,
		VO_GET_OVERLAY_ENABLE = 0x03,
		VO_SET_VIDEO_SOURCE = 0x04,
		VO_GET_VIDEO_SOURCE = 0x05,
		VO_SET_CROSS_HAIR_INDICATOR_MODE = 0x06,
		VO_GET_CROSS_HAIR_INDICATOR_MODE = 0x07,
		VO_SET_ATTITUDE_INDICATOR_MODE = 0x08,
		VO_GET_ATTITUDE_INDICATOR_MODE = 0x09,
		VO_SET_OVERLAY_PARAMETER_VALUE = 0x0A,
		VO_SET_OVERLAY_STRING = 0x0B,
		VO_IMPLEMENTED_MESSAGES = 0x80,
		VO_VERSION = 0x81,
		VO_OVERLAY_ENABLE = 0x82,
		VO_VIDEO_SOURCE = 0x83,
		VO_CROSS_HAIR_INDICATOR_MODE = 0x84,
		VO_ATTITUDE_INDICATOR_MODE = 0x85,
		VO_OVERLAY_PARAMETER_VALUE = 0x86,
		VO_OVERLAY_STRING_ACKNOWLEDGE = 0x87;


	static const GeoLocationAndPositioning_MessageCode
		GEO_GET_IMPLEMENTED_MESSAGES = 0x00,
		GEO_GET_VERSION = 0x01,
		GEO_SET_HORIZONTAL_POSITION = 0x02,
		GEO_SET_ALTITUDE = 0x03,
		GEO_SET_HEADING = 0x04,
		GEO_SET_TARGET_POSITION = 0x05,
		GEO_GET_TARGET_POSITION = 0x06,
		GEO_GET_ATTITUDE = 0x07,
		GEO_GET_TARGET_POSITION_EXTENDED = 0x08,
		GEO_SET_INS_MOUNTING_OFFSET = 0x09,
		GEO_GET_INS_MOUNTING_OFFSET = 0x0A,
		GEO_SET_FIXED_ANGLE = 0x0B,
		GEO_GET_FIXED_ANGLE = 0x0C,
		GEO_MESSAGES = 0x80,
		GEO_VERSION = 0x81,
		GEO_HORIZONTAL_POSITION = 0x82,
		GEO_ALTITUDE = 0x83,
		GEO_HEADING = 0x84,
		GEO_TARGET_POSITION = 0x85,
		GEO_ATTITUDE = 0x86,
		GEO_TARGET_POSITION_EXTENDED = 0x87,
		GEO_INS_MOUNTING_OFFSET = 0x88,
		GEO_FIXED_ANGLE = 0x89;


	typedef uint16_t HorizontalFieldOfView;
	typedef int8_t ZoomRate;
	DEF_INTPAIR(HorizontalFieldOfViewLimits, uint16_t, maximum, uint16_t, minimum);

	static const Camera_MessageCode
		CAMERA_GET_IMPLEMENTED_MESSAGES = 0x00,
		CAMERA_GET_VERSION = 0x01,
		CAMERA_GET_STATUS = 0x02,
		CAMERA_SET_HORIZONTAL_FIELD_OF_VIEW = 0x03,
		CAMERA_GET_HORIZONTAL_FIELD_OF_VIEW = 0x04,
		CAMERA_GET_HORIZONTAL_FIELD_OF_VIEW_LIMITS = 0x05,
		CAMERA_SET_ZOOM_RATE = 0x06,
		CAMERA_SET_FOCUS_MODE = 0x07,
		CAMERA_GET_FOCUS_MODE = 0x08,
		CAMERA_SET_FOCUS_RATE = 0x09,
		CAMERA_SET_FOCUS_POSITION = 0x0A,
		CAMERA_GET_FOCUS_POSITION = 0x0B,
		CAMERA_SET_VIDEO_SIGNAL_MODE = 0x0C,
		CAMERA_GET_VIDEO_SIGNAL_MODE = 0x0D,
		CAMERA_SET_LOW_LIGHT_MODE = 0x0E,
		CAMERA_GET_LOW_LIGHT_MODE = 0x0F,
		CAMERA_GET_DRIVER_ID = 0x10,
		CAMERA_SET_EXPOSURE_MODE = 0x11,
		CAMERA_GET_EXPOSURE_MODE = 0x12,
		CAMERA_SET_EXPOSURE_SETTINGS = 0x13,
		CAMERA_GET_EXPOSURE_SETTINGS = 0x14,
		CAMERA_SET_EXPOSURE_COMPENSATION = 0x15,
		CAMERA_GET_EXPOSURE_COMPENSATION = 0x16,
		CAMERA_SET_BACKLIGHT_COMPENSATION = 0x17,
		CAMERA_GET_BACKLIGHT_COMPENSATION = 0x18,
		CAMERA_FLIP_IMAGE = 0x19,
		CAMERA_IMPLEMENTED_MESSAGES = 0x80,
		CAMERA_VERSION = 0x81,
		CAMERA_STATUS = 0x82,
		CAMERA_HORIZONTAL_FIELD_OF_VIEW = 0x83,
		CAMERA_HORIZONTAL_FIELD_OF_VIEW_LIMITS = 0x84,
		CAMERA_ZOOM_RATE = 0x85,
		CAMERA_FOCUS_MODE = 0x86,
		CAMERA_FOCUS_RATE = 0x87,
		CAMERA_FOCUS_POSITION = 0x88,
		CAMERA_VIDEO_SIGNAL_MODE = 0x89,
		CAMERA_LOW_LIGHT_MODE = 0x8A,
		CAMERA_DRIVER_ID = 0x8B,
		CAMERA_EXPOSURE_MODE = 0x8C,
		CAMERA_EXPOSURE_SETTINGS = 0x8D,
		CAMERA_EXPOSURE_COMPENSATION = 0x8E,
		CAMERA_BACKLIGHT_COMPENSATION = 0x8F,
		CAMERA_IMAGE_FLIP_ACKNOWLEDGE = 0x9A;




	//////////////////////////////////  SENDER ////////////////////////////////////////////////////////////////////
	struct MsgHead
	{
		uint8_t IR, IT, CC, MC;
	};
	struct MsgTail
	{
		uint8_t MSB, LSB;
	};
	// MSG = FLAG_BYTE + HEAD + ARGS + TAIL + FLAG_BYTE
	static const uint8_t MSG_BUFFER_SIZE = 32;
	static const uint8_t FLAG_BYTE = 0x7E, ESCAPE_BYTE = 0x7D;
	uint8_t m_buf[MSG_BUFFER_SIZE];
	uint8_t *m_end;

	void stuffAndTransmitByte(uint8_t b) const;

	void transmitMsg();

	uint16_t calcCRC16(const uint8_t *dataPtr, const uint8_t *dataEnd)
	{
		int16_t dataLen = dataEnd - dataPtr;
		//ASSERT(dataLen >= 4);
		uint16_t crc = 0xFFFF;
		uint8_t x;
		while (dataLen--)
		{
			x = crc >> 8 ^ *dataPtr++;
			x ^= x >> 4;
			crc = uint16_t(crc << 8) ^ uint16_t(x << 12) ^ uint16_t(x << 5) ^ uint16_t(x);
		}
		return crc;
	}

	void buildMsg(TransmitterID IT, ReceiverID IR, ClassCode CC, uint8_t MessageCode, Bytes arguments)
	{

		// BUILD PROLOGUE
		MsgHead *head = reinterpret_cast<MsgHead *>(m_buf);
		head->IR = IR;
		head->IT = IT;
		head->CC = uint8_t(CC);
		head->MC = MessageCode;
		// SAVE ARGUMENT VALUE
		uint8_t *argPtr = reinterpret_cast<uint8_t *>(head) + sizeof(MsgHead);
		//ASSERT(argPtr - m_buf == 4);
		for (uint8_t i = 0; i < arguments.len; ++i) argPtr[i] = arguments.ptr[i];
		// CALC AND SET CHECKSUM
		MsgTail *tail = reinterpret_cast<MsgTail *>(argPtr + arguments.len);
		uint16_t cs = calcCRC16(
			reinterpret_cast<uint8_t *>(head),
			reinterpret_cast<uint8_t *>(tail)
		);
		tail->LSB = uint8_t(cs & 0xFF);
		tail->MSB = uint8_t(cs >> 8);
		// END
		m_end = reinterpret_cast<uint8_t *>(tail) + sizeof(MsgTail);
		//ASSERT_M(m_end - m_buf == 6 + arguments.len, "MSGEND INCORR");
	}

	enum class ReceiveStatus : int8_t
	{
		END_OF_MSG = 0,
		TIMEOUT = 1
	};

	ReceiveStatus receiveMsg()
	{
		//m_end = m_buf;
		//m_buf[0] = 0;
		//	for (;;)
		//	{
		//read//		if (Serial.available() > 0)
		//{
		//	uint8_t b = Serial.read();
		//	if (b == FLAG_BYTE)
		//	{
		//		if (m_buf < m_end) return ReceiveStatus::END_OF_MSG;
		//	}
		//	else if (b == ESCAPE_BYTE)
		//	{
		//		*m_end = ESCAPE_BYTE;
		//	}
		//	else if (*m_end == ESCAPE_BYTE)
		//	{
		//		*m_end = b ^ 32;
		//		++m_end;
		//	}
		//	else
		//	{
		//		*m_end = b;
		//		//ASSERT(*m_end != FLAG_BYTE);
		//		++m_end;
		//	}
		//	}
		//	else
		//	{
		//		//delay(1);
		//		if (Serial.available() == 0) return ReceiveStatus::TIMEOUT;
		//	}
		//}
		return ReceiveStatus::TIMEOUT;
	}

	enum class ParseErrCode : int8_t
	{
		// NOMSG, SHORTMSG, CLSCODE, MSGCODE, RECEIVERID, SENDERID, CHKSUM, ARGSIZE, OK
		PARSECODE_NOMSG = 0,
		PARSECODE_SHORTMSG = 1,
		PARSECODE_CLSCODE = 2,
		PARSECODE_MSGCODE = 3,
		PARSECODE_RECEIVERID = 4,
		PARSECODE_SENDERID = 5,
		PARSECODE_CHKSUM = 6,
		PARSECODE_ARGSIZE = 7,
		PARSECODE_OK = 8
	};

	ParseErrCode tryParseMsg(TransmitterID IT, ReceiverID IR, ClassCode CC, uint8_t MessageCode, Bytes arguments)
	{
		if (m_buf == m_end) return ParseErrCode::PARSECODE_NOMSG;
		if (m_buf + sizeof(MsgHead) + sizeof(MsgTail) > m_end) return ParseErrCode::PARSECODE_SHORTMSG;

		MsgHead *h = reinterpret_cast<MsgHead *>(m_buf);
		MsgTail *t = reinterpret_cast<MsgTail *>(m_end) - 1;
		uint8_t *a = reinterpret_cast<uint8_t *>(h) + sizeof(MsgHead);
		//ASSERT(a - (uint8_t *)h == 4);
		//ASSERT(0 <= reinterpret_cast<uint8_t *>(t) - a);
		//ASSERT_M(msgSize() > 0, msgSize());

		uint16_t cs = calcCRC16(reinterpret_cast<uint8_t *>(h), reinterpret_cast<uint8_t *>(t));
		if (uint8_t(cs >> 8) != t->MSB || uint8_t(cs) != t->LSB) return ParseErrCode::PARSECODE_CHKSUM;

		if (uint8_t(CC) != h->CC) return ParseErrCode::PARSECODE_CLSCODE;
		if (uint8_t(MessageCode) != h->MC) return ParseErrCode::PARSECODE_MSGCODE;

		Bytes args = Bytes::from(a, reinterpret_cast<uint8_t *>(t) - a);
		if (arguments.len != args.len) return ParseErrCode::PARSECODE_ARGSIZE;

		if (uint8_t(IR) != h->IR && uint8_t(0x00) != h->IR) return ParseErrCode::PARSECODE_RECEIVERID;
		if (uint8_t(IT) != h->IT && uint8_t(0xFF) != h->IT) return ParseErrCode::PARSECODE_SENDERID;

		for (uint8_t i = 0; i < args.len; ++i) arguments.ptr[i] = args.ptr[i];
		return ParseErrCode::PARSECODE_OK;
	}

	void new_transmission(ClassCode CC, uint8_t requestMC, uint8_t expectMC, const Bytes args, Bytes retVal)
	{
		buildMsg(TransmitterID(MASTER_ID), ReceiverID(SLAVE_ID), ClassCode(CC), uint8_t(requestMC), args);

		//for (;;)
		//{
			transmitMsg();

			ReceiveStatus sts = receiveMsg();

			auto err = tryParseMsg(TransmitterID(SLAVE_ID), ReceiverID(MASTER_ID), ClassCode(CC), uint8_t(expectMC), retVal);

		//	if (err == ParseErrCode::PARSECODE_OK)
		//	{
		//		break;
		//	}

		//}
	}





	////////////////////////////////////////////////////////////////////////////////


	HorizontalFieldOfView getHorizontalFieldOfView()
	{
		HorizontalFieldOfView hfView;
		new_transmission(
			ClassCode::CAMERA_A,
			CAMERA_GET_HORIZONTAL_FIELD_OF_VIEW,
			CAMERA_HORIZONTAL_FIELD_OF_VIEW,
			Bytes(),
			Bytes::from<HorizontalFieldOfView>(hfView)
		);
		reverseByteOrderInPlace(hfView);  // set bytes order as little-endian
										  // every time a gimbal|arduino is powered on, minimum and maximum fields differ from previous ones
		return hfView;
	}


	/////////////////////////////////////////////////////////////////

private:

    // get_angles -
    void get_angles();

    // set_motor will activate motors if true, and disable them if false
    void set_motor(bool on);

    // get_boardinfo - get board version and firmware version
    void get_boardinfo();

    // control_axis - send new angles to the gimbal at a fixed speed of 30 deg/s
    void control_axis(const Vector3f& angle , bool targets_in_degrees);

    // read_params - read current profile profile_id and global parameters from the gimbal settings
    void read_params(uint8_t profile_id);

    // write_params - write new parameters to the gimbal settings
    void write_params();

    bool get_realtimedata( Vector3f& angle);

    // Alexmos Serial Protocol reading part implementation
    // send_command - send a command to the Alemox Serial API
    void send_command(uint8_t cmd, uint8_t* data, uint8_t size);

    // Parse the body of the message received from the Alexmos gimbal
    void parse_body();

    // read_incoming - detect and read the header of the incoming message from the gimbal
    void read_incoming();

    // structure for the Serial Protocol

    // CMD_BOARD_INFO
    struct PACKED alexmos_version {
        uint8_t _board_version;
        uint16_t _firmware_version;
        uint8_t debug_mode;
        uint16_t _board_features;
    };

    // CMD_GET_ANGLES
    struct PACKED alexmos_angles {
        int16_t angle_roll;
        int16_t rc_angle_roll;
        int16_t rc_speed_roll;
        int16_t angle_pitch;
        int16_t rc_angle_pitch;
        int16_t rc_speed_pitch;
        int16_t angle_yaw;
        int16_t rc_angle_yaw;
        int16_t rc_speed_yaw;
    };

    // CMD_CONTROL
    struct PACKED alexmos_angles_speed {
        int8_t mode;
        int16_t speed_roll;
        int16_t angle_roll;
        int16_t speed_pitch;
        int16_t angle_pitch;
        int16_t speed_yaw;
        int16_t angle_yaw;
    };

    // CMD_READ_PARAMS
    struct PACKED alexmos_params {
        uint8_t profile_id;
        uint8_t roll_P;
        uint8_t roll_I;
        uint8_t roll_D;
        uint8_t roll_power;
        uint8_t roll_invert;
        uint8_t roll_poles;
        uint8_t pitch_P;
        uint8_t pitch_I;
        uint8_t pitch_D;
        uint8_t pitch_power;
        uint8_t pitch_invert;
        uint8_t pitch_poles;
        uint8_t yaw_P;
        uint8_t yaw_I;
        uint8_t yaw_D;
        uint8_t yaw_power;
        uint8_t yaw_invert;
        uint8_t yaw_poles;
        uint8_t acc_limiter;
        int8_t ext_fc_gain_roll;
        int8_t ext_fc_gain_pitch;
        int16_t roll_rc_min_angle;
        int16_t roll_rc_max_angle;
        uint8_t roll_rc_mode;
        uint8_t roll_rc_lpf;
        uint8_t roll_rc_speed;
        uint8_t roll_rc_follow;
        int16_t pitch_rc_min_angle;
        int16_t pitch_rc_max_angle;
        uint8_t pitch_rc_mode;
        uint8_t pitch_rc_lpf;
        uint8_t pitch_rc_speed;
        uint8_t pitch_rc_follow;
        int16_t yaw_rc_min_angle;
        int16_t yaw_rc_max_angle;
        uint8_t yaw_rc_mode;
        uint8_t yaw_rc_lpf;
        uint8_t yaw_rc_speed;
        uint8_t yaw_rc_follow;
        uint8_t gyro_trust;
        uint8_t use_model;
        uint8_t pwm_freq;
        uint8_t serial_speed;
        int8_t rc_trim_roll;
        int8_t rc_trim_pitch;
        int8_t rc_trim_yaw;
        uint8_t rc_deadband;
        uint8_t rc_expo_rate;
        uint8_t rc_virt_mode;
        uint8_t rc_map_roll;
        uint8_t rc_map_pitch;	
        uint8_t rc_map_yaw;
        uint8_t rc_map_cmd;
        uint8_t rc_map_fc_roll;
        uint8_t rc_map_fc_pitch;

        uint8_t rc_mix_fc_roll;
        uint8_t rc_mix_fc_pitch;

        uint8_t follow_mode;
        uint8_t follow_deadband;
        uint8_t follow_expo_rate;
        int8_t follow_offset_roll;
        int8_t follow_offset_pitch;
        int8_t follow_offset_yaw;

        int8_t axis_top;
        int8_t axis_right;

        uint8_t gyro_lpf;

        uint8_t gyro_sens;
        uint8_t i2c_internal_pullups;
        uint8_t sky_gyro_calib;

        uint8_t rc_cmd_low;
        uint8_t rc_cmd_mid;
        uint8_t rc_cmd_high;

        uint8_t menu_cmd_1;
        uint8_t menu_cmd_2;
        uint8_t menu_cmd_3;
        uint8_t menu_cmd_4;
        uint8_t menu_cmd_5;
        uint8_t menu_cmd_long;

        uint8_t output_roll;
        uint8_t output_pitch;
        uint8_t output_yaw;

        int16_t bat_threshold_alarm;
        int16_t bat_threshold_motors;
        int16_t bat_comp_ref;

        uint8_t beeper_modes;

        uint8_t follow_roll_mix_start;
        uint8_t follow_roll_mix_range;

        uint8_t booster_power_roll;
        uint8_t booster_power_pitch;
        uint8_t booster_power_yaw;

        uint8_t follow_speed_roll;
        uint8_t follow_speed_pitch;
        uint8_t follow_speed_yaw;

        uint8_t frame_angle_from_motors;

        uint8_t cur_profile_id;

    };
    union PACKED alexmos_parameters {
        DEFINE_BYTE_ARRAY_METHODS
        alexmos_version version;
        alexmos_angles angles;
        alexmos_params params;
        alexmos_angles_speed angle_speed;
    } _buffer,_current_parameters;

    AP_HAL::UARTDriver *_port;
    bool _initialised : 1;

    // result of the get_boardinfo
    uint8_t _board_version;
    float _current_firmware_version;
    uint8_t _firmware_beta_version;
    bool _gimbal_3axis : 1;
    bool _gimbal_bat_monitoring : 1;

    // keep the last _current_angle values
    Vector3f _current_angle;

    // CMD_READ_PARAMS has been called once
    bool _param_read_once : 1;

    // Serial Protocol Variables
    uint8_t _checksum;
    uint8_t _step;
    uint8_t _command_id;
    uint8_t _payload_length;
    uint8_t _payload_counter;

    // confirmed that last command was ok
    bool _last_command_confirmed : 1;

	enum RTS_control {
		RTS_CONTROL_DISABLE = 0, RTS_CONTROL_ENABLE = 1, RTS_CONTROL_AUTO = 2
	};

};
