#include "AP_Mount_Alexmos.h"
#include <stdint.h>
#include "stm32.h"
#include "board_config.h"

extern const AP_HAL::HAL& hal;

void AP_Mount_Alexmos::init(const AP_SerialManager& serial_manager)
{
	// check for alexmos protcol
	if ((_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_AlexMos, 0))) {
		_initialised = true;
		//get_boardinfo();
		//read_params(0); //we request parameters for profile 0 and therfore get global and profile parameters
		
	}
}



// update mount position - should be called periodically
void AP_Mount_Alexmos::update()
{
	if (!_initialised) {
		return;
	}

	//stm32_gpiowrite(GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN11, false);

	stm32_gpiowrite(GPIO_OUTPUT | GPIO_OPENDRAIN | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTD | GPIO_PIN12,true);

		//_port->set_flow_control();
		// update targets using pilot's rc inputs
		
	//GPIO_SetBits(GPIOD, GPIO_Pin_11);

		update_targets_from_rc();

		uint8_t ret;
		new_transmission(ClassCode::GIMBAL, GIMBAL_SET_MODE, GIMBAL_MODE, Bytes::from<uint8_t>((uint8_t)Gimbal_Mode::JOYSTICK_SPEED_CONTROL), Bytes::from<uint8_t>(ret));
		Rate gimbalRateResp, gimbalRate = Rate(1500, 1500);

		HorizontalFieldOfView field = getHorizontalFieldOfView();

		Rate result;
		Vector3f target_deg = _angle_ef_target_rad;

		target_deg *= RAD_TO_DEG;

		result.elevation = DEGREE_TO_VALUE(target_deg.x);;
		result.azimuth = DEGREE_TO_VALUE(target_deg.y);;


		gimbalRate = result;
		reverseByteOrderInPlace(gimbalRate.azimuth);
		reverseByteOrderInPlace(gimbalRate.elevation); // set big-endian order before sending

		new_transmission(ClassCode::GIMBAL, GIMBAL_SET_RATE, GIMBAL_RATE, Bytes::from(gimbalRate), Bytes::from(gimbalRateResp));


		//_port->write(0x7E);
	//hal.gpio->write(58, 0);
	//hal.gpio->write(PD10, 0);
	//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, 1);
	//HAL_GPIO_Write1();
	//ioctl(_gpio_fd, GPIO_SET_OUTPUT, 1)

	//_port->write(0x7E);
	//_port->write(0x7E);
	//_port->write(0x7E);
	//_port->write(target_deg.x);
	//_port->write(0x7D);
	//_port->write(0x7D);
	//_port->write(0x7D);
	//_port->write(target_deg.y);
	//_port->write(0x7C);
	//_port->write(0x7C);
	//_port->write(0x7C);
	//hal.gpio->pinMode(59, 1);
	//hal.gpio->write(59, 1);
	//hal.gpio->pinMode(58, 1);
	//hal.gpio->write(58, 1);
	//PX4GPIO::write(HAL_GPIO_A_LED_PIN, 1);
	
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_Alexmos::has_pan_control() const
{
	return _gimbal_3axis;
}

// set_mode - sets mount's mode
void AP_Mount_Alexmos::set_mode(enum MAV_MOUNT_MODE mode)
{
	// record the mode change and return success
	_state._mode = mode;
}

// status_msg - called to allow mounts to send their status to GCS using the MOUNT_STATUS message  STATUS TO SERIAL FROM GIMBAL
void AP_Mount_Alexmos::status_msg(mavlink_channel_t chan)
{
	if (!_initialised) {
		return;
	}

	//get_angles();
	//mavlink_msg_mount_status_send(chan, 0, 0, _current_angle.y * 100, _current_angle.x * 100, _current_angle.z * 100);


}

/*
* get_angles
*/
void AP_Mount_Alexmos::get_angles()
{
	uint8_t data[1] = { (uint8_t)1 };
	send_command(CMD_GET_ANGLES, data, 1);
}

/*
* set_motor will activate motors if true, and disable them if false.
*/
void AP_Mount_Alexmos::set_motor(bool on)
{
	if (on) {
		uint8_t data[1] = { (uint8_t)1 };
		send_command(CMD_MOTORS_ON, data, 1);
	}
	else {
		uint8_t data[1] = { (uint8_t)1 };
		send_command(CMD_MOTORS_OFF, data, 1);
	}
}

/*
* get board version and firmware version
*/
void AP_Mount_Alexmos::get_boardinfo()
{
	if (_board_version != 0) {
		return;
	}
	uint8_t data[1] = { (uint8_t)1 };
	send_command(CMD_BOARD_INFO, data, 1);
}

/*
control_axis : send new angles to the gimbal at a fixed speed of 30 deg/s2
*/
void AP_Mount_Alexmos::control_axis(const Vector3f& angle, bool target_in_degrees)
{
	// convert to degrees if necessary
	Vector3f target_deg = angle;
	if (!target_in_degrees) {
		target_deg *= RAD_TO_DEG;
	}
	alexmos_parameters outgoing_buffer;
	outgoing_buffer.angle_speed.mode = AP_MOUNT_ALEXMOS_MODE_ANGLE;
	outgoing_buffer.angle_speed.speed_roll = DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_ALEXMOS_SPEED);
	outgoing_buffer.angle_speed.angle_roll = DEGREE_TO_VALUE(target_deg.x);
	outgoing_buffer.angle_speed.speed_pitch = DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_ALEXMOS_SPEED);
	outgoing_buffer.angle_speed.angle_pitch = DEGREE_TO_VALUE(target_deg.y);
	outgoing_buffer.angle_speed.speed_yaw = DEGREE_PER_SEC_TO_VALUE(AP_MOUNT_ALEXMOS_SPEED);
	outgoing_buffer.angle_speed.angle_yaw = DEGREE_TO_VALUE(target_deg.z);
	send_command(CMD_CONTROL, (uint8_t *)&outgoing_buffer.angle_speed, sizeof(alexmos_angles_speed));
}

/*
read current profile profile_id and global parameters from the gimbal settings
*/
void AP_Mount_Alexmos::read_params(uint8_t profile_id)
{
	uint8_t data[1] = { (uint8_t)profile_id };
	send_command(CMD_READ_PARAMS, data, 1);
}

/*
write new parameters to the gimbal settings
*/
void AP_Mount_Alexmos::write_params()
{
	if (!_param_read_once) {
		return;
	}
	send_command(CMD_WRITE_PARAMS, (uint8_t *)&_current_parameters.params, sizeof(alexmos_params));
}

/*
send a command to the Alemox Serial API
*/
void AP_Mount_Alexmos::send_command(uint8_t cmd, uint8_t* data, uint8_t size)
{
	if (_port->txspace() < (size + 5U)) {
		return;
	}
	uint8_t checksum = 0;
	_port->write('>');
	_port->write(cmd);  // write command id
	_port->write(size);  // write body size
	_port->write(cmd + size); // write header checkum

	for (uint8_t i = 0; i != size; i++) {
		checksum += data[i];
		_port->write(data[i]);
	}
	_port->write(checksum);
}

/*
* Parse the body of the message received from the Alexmos gimbal
*/
void AP_Mount_Alexmos::parse_body()
{
	switch (_command_id) {
	case CMD_BOARD_INFO:
		_board_version = _buffer.version._board_version / 10;
		_current_firmware_version = _buffer.version._firmware_version / 1000.0f;
		_firmware_beta_version = _buffer.version._firmware_version % 10;
		_gimbal_3axis = (_buffer.version._board_features & 0x1);
		_gimbal_bat_monitoring = (_buffer.version._board_features & 0x2);
		break;

	case CMD_GET_ANGLES:
		_current_angle.x = VALUE_TO_DEGREE(_buffer.angles.angle_roll);
		_current_angle.y = VALUE_TO_DEGREE(_buffer.angles.angle_pitch);
		_current_angle.z = VALUE_TO_DEGREE(_buffer.angles.angle_yaw);
		break;

	case CMD_READ_PARAMS:
		_param_read_once = true;
		_current_parameters.params = _buffer.params;
		break;

	case CMD_WRITE_PARAMS:
		break;

	default:
		_last_command_confirmed = true;
		break;
	}
}

/*
* detect and read the header of the incoming message from the gimbal
*/
void AP_Mount_Alexmos::read_incoming()
{
	uint8_t data;
	int16_t numc;

	numc = _port->available();

	if (numc < 0) {
		return;
	}

	for (int16_t i = 0; i < numc; i++) {        // Process bytes received
		data = _port->read();
		switch (_step) {
		case 0:
			if ('>' == data) {
				_step = 1;
				_checksum = 0; //reset checksum accumulator
				_last_command_confirmed = false;
			}
			break;

		case 1: // command ID
			_checksum = data;
			_command_id = data;
			_step++;
			break;

		case 2: // Size of the body of the message
			_checksum += data;
			_payload_length = data;
			_step++;
			break;

		case 3:	// checksum of the header
			if (_checksum != data) {
				_step = 0;
				_checksum = 0;
				// checksum error
				break;
			}
			_step++;
			_checksum = 0;
			_payload_counter = 0;                               // prepare to receive payload
			break;

		case 4: // parsing body
			_checksum += data;
			if (_payload_counter < sizeof(_buffer)) {
				_buffer[_payload_counter] = data;
			}
			if (++_payload_counter == _payload_length)
				_step++;
			break;

		case 5:// body checksum
			_step = 0;
			if (_checksum != data) {
				break;
			}
			parse_body();
		}
	}
}

void AP_Mount_Alexmos::stuffAndTransmitByte(uint8_t b) const
{
	switch (b)
	{
		case ESCAPE_BYTE:
		{
			//Serial.write(ESCAPE_BYTE);
			//Serial.write(ESCAPE_BYTE ^ 32);
			_port->write(ESCAPE_BYTE);
			_port->write(ESCAPE_BYTE ^ 32);
			break;
		}
		case FLAG_BYTE:
		{
			//Serial.write(ESCAPE_BYTE);
			//Serial.write(FLAG_BYTE ^ 32);
			_port->write(ESCAPE_BYTE);
			_port->write(FLAG_BYTE ^ 32);
			break;
		}
		default:
		{
			//Serial.write(b);
			_port->write(b);
			break;
		}
	}
}

void AP_Mount_Alexmos::transmitMsg()
{
	if (m_buf == m_end) return;
	_port->write(FLAG_BYTE);
	for (uint8_t *p = m_buf; p < m_end; ++p) stuffAndTransmitByte(*p);
	_port->write(FLAG_BYTE);
	_port->flush();
	//printf(" \n");
}