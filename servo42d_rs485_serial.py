# For RS485 Serial Connections (Not Modbus)
from wf_console import Console
import platform
import serial
from datetime import datetime, timezone

class Servo42D_RS485_Serial:
    # Declare constants.
    DOWNLINK_PACKET_HEAD = 0xFA
    UPLINK_PACKET_HEAD = 0xFB
    READ_ENCODER_VALUE_ADDITION = 0x31
    ENABLE_MOTOR = 0xF3

    # Declare class parameters.
    slave_address = 0x01
    encoder_postion = 0
    serial_connection = None
    steps_per_revolution = 200
    status = {}
    configuration = {}

    def __init__(self, slave_address, com_port: str, baudrate: int, timeout: float = 1.0, steps_per_revolution: int = 200):
        
        # Ensure we're on Windows platform.
        if self._determine_platform() != "Windows": raise RuntimeError(f"unsupported operating system.")

        # Set slave address of the motor.
        self.slave_address = slave_address

        # Set steps per revolution.
        self.steps_per_revolution = steps_per_revolution

        # Open serial connection.
        try: self.serial_connection = self._open_serial_connection(port=com_port, baudrate=baudrate, timeout=timeout)
        except Exception as e: raise e

    @staticmethod
    def _byte_array_to_int(byte_array: list[int]) -> int:
        return int.from_bytes(bytearray(byte_array), byteorder='big', signed=True)
    
    @staticmethod
    def _int_to_byte_array(value: int, length: int) -> list[int]:
        return list(value.to_bytes(length, byteorder='big', signed=True))

    @staticmethod
    def _decode_uplink_packet(packet: list[int], verbose: bool = False) -> int:
        
        # Debug output.
        if verbose: Console.fancy_print(f"<FUNCTION>_decode_uplink_packet()</FUNCTION>")

        # Extract components.
        packet_head = packet[0]
        slave_address = packet[1]
        function_code = packet[2]
        payload = packet[3:-1]
        checksum = packet[-1]

        # Convert payload to integer.
        payload_int = Servo42D_RS485_Serial._byte_array_to_int(payload)

        # Debug output.
        if verbose:
            Console.fancy_print(f"  <DATA>packet: {[f'0x{b:02X}' for b in packet]}</DATA>")
            Console.fancy_print(f"  <DATA>uplink packet head: 0x{packet_head:02X}</DATA>")
            Console.fancy_print(f"  <DATA>slave address: 0x{slave_address:02X}</DATA>")
            Console.fancy_print(f"  <DATA>function code: 0x{function_code:02X}</DATA>")
            Console.fancy_print(f"  <DATA>payload (bytes): {[f'0x{b:02X}' for b in payload]}</DATA>")
            Console.fancy_print(f"  <DATA>payload (int): {payload_int}</DATA>")
            Console.fancy_print(f"  <DATA>checksum: 0x{checksum:02X}</DATA>")

        # Validate packet head.
        if packet_head != Servo42D_RS485_Serial.UPLINK_PACKET_HEAD:
            raise ValueError(f"Invalid uplink packet head: 0x{packet_head:02X}")

        # Validate checksum.
        expected_checksum = (packet_head + slave_address + function_code + sum(payload)) & 0xFF
        if checksum != expected_checksum:
            raise ValueError(f"Invalid uplink packet checksum: 0x{checksum:02X} (expected: 0x{expected_checksum:02X})")

        return payload_int

    @staticmethod
    def _determine_platform() -> str:
        system = platform.system()
        if system == "Windows":
            return "Windows"
        elif system == "Linux":
            return "Linux"
        elif system == "Darwin":
            return "macOS"
        else:
            return "Unknown"

    @staticmethod
    def _open_serial_connection(port: str, baudrate: int = 9600, timeout: float = 1.0) -> serial.Serial:
        
        try:
            connection = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            return connection
        except Exception as e:
            raise RuntimeError(f"could not open serial connection on port '{port}': {e}")

    @staticmethod
    def _parse_int16(b1, b2):
        """Helper to parse a signed 16-bit integer from two bytes (big-endian)."""
        val = (b1 << 8) | b2
        return val - (1 << 16) if val & (1 << 15) else val

    @staticmethod
    def _parse_int32(b1, b2, b3, b4):
        """Helper to parse a signed 32-bit integer from four bytes (big-endian)."""
        val = (b1 << 24) | (b2 << 16) | (b3 << 8) | b4
        return val - (1 << 32) if val & (1 << 31) else val

    @staticmethod
    def _parse_int48(b1, b2, b3, b4, b5, b6):
        """Helper to parse a signed 48-bit integer from six bytes (big-endian)."""
        val = (b1 << 40) | (b2 << 32) | (b3 << 24) | (b4 << 16) | (b5 << 8) | b6
        return val - (1 << 48) if val & (1 << 47) else val
   
   
    def _calculate_downlink_packet(self, function_code: int, payload: list[int], verbose: bool = False) -> list[int]:
        
        # Debug output.
        if verbose: Console.fancy_print(f"<FUNCTION>_calculate_downlink_packet()</FUNCTION>")

        # Check function_code type.
        if not isinstance(function_code, int): 
            raise TypeError(f"unsupported function_code type: {type(function_code)}")
        
        # Check payload type.
        if not isinstance(payload, list): 
            raise TypeError(f"unsupported payload type: {type(payload)}")

        # Calculate checksum by summing all individual values
        # Boolean AND with 0xFF to keep the lowest 8 bits.
        checksum = (self.DOWNLINK_PACKET_HEAD + self.slave_address + function_code + sum(payload)) & 0xFF

        # Final package is the combination of all parts.
        packet = [self.DOWNLINK_PACKET_HEAD, self.slave_address, function_code] + payload + [checksum]

        # Convert payload to integer.
        payload_int = self._byte_array_to_int(payload)

        # Debug output.
        if verbose:
            Console.fancy_print(f"  <DATA>packet: {[f'0x{b:02X}' for b in packet]}</DATA>")
            Console.fancy_print(f"  <DATA>downlink packet head: 0x{self.DOWNLINK_PACKET_HEAD:02X}</DATA>")
            Console.fancy_print(f"  <DATA>slave address: 0x{self.slave_address:02X}</DATA>")
            Console.fancy_print(f"  <DATA>function code: 0x{function_code:02X}</DATA>")
            Console.fancy_print(f"  <DATA>payload (bytes): {[f'0x{b:02X}' for b in payload]}</DATA>")
            Console.fancy_print(f"  <DATA>payload (int): {payload_int}</DATA>")
            Console.fancy_print(f"  <DATA>checksum: 0x{checksum:02X}</DATA>")
        
        return packet
    

    def _send_and_receive_packet(self, command_packet: list[int], verbose: bool = False) -> list[int]:
        
        self.serial_connection.reset_input_buffer()

        # Send command packet.
        self.serial_connection.write(bytearray(command_packet))

        # Read response packet.
        response = self.serial_connection.readline()  # Readline is blocking by timeout length by default.

        # Convert response to list of integers.
        response_packet = list(response)

        # Debug output.
        if verbose:
            Console.fancy_print(f"  <DATA>sent command packet: {[f'0x{b:02X}' for b in command_packet]}</DATA>")
            Console.fancy_print(f"  <DATA>received response packet: {[f'0x{b:02X}' for b in response_packet]}</DATA>")

        return response_packet


    def get_encoder_position(self, verbose: bool = False) -> list[int]:

        # Construct the command packet.
        command_packet = self._calculate_downlink_packet(
            function_code=self.READ_ENCODER_VALUE_ADDITION,
            payload=[],
            verbose=verbose
        )

        response_packet = self._send_and_receive_packet(command_packet, verbose=verbose)

        # Decode the response packet.
        encoder_position = self._decode_uplink_packet(response_packet, verbose=verbose)

        return encoder_position


    def send_command(self, function_code: int, payload: list[int], verbose: bool = False):

        # Construct the command packet.
        command_packet = self._calculate_downlink_packet(
            function_code=function_code,
            payload=payload,
            verbose=verbose
        )

        # Send command and receive response.
        response_packet = self._send_and_receive_packet(command_packet, verbose=verbose)

        # Decode the response packet.
        status = self._decode_uplink_packet(response_packet, verbose=verbose)

        return status
    

    def enable_motor(self, verbose: bool = False) -> bool:

        # Construct the command packet.
        command_packet = self._calculate_downlink_packet(
            function_code=self.ENABLE_MOTOR,
            payload=[0x01],
            verbose=verbose
        )

        # Send command and receive response.
        response_packet = self._send_and_receive_packet(command_packet, verbose=verbose)

        # Decode the response packet.
        status = self._decode_uplink_packet(response_packet, verbose=verbose)

        # Returns True if motor enabled successfully.
        return status
    

    def calibrate_encoder(self, verbose: bool = False) -> bool:

        # Construct the command packet.
        command_packet = self._calculate_downlink_packet(
            function_code=0x80,
            payload=[],
            verbose=verbose
        )

        # Send command and receive response.
        response_packet = self._send_and_receive_packet(command_packet, verbose=verbose)

        # Decode the response packet.
        status = self._decode_uplink_packet(response_packet, verbose=verbose)

        # Returns True if calibration successful.
        return status


    def zero_encoder(self, verbose: bool = False) -> bool:

        # Construct the command packet.
        command_packet = self._calculate_downlink_packet(
            function_code=0x92,
            payload=[],
            verbose=verbose
        )

        # Send command and receive response.
        response_packet = self._send_and_receive_packet(command_packet, verbose=verbose)

        # Decode the response packet.
        status = self._decode_uplink_packet(response_packet, verbose=verbose)

        # Returns True if zeroing successful.
        return status


    def disable_motor(self, verbose: bool = False) -> bool:

        # Construct the command packet.
        command_packet = self._calculate_downlink_packet(
            function_code=self.ENABLE_MOTOR,
            payload=[0x00],
            verbose=verbose
        )

        # Send command and receive response.
        response_packet = self._send_and_receive_packet(command_packet, verbose=verbose)

        # Decode the response packet.
        status = self._decode_uplink_packet(response_packet, verbose=verbose)

        # Returns True if motor disabled successfully.
        return status


    def read_all_configuration_parameters(self, verbose: bool = False) -> bool:

        # Construct the command packet.
        command_packet = [self.DOWNLINK_PACKET_HEAD,  self.slave_address, 0x47, 0x42]
        
        # Send command and receive response.
        response_packet = self._send_and_receive_packet(command_packet, verbose=verbose)

        # Payload general information creation.
        payload = {}

        payload['timestamp'] = {}
        payload['timestamp']['utc'] = datetime.now(timezone.utc).isoformat()
        payload['timestamp']['local'] = datetime.now().isoformat()

        payload['documentation'] = {}
        payload['documentation']['manual_version'] = '1.0.6'
        payload['documentation']['section'] = '5.9.2'
        payload['documentation']['page'] = 32

        # Decode mode.
        payload['mode'] = {}
        payload['mode']['documentation'] = {}
        payload['mode']['documentation']['manual_version'] = '1.0.6'
        payload['mode']['documentation']['section'] = '5.2.2'
        payload['mode']['documentation']['page'] = 18
        payload['mode']['bytes'] = response_packet[3]
        match payload['mode']['bytes']:
            case 0x00: payload['mode']['value'] = 'CR_OPEN'
            case 0x01: payload['mode']['value'] = 'CR_CLOSE'
            case 0x02: payload['mode']['value'] = 'CRvFOC'
            case 0x03: payload['mode']['value'] = 'SR_OPEN'
            case 0x04: payload['mode']['value'] = 'SR_CLOSE'
            case 0x05: payload['mode']['value'] = 'SR_FOC'
            case    _: payload['mode']['value'] = 'unknown'

        # Decode work current.
        payload['work_current'] = {}
        payload['work_current']['documentation'] = {}
        payload['work_current']['documentation']['manual_version'] = '1.0.6'
        payload['work_current']['documentation']['section'] = '5.2.3'
        payload['work_current']['documentation']['page'] = 19
        payload['work_current']['bytes'] = response_packet[4] + response_packet[5]
        payload['work_current']['value'] = payload['work_current']['bytes'] * 10  # Scale factor of 10mA.
        payload['work_current']['units'] = 'mA'

        # Decode hold current.
        payload['hold_current_percentage'] = {}
        payload['hold_current_percentage']['documentation'] = {}
        payload['hold_current_percentage']['documentation']['manual_version'] = '1.0.6'
        payload['hold_current_percentage']['documentation']['section'] = '5.2.4'
        payload['hold_current_percentage']['documentation']['page'] = 19
        payload['hold_current_percentage']['bytes'] = response_packet[6]
        payload['hold_current_percentage']['value'] = (payload['hold_current_percentage']['bytes'] + 1) * 10
        payload['hold_current_percentage']['units'] = '%'

        # Decode microsteps.
        payload['microsteps'] = {}
        payload['microsteps']['documentation'] = {}
        payload['microsteps']['documentation']['manual_version'] = '1.0.6'
        payload['microsteps']['documentation']['section'] = '5.2.5'
        payload['microsteps']['documentation']['page'] = 19
        payload['microsteps']['bytes'] = response_packet[7]
        payload['microsteps']['value'] = payload['microsteps']['bytes']
        payload['microsteps']['units'] = 'microsteps per step'
        payload['microsteps']['microsteps_per_revolution'] = self.steps_per_revolution * payload['microsteps']['value']
        payload['microsteps']['degrees_per_microstep'] = 360.0 / payload['microsteps']['microsteps_per_revolution']

        # Decode the enable pin configuration.
        payload['en_pin'] = {}
        payload['en_pin']['documentation'] = {}
        payload['en_pin']['documentation']['manual_version'] = '1.0.6'
        payload['en_pin']['documentation']['section'] = '5.2.6'
        payload['en_pin']['documentation']['page'] = 20
        payload['en_pin']['bytes'] = response_packet[8]
        match payload['en_pin']['bytes']:
            case 0x00: payload['en_pin']['value'] = 'active low'
            case 0x01: payload['en_pin']['value'] = 'active high'
            case 0x02: payload['en_pin']['value'] = 'pin disabled (always active)'
            case    _: payload['en_pin']['value'] = 'unknown'

        # Decode the direction.
        payload['direction'] = {}
        payload['direction']['documentation'] = {}
        payload['direction']['documentation']['manual_version'] = '1.0.6'
        payload['direction']['documentation']['section'] = '5.2.7'
        payload['direction']['documentation']['page'] = 20
        payload['direction']['bytes'] = response_packet[9]
        match payload['direction']['bytes']:
            case 0x00: payload['direction']['value'] = 'clockwise'
            case 0x01: payload['direction']['value'] = 'counter-clockwise'
            case    _: payload['direction']['value'] = 'unknown'

        # Decode screen timeout function.
        payload['auto_screen_off'] = {}
        payload['auto_screen_off']['documentation'] = {}
        payload['auto_screen_off']['documentation']['manual_version'] = '1.0.6'
        payload['auto_screen_off']['documentation']['section'] = '5.2.8'
        payload['auto_screen_off']['documentation']['page'] = 20
        payload['auto_screen_off']['bytes'] = response_packet[10]
        match payload['auto_screen_off']['bytes']:
            case 0x00: payload['auto_screen_off']['value'] = 'disabled'
            case 0x01: payload['auto_screen_off']['value'] = 'enabled'
            case    _: payload['auto_screen_off']['value'] = 'unknown'
        
        # Decode stall protection.
        payload['stall_protection'] = {}
        payload['stall_protection']['documentation'] = {}
        payload['stall_protection']['documentation']['manual_version'] = '1.0.6'
        payload['stall_protection']['documentation']['section'] = '5.2.9'
        payload['stall_protection']['documentation']['page'] = 21
        payload['stall_protection']['bytes'] = response_packet[11]
        match payload['stall_protection']['bytes']:
            case 0x00: payload['stall_protection']['value'] = 'disabled'
            case 0x01: payload['stall_protection']['value'] = 'enabled'
            case    _: payload['stall_protection']['value'] = 'unknown'
        
        # Decode subdivision interpolation.
        payload['interpolation'] = {}
        payload['interpolation']['documentation'] = {}
        payload['interpolation']['documentation']['manual_version'] = '1.0.6'
        payload['interpolation']['documentation']['section'] = '5.2.10'
        payload['interpolation']['documentation']['page'] = 21
        payload['interpolation']['bytes'] = response_packet[12]
        match payload['interpolation']['bytes']:
            case 0x00: payload['interpolation']['value'] = 'disabled'
            case 0x01: payload['interpolation']['value'] = 'enabled'
            case    _: payload['interpolation']['value'] = 'unknown'

        # Decode baud rate.
        payload['baud_rate'] = {}
        payload['baud_rate']['documentation'] = {}
        payload['baud_rate']['documentation']['manual_version'] = '1.0.6'
        payload['baud_rate']['documentation']['section'] = '5.2.11'
        payload['baud_rate']['documentation']['page'] = 22
        payload['baud_rate']['bytes'] = response_packet[13]
        match payload['baud_rate']['bytes']:
            case 0x01: payload['baud_rate']['value'] = 9600
            case 0x02: payload['baud_rate']['value'] = 19200
            case 0x03: payload['baud_rate']['value'] = 25000
            case 0x04: payload['baud_rate']['value'] = 38400
            case 0x05: payload['baud_rate']['value'] = 57600
            case 0x06: payload['baud_rate']['value'] = 115200
            case 0x07: payload['baud_rate']['value'] = 256000
            case    _: payload['baud_rate']['value'] = 'unknown'
        
        # Decode slave address.
        payload['slave_address'] = {}
        payload['slave_address']['documentation'] = {}
        payload['slave_address']['documentation']['manual_version'] = '1.0.6'
        payload['slave_address']['documentation']['section'] = '5.2.12'
        payload['slave_address']['documentation']['page'] = 22
        payload['slave_address']['bytes'] = response_packet[14]
        payload['slave_address']['value'] = payload['slave_address']['bytes']

        # Decode group address.
        payload['group_address'] = {}
        payload['group_address']['documentation'] = {}
        payload['group_address']['documentation']['manual_version'] = '1.0.6'
        payload['group_address']['documentation']['section'] = '5.2.16'
        payload['group_address']['documentation']['page'] = 24
        payload['group_address']['bytes'] = response_packet[15]
        payload['group_address']['value'] = payload['group_address']['bytes']

        # Decode slave respond.
        payload['slave_respond'] = {}
        payload['slave_respond']['documentation'] = {}
        payload['slave_respond']['documentation']['manual_version'] = '1.0.6'
        payload['slave_respond']['documentation']['section'] = '5.2.13'
        payload['slave_respond']['documentation']['page'] = 23
        payload['slave_respond']['respond_bytes'] = response_packet[16]
        payload['slave_respond']['active_bytes'] = response_packet[17]
        match payload['slave_respond']['respond_bytes']:
            case 0x00: payload['slave_respond']['respond'] = 'disabled'
            case 0x01: payload['slave_respond']['respond'] = 'enabled'
            case    _: payload['slave_respond']['respond'] = 'unknown'
        match payload['slave_respond']['active_bytes']:
            case 0x00: payload['slave_respond']['active'] = 'disabled'
            case 0x01: payload['slave_respond']['active'] = 'enabled'
            case    _: payload['slave_respond']['active'] = 'unknown'
            
        # Decode MODBUS-RTU.
        payload['modbus_rtu'] = {}
        payload['modbus_rtu']['documentation'] = {}
        payload['modbus_rtu']['documentation']['manual_version'] = '1.0.6'
        payload['modbus_rtu']['documentation']['section'] = '5.2.14'
        payload['modbus_rtu']['documentation']['page'] = 23
        payload['modbus_rtu']['bytes'] = response_packet[18]
        match payload['modbus_rtu']['bytes']:
            case 0x00: payload['modbus_rtu']['value'] = 'disabled'
            case 0x01: payload['modbus_rtu']['value'] = 'enabled'
            case    _: payload['modbus_rtu']['value'] = 'unknown'

        # Decode key lock.
        payload['key_lock'] = {}
        payload['key_lock']['documentation'] = {}
        payload['key_lock']['documentation']['manual_version'] = '1.0.6'
        payload['key_lock']['documentation']['section'] = '5.2.15'
        payload['key_lock']['documentation']['page'] = 24
        payload['key_lock']['bytes'] = response_packet[19]
        match payload['key_lock']['bytes']:
            case 0x00: payload['key_lock']['value'] = 'unlocked'
            case 0x01: payload['key_lock']['value'] = 'locked'
            case    _: payload['key_lock']['value'] = 'unknown'

        # Decode homing parameters.
        payload['homing'] = {}
        payload['homing']['documentation'] = {}
        payload['homing']['documentation']['manual_version'] = '1.0.6'
        payload['homing']['documentation']['section'] = '5.4.1'
        payload['homing']['documentation']['page'] = 25

        # Decode home trigger level.
        payload['homing']['trigger_level_bytes'] = response_packet[20]
        match payload['homing']['trigger_level_bytes']:
            case 0x00: payload['homing']['trigger_level'] = 'low'
            case 0x01: payload['homing']['trigger_level'] = 'high'
            case    _: payload['homing']['trigger_level'] = 'unknown'

        # Decode home direction.
        payload['homing']['direction_bytes'] = response_packet[21]
        match payload['homing']['direction_bytes']:
            case 0x00: payload['homing']['direction'] = 'clockwise'
            case 0x01: payload['homing']['direction'] = 'counter-clockwise'
            case    _: payload['homing']['direction'] = 'unknown'

        # Decode home speed.
        payload['homing']['speed_bytes'] = (response_packet[22] << 8) | response_packet[23]
        payload['homing']['speed_value'] = payload['homing']['speed_bytes']
        payload['homing']['speed_units'] = 'rpm'

        # Decode endstop limit.
        payload['homing']['endstop_limit_bytes'] = response_packet[24]
        match payload['homing']['endstop_limit_bytes']:
            case 0x00: payload['homing']['endstop_limit'] = 'disabled'
            case 0x01: payload['homing']['endstop_limit'] = 'enabled'
            case    _: payload['homing']['endstop_limit'] = 'unknown'

        # Decode no-limit homing parameters.
        payload['no_limit_homing'] = {}
        payload['no_limit_homing']['documentation'] = {}
        payload['no_limit_homing']['documentation']['manual_version'] = '1.0.6'
        payload['no_limit_homing']['documentation']['section'] = '5.4.4'
        payload['no_limit_homing']['documentation']['page'] = 27

        # Decode reverse angle value.
        payload['no_limit_homing']['reverse_angle_bytes'] = (response_packet[25] << 24) | \
                                    (response_packet[26] << 16) | \
                                    (response_packet[27] << 8)  | \
                                    response_packet[28]
        payload['no_limit_homing']['reverse_angle_value'] = payload['no_limit_homing']['reverse_angle_bytes']
        payload['no_limit_homing']['reverse_angle_note'] = "e.g., 0x2000 = 180 degrees"

        # Decode home mode.
        payload['no_limit_homing']['mode_bytes'] = response_packet[29]
        match payload['no_limit_homing']['mode_bytes']:
            case 0x00: payload['no_limit_homing']['mode'] = 'use Limit switch'
            case 0x01: payload['no_limit_homing']['mode'] = 'no Limit switch'
            case _: payload['no_limit_homing']['mode'] = 'unknown'
        
        # Decode home current.
        payload['no_limit_homing']['current_bytes'] = (response_packet[30] << 8) | response_packet[31]
        payload['no_limit_homing']['current_value'] = payload['no_limit_homing']['current_bytes']
        payload['no_limit_homing']['current_units'] = 'mA'

        # Decode limit port remap.
        payload['limit_port_remap'] = {}
        payload['limit_port_remap']['documentation'] = {}
        payload['limit_port_remap']['documentation']['manual_version'] = '1.0.6'
        payload['limit_port_remap']['documentation']['section'] = '5.4.5'
        payload['limit_port_remap']['documentation']['page'] = 28
        payload['limit_port_remap']['bytes'] = response_packet[32]
        match payload['limit_port_remap']['bytes']:
            case 0x00: payload['limit_port_remap']['value'] = 'disabled'
            case 0x01: payload['limit_port_remap']['value'] = 'enabled'
            case _: payload['limit_port_remap']['value'] = 'unknown'

        # Decode power-on go-to-zero parameters.
        payload['power_on_zero'] = {}
        payload['power_on_zero']['documentation'] = {}
        payload['power_on_zero']['documentation']['manual_version'] = '1.0.6'
        payload['power_on_zero']['documentation']['section'] = '5.5.1'
        payload['power_on_zero']['documentation']['page'] = 29

        # Decode power-on zero mode.
        payload['power_on_zero']['mode_bytes'] = response_packet[33]
        match payload['power_on_zero']['mode_bytes']:
            case 0x00: payload['power_on_zero']['mode'] = 'disabled'
            case 0x01: payload['power_on_zero']['mode'] = 'direction mode'
            case 0x02: payload['power_on_zero']['mode'] = 'near mode'
            case _: payload['power_on_zero']['mode'] = 'unknown'
            
        # Decode reserved set 0.
        payload['power_on_zero']['reserved_set_0_bytes'] = response_packet[34]

        # Decode power-on zero speed.
        payload['power_on_zero']['speed_bytes'] = response_packet[35]
        payload['power_on_zero']['speed_value'] = f"{payload['power_on_zero']['speed_bytes']} (0-4, 0=slowest)"
        
        # Decode power-on zero direction.
        payload['power_on_zero']['direction_bytes'] = response_packet[36]
        match payload['power_on_zero']['direction_bytes']:
            case 0x00: payload['power_on_zero']['direction'] = 'clockwise'
            case 0x01: payload['power_on_zero']['direction'] = 'counter-clockwise'
            case _: payload['power_on_zero']['direction'] = 'unknown'

        if verbose:
            import json
            Console.fancy_print(f"<DATA>{json.dumps(payload, indent=4)}</DATA>")

        self.configuration = payload
    

    def read_all_status_parameters(self, verbose: bool = False) -> bool:

        # Construct the command packet.
        command_packet = [self.DOWNLINK_PACKET_HEAD,  self.slave_address, 0x48, 0x43]
        
        # Send command and receive response.
        response_packet = self._send_and_receive_packet(command_packet, verbose=verbose)

        # Validate the response packet
        # A valid response has 31 bytes:
        # 1 (Head) + 1 (Addr) + 1 (Func) + 27 (Data) + 1 (CRC)
        if not response_packet or len(response_packet) < 31 or response_packet[2] != 0x48:
            if verbose:
                print(f"Error: Invalid or short response for 0x48 command. Got: {response_packet}")
            return False

        # Payload general information creation.
        payload = {}

        payload['timestamp'] = {}
        payload['timestamp']['utc'] = datetime.now(timezone.utc).isoformat()
        payload['timestamp']['local'] = datetime.now().isoformat()

        payload['documentation'] = {}
        payload['documentation']['command'] = '0x48'
        payload['documentation']['manual_version'] = '1.0.6'
        payload['documentation']['section'] = '5.9.3'
        payload['documentation']['page'] = 33

        # --- Start of Payload Decoding ---
        # The data payload starts at response_packet[3] (Byte 4 in the manual table)

        # Decode Motor Status (Byte 4)
        payload['motor_status'] = {}
        payload['motor_status']['documentation'] = {'section': '6.2.1', 'page': 38}
        payload['motor_status']['bytes'] = response_packet[3]
        match payload['motor_status']['bytes']:
            case 0x01: payload['motor_status']['value'] = 'motor stop'
            case 0x02: payload['motor_status']['value'] = 'motor speed up'
            case 0x03: payload['motor_status']['value'] = 'motor speed down'
            case 0x04: payload['motor_status']['value'] = 'motor full speed'
            case 0x05: payload['motor_status']['value'] = 'motor is homing'
            case 0x06: payload['motor_status']['value'] = 'motor is Cal...' # From page 59
            case _:    payload['motor_status']['value'] = 'Unknown'

        # Decode Encoder Value (Bytes 5-10)
        payload['encoder_value'] = {}
        payload['encoder_value']['documentation'] = {'section': '5.1.2', 'page': 15}
        payload['encoder_value']['value'] = self._parse_int48(response_packet[4], response_packet[5], response_packet[6], response_packet[7], response_packet[8], response_packet[9])
        
        # Decode Speed (Bytes 11-12)
        payload['speed'] = {}
        payload['speed']['documentation'] = {'section': '5.1.3', 'page': 16}
        payload['speed']['value'] = self._parse_int16(response_packet[10], response_packet[11])
        payload['speed']['units'] = 'RPM'

        # Decode Pulses (Bytes 13-16)
        payload['pulses'] = {}
        payload['pulses']['documentation'] = {'section': '5.1.4', 'page': 16}
        payload['pulses']['value'] = self._parse_int32(response_packet[12], response_packet[13], response_packet[14], response_packet[15])
        
        # Decode IO Status (Byte 17)
        payload['io_status'] = {}
        payload['io_status']['documentation'] = {'section': '5.1.5', 'page': 16}
        payload['io_status']['bytes'] = response_packet[16]
        payload['io_status']['value'] = {
            'IN_1': bool(response_packet[16] & 0x01),
            'IN_2': bool(response_packet[16] & 0x02),
            'OUT_1': bool(response_packet[16] & 0x04),
            'OUT_2': bool(response_packet[16] & 0x08),
        }

        # Decode RAW Encoder Value (Bytes 18-23)
        payload['raw_encoder_value'] = {}
        payload['raw_encoder_value']['documentation'] = {'section': '5.1.6', 'page': 16}
        payload['raw_encoder_value']['value'] = self._parse_int48(response_packet[17], response_packet[18], response_packet[19], response_packet[20], response_packet[21], response_packet[22])
        
        # Decode Angle Error (Bytes 24-27)
        payload['angle_error'] = {}
        payload['angle_error']['documentation'] = {'section': '5.1.7', 'page': 17}
        payload['angle_error']['value'] = self._parse_int32(response_packet[23], response_packet[24], response_packet[25], response_packet[26])
        payload['angle_error']['note'] = "0-51200 corresponds to 0-360 degrees"

        # Decode En Status (Byte 28)
        payload['en_status'] = {}
        payload['en_status']['documentation'] = {'section': '5.1.8', 'page': 17}
        payload['en_status']['bytes'] = response_packet[27]
        match payload['en_status']['bytes']:
            case 0x00: payload['en_status']['value'] = 'Disabled'
            case 0x01: payload['en_status']['value'] = 'Enabled'
            case _:    payload['en_status']['value'] = 'Unknown'

        # Decode Go Back to Zero Status (0_status) (Byte 29)
        payload['zero_status'] = {}
        payload['zero_status']['documentation'] = {'section': '5.1.9', 'page': 17}
        payload['zero_status']['bytes'] = response_packet[28]
        match payload['zero_status']['bytes']:
            case 0x00: payload['zero_status']['value'] = 'going to zero'
            case 0x01: payload['zero_status']['value'] = 'go back to zero success'
            case 0x02: payload['zero_status']['value'] = 'go back to zero fail'
            case _:    payload['zero_status']['value'] = 'Unknown'

        # Decode Protection Status (Protect status) (Byte 30)
        payload['protection_status'] = {}
        payload['protection_status']['documentation'] = {'section': '5.1.11', 'page': 17}
        payload['protection_status']['bytes'] = response_packet[29]
        match payload['protection_status']['bytes']:
            case 0x00: payload['protection_status']['value'] = 'no protected'
            case 0x01: payload['protection_status']['value'] = 'protected'
            case _:    payload['protection_status']['value'] = 'Unknown'

        # Store the payload as a class attribute
        self.status = payload
