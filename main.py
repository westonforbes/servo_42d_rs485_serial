# For RS485 Serial Connections (Not Modbus)
from wf_console import Console
import platform
import serial

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

    def __init__(self, slave_address, com_port: str, baudrate: int, timeout: float = 1.0):
        
        # Ensure we're on Windows platform.
        if self._determine_platform() != "Windows": raise RuntimeError(f"unsupported operating system.")

        # Set slave address of the motor.
        self.slave_address = slave_address

        # Open serial connection.
        #try: self.serial_connection = self._open_serial_connection(port=com_port, baudrate=baudrate, timeout=timeout)
        #except Exception as e: raise e


    def _byte_array_to_int(self, byte_array: list[int]) -> int:
        return int.from_bytes(bytearray(byte_array), byteorder='big', signed=True)
    

    def _int_to_byte_array(self, value: int, length: int) -> list[int]:
        return list(value.to_bytes(length, byteorder='big', signed=True))


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
    

    def _debug_calculate_uplink_packet(self, function_code: int, payload: list[int], verbose: bool = False) -> list[int]:
        
        # Debug output.
        if verbose: Console.fancy_print(f"<FUNCTION>_debug_calculate_uplink_packet()</FUNCTION>")

        # Check function_code type.
        if not isinstance(function_code, int): 
            raise TypeError(f"unsupported function_code type: {type(function_code)}")
        
        # Check payload type.
        if not isinstance(payload, list): 
            raise TypeError(f"unsupported payload type: {type(payload)}")

        # Calculate checksum by summing all individual values
        # Boolean AND with 0xFF to keep the lowest 8 bits.
        checksum = (self.UPLINK_PACKET_HEADLINK_PACKET_HEAD + self.slave_address + function_code + sum(payload)) & 0xFF

        # Final package is the combination of all parts.
        packet = [self.UPLINK_PACKET_HEAD, self.slave_address, function_code] + payload + [checksum]

        # Convert payload to integer.
        payload_int = self._byte_array_to_int(payload)

        # Debug output.
        if verbose:
            Console.fancy_print(f"  <DATA>packet: {[f'0x{b:02X}' for b in packet]}</DATA>")
            Console.fancy_print(f"  <DATA>uplink packet head: 0x{self.UPLINK_PACKET_HEAD:02X}</DATA>")
            Console.fancy_print(f"  <DATA>slave address: 0x{self.slave_address:02X}</DATA>")
            Console.fancy_print(f"  <DATA>function code: 0x{function_code:02X}</DATA>")
            Console.fancy_print(f"  <DATA>payload (bytes): {[f'0x{b:02X}' for b in payload]}</DATA>")
            Console.fancy_print(f"  <DATA>payload (int): {payload_int}</DATA>")
            Console.fancy_print(f"  <DATA>checksum: 0x{checksum:02X}</DATA>")
        
        return packet
    

    def _decode_uplink_packet(self, packet: list[int], verbose: bool = False) -> int:
        
        # Debug output.
        if verbose: Console.fancy_print(f"<FUNCTION>_decode_uplink_packet()</FUNCTION>")
        
        # Check packet length.
        if len(packet) < 4:
            raise ValueError(f"Invalid uplink packet length: {len(packet)}")

        # Extract components.
        packet_head = packet[0]
        slave_address = packet[1]
        function_code = packet[2]
        payload = packet[3:-1]
        checksum = packet[-1]

        # Convert payload to integer.
        payload_int = self._byte_array_to_int(payload)

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
        if packet_head != self.UPLINK_PACKET_HEAD:
            raise ValueError(f"Invalid uplink packet head: 0x{packet_head:02X}")

        # Validate checksum.
        expected_checksum = (packet_head + slave_address + function_code + sum(payload)) & 0xFF
        if checksum != expected_checksum:
            raise ValueError(f"Invalid uplink packet checksum: 0x{checksum:02X} (expected: 0x{expected_checksum:02X})")

        return payload_int


    def _determine_platform(self) -> str:
        system = platform.system()
        if system == "Windows":
            return "Windows"
        elif system == "Linux":
            return "Linux"
        elif system == "Darwin":
            return "macOS"
        else:
            return "Unknown"


    def _open_serial_connection(self, port: str, baudrate: int = 9600, timeout: float = 1.0) -> serial.Serial:
        
        try:
            connection = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
            return connection
        except Exception as e:
            raise RuntimeError(f"could not open serial connection on port '{port}': {e}")


    def _send_and_receive_packet(self, command_packet: list[int], verbose: bool = False) -> list[int]:
        
        # Send command packet.
        self.serial_connection.write(bytearray(command_packet))

        # Read response packet.
        response = self.serial_connection.readline()  # Readline is blocking by timeout length by default.

        # Convert response to list of integers.
        response_packet = list(response)

        # Debug output.
        if verbose:
            Console.fancy_print(f"<DATA>sent command packet: {[f'0x{b:02X}' for b in command_packet]}</DATA>")
            Console.fancy_print(f"<DATA>received response packet: {[f'0x{b:02X}' for b in response_packet]}</DATA>")

        return response_packet


    def get_encoder_position(self, verbose: bool = False) -> list[int]:

        # Construct the command packet.
        command_packet = self._calculate_downlink_packet(
            function_code=self.READ_ENCODER_VALUE_ADDITION,
            payload=[],
            verbose=verbose
        )

        #response_packet = self._send_and_receive_packet(command_packet, verbose=verbose)
        response_packet =[0xFB, 0x01, 0x31, 0x7F, 0xF0, 0x9C]  # Placeholder for received packet.

        # Decode the response packet.
        encoder_position = self._decode_uplink_packet(response_packet, verbose=verbose)

        return encoder_position


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


    def position_motor(self, raw_position: int, speed: int, acc: int, verbose: bool = False) -> bool:
            
        # Construct the command packet.
        command_packet = self._calculate_downlink_packet(
            function_code=0xFE,
            payload=[0x02, 0x58, 0x02, 0x00, 0x00, 0x40, 0x00], # payload=[0x02, 0x58, 0x02, 0xFF, 0xFF, 0xC0, 0x00],
            verbose=verbose
        )

        # Send command and receive response.
        response_packet = self._send_and_receive_packet(command_packet, verbose=verbose)

        # Decode the response packet.
        status = self._decode_uplink_packet(response_packet, verbose=verbose)

        # Returns True if motor disabled successfully.
        return status


if __name__ == "__main__":

    # Clear the console.
    Console.clear()

    # Print header.
    Console.fancy_print("<INFO>\nservo_42d_rs485_serial class test script</INFO>")

    # Initialize servo object, passing in the desired slave address.
    Console.fancy_print("<INFO>\nInitializing Servo42D_RS485_Serial object...</INFO>")
    servo = None
    try: servo = Servo42D_RS485_Serial(slave_address=0x01, com_port="COM8", baudrate=9600, timeout=1.0)
    except Exception as e:
        Console.fancy_print(f"<BAD>Failed to initialize Servo42D_RS485_Serial object: {e}</BAD>")
        exit(1)
    Console.fancy_print("<GOOD>Servo42D_RS485_Serial object initialized.</GOOD>")

    # Define demo function for decoding uplink packets.
    def uplink_packet_demo(packet):
        try:
            position = servo._decode_uplink_packet(packet, verbose=True)
            Console.fancy_print(f"<GOOD>Successfully decoded uplink packet. Encoder position: {position}</GOOD>")
        except ValueError as e:
            Console.fancy_print(f"<BAD>Failed to decode uplink packet: {e}</BAD>")

    example_encoder_position_packet_valid     = [0xFB, 0x01, 0x31, 0x7F, 0xF0, 0x9C]
    example_encoder_position_packet_corrupted = [0xFB, 0x01, 0x31, 0x7F, 0xF0, 0x93]  # Corrupted checksum.

    # Demonstrate decoding a valid uplink packet using package documented in 5.1.2.
    Console.fancy_print("<INFO>\nExample of valid uplink packet decoding (private method, not typically for user use):</INFO>")
    uplink_packet_demo(example_encoder_position_packet_valid)

    # Demonstrate decoding a corrupted uplink packet.
    Console.fancy_print("<INFO>\nExample of corrupted uplink packet decoding (private method, not typically for user use):</INFO>")
    uplink_packet_demo(example_encoder_position_packet_corrupted)

    # Demonstrate creating a downlink packet.
    Console.fancy_print("<INFO>\nExample of downlink packet creation (private method, not typically for user use):</INFO>")
    downlink_packet = servo._calculate_downlink_packet(function_code=Servo42D_RS485_Serial.READ_ENCODER_VALUE_ADDITION,payload=[],verbose=True)
    Console.fancy_print(f"<GOOD>Successfully created downlink packet: {[f'0x{b:02X}' for b in downlink_packet]}</GOOD>")

    Console.fancy_print("<INFO>\nExample of getting encoder position.</INFO>")
    encoder_position = servo.get_encoder_position(verbose=True)
    Console.fancy_print(f"<GOOD>Successfully retrieved encoder position: {encoder_position}</GOOD>")

    Console.fancy_print("<INFO>\nTest 1.</INFO>")
    motor_enabled = servo.enable_motor(verbose=True)
    servo.position_motor(raw_position=10000, speed=600, acc=400, verbose=True)


