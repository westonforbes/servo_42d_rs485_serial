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
        try: self.serial_connection = self._open_serial_connection(port=com_port, baudrate=baudrate, timeout=timeout)
        except Exception as e: raise e


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
            #payload=[0x02, 0x58, 0x02, 0x00, 0x00, 0x40, 0x00], 
            payload=[0x02, 0x58, 0x02, 0xFF, 0xFF, 0xC0, 0x00],
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
    Console.fancy_print("<INFO>\ninitializing Servo42D_RS485_Serial object...</INFO>")
    servo = None
    try: servo = Servo42D_RS485_Serial(slave_address=0x01, com_port="COM3", baudrate=38400, timeout=1.0)
    except Exception as e:
        Console.fancy_print(f"<BAD>failed to initialize Servo42D_RS485_Serial object: {e}</BAD>")
        exit(1)
    Console.fancy_print("<GOOD>Servo42D_RS485_Serial object initialized.</GOOD>")
    Console.press_enter_pause()

    Console.clear()
    Console.fancy_print("<INFO>\nfactory reset. manual section 5.6, pg 30.</INFO>")
    result = servo.send_command(function_code=0x3F, payload=[],verbose= True)
    Console.fancy_print(f"<INFO>return code: </INFO><DATA>{result}</DATA>")
    Console.press_enter_pause()

    Console.clear()
    Console.fancy_print("<INFO>\n restart command. manual section 5.7, pg 30.</INFO>")
    result = servo.send_command(function_code=0x41, payload=[],verbose= False)
    Console.fancy_print(f"<INFO>return code: </INFO><DATA>{result}</DATA>")
    Console.press_enter_pause()

    Console.clear()
    Console.fancy_print("<INFO>\nget encoder position.</INFO>")
    encoder_position = servo.get_encoder_position(verbose=False)
    Console.fancy_print(f"<GOOD>Successfully retrieved encoder position: {encoder_position}</GOOD>")
    Console.press_enter_pause()

    Console.clear()
    Console.fancy_print("<INFO>\nset work mode command. manual section 5.2.2, pg 18.</INFO>")
    result = servo.send_command(function_code=0x82, payload=[0x04],verbose= True) # Set the enable pin mode to always active.
    Console.fancy_print(f"<INFO>return code: </INFO><DATA>{result}</DATA>")
    Console.press_enter_pause()
    
    Console.clear()
    Console.fancy_print("<INFO>\nset enable pin mode.</INFO>")
    result = servo.send_command(function_code=0x85, payload=[0x02],verbose= True) # Set the enable pin mode to always active.
    Console.fancy_print(f"<INFO>return code: </INFO><DATA>{result}</DATA>")
    Console.press_enter_pause()

    Console.fancy_print("<INFO>\nenable motor.</INFO>")
    motor_enabled = servo.enable_motor(verbose=True)

    Console.fancy_print("<INFO>\nmove motor.</INFO>")
    servo.position_motor(raw_position=10000, speed=600, acc=400, verbose=True)


