from wf_console import Console
from servo42d_rs485_serial import Servo42D_RS485_Serial

if __name__ == "__main__":
    
    servo = None
    try: servo = Servo42D_RS485_Serial(slave_address=0x01, com_port="COM4", baudrate=38400, timeout=1.0)
    except Exception as e:
        Console.fancy_print(f"<BAD>failed to initialize Servo42D_RS485_Serial object: {e}</BAD>")
        exit(1)

    servo.read_all_configuration_parameters(verbose= False)
    servo.read_all_status_parameters(verbose= False)
    exit(0)

#--------------------------------------------------------------

    # Clear the console.
    Console.clear()

    # Print header.
    Console.fancy_print("<INFO>\nservo_42d_rs485_serial class test script</INFO>")

    # Initialize servo object, passing in the desired slave address.
    Console.fancy_print("<INFO>\ninitializing Servo42D_RS485_Serial object...</INFO>")
    servo = None
    try: servo = Servo42D_RS485_Serial(slave_address=0x01, com_port="COM4", baudrate=38400, timeout=1.0)
    except Exception as e:
        Console.fancy_print(f"<BAD>failed to initialize Servo42D_RS485_Serial object: {e}</BAD>")
        exit(1)
    Console.fancy_print("<GOOD>Servo42D_RS485_Serial object initialized.</GOOD>")
    Console.press_enter_pause()

    # Create menu items.
    menu_items = ['factory reset',
                  'restart command',
                  'read all configuration parameters',
                  'set work mode',
                  'set enable pin mode',
                  'enable motor',
                  'disable motor',
                  'calibrate encoder',
                  'zero encoder',
                  'get encoder position',
                  'position mode - absolute move',
                  'speed mode - start motor',
                  'speed mode - stop motor',
                  'exit script']

    while True:

        # Display menu.
        int_selection, str_selection = Console.integer_only_menu_with_validation(title = 'main menu', item_list = menu_items, input_message = 'select an option: ')
        
        # Clear the console once a selection is made.
        Console.clear()

        # Select what to do based on menu selection.
        if str_selection == 'factory reset':
            Console.fancy_print("<INFO>\nfactory reset. manual section 5.6, pg 30.</INFO>")
            Console.fancy_print("<DATA>expected response: 0x01 for success, 0x00 for failure.</DATA>")
            result = servo.send_command(function_code=0x3F, payload=[],verbose= True)
            Console.fancy_print(f"<INFO>return code: </INFO><DATA>{result}</DATA>")
            Console.press_enter_pause()

        elif str_selection == 'restart command':
            Console.fancy_print("<INFO>\n restart command. manual section 5.7, pg 30.</INFO>")
            Console.fancy_print("<DATA>expected response: 0x01 for success, 0x00 for failure.</DATA>")
            result = servo.send_command(function_code=0x41, payload=[],verbose= True)
            Console.fancy_print(f"<INFO>return code: </INFO><DATA>{result}</DATA>")
            Console.press_enter_pause()

        elif str_selection == 'read all configuration parameters':
            Console.fancy_print("<INFO>\nread all configuration parameters. manual section 5.9.2, pg 32.</INFO>")
            Console.fancy_print("<DATA>expected response: 0x01 for success, 0x00 for failure.</DATA>")
            result = servo.read_all_configuration_parameters(verbose= True)
            Console.fancy_print(f"<INFO>return code: </INFO><DATA>{result}</DATA>")
            Console.press_enter_pause()

        elif str_selection == 'set work mode':
            Console.fancy_print("<INFO>\nset work mode command. manual section 5.2.2, pg 18.</INFO>")
            result = servo.send_command(function_code=0x82, payload=[0x04],verbose= True) # Set the work mode to SR_CLOSE (serial closed loop).
            #result = servo.send_command(function_code=0x82, payload=[0x05],verbose= True) # Set the work mode to SR_FOC (serial field-oriented control).
            Console.fancy_print(f"<INFO>return code: </INFO><DATA>{result}</DATA>")
            Console.press_enter_pause()

        elif str_selection == 'set enable pin mode':
            Console.fancy_print("<INFO>\nset enable pin mode.manual section 5.2.6, pg 20.</INFO>")
            result = servo.send_command(function_code=0x85, payload=[0x02],verbose= True) # Set the enable pin mode to always active.
            Console.fancy_print(f"<INFO>return code: </INFO><DATA>{result}</DATA>")
            Console.press_enter_pause()

        elif str_selection == 'enable motor':
            Console.fancy_print("<INFO>\nenable motor. manual section 6.2.2, pg 38.</INFO>")
            motor_enabled = servo.enable_motor(verbose=True)
            Console.press_enter_pause()

        elif str_selection == 'disable motor':
            Console.fancy_print("<INFO>\ndisable motor. manual section 6.2.2, pg 38.</INFO>")
            motor_disabled = servo.disable_motor(verbose=True)
            Console.press_enter_pause()

        elif str_selection == 'calibrate encoder':
            Console.fancy_print("<INFO>\ncalibrate encoder. manual section 5.2.1, pg 18.</INFO>")
            Console.fancy_print("<DATA>expected response: 0x00 for in-process, 0x01 for success, 0x02 for failure.</DATA>")
            calibration_successful = servo.calibrate_encoder(verbose=True)
            Console.press_enter_pause()

        elif str_selection == 'zero encoder':
            Console.fancy_print("<INFO>\nzero encoder. manual section 5.2.1, pg 18.</INFO>")
            Console.fancy_print("<DATA>expected response: 0x00 for in-process, 0x01 for success, 0x02 for failure.</DATA>")
            zeroing_successful = servo.zero_encoder(verbose=True)
            Console.press_enter_pause()

        elif str_selection == 'get encoder position':
            Console.fancy_print("<INFO>\nget encoder position. manual section 5.1.2, pg 15.</INFO>")
            encoder_position = servo.get_encoder_position(verbose=False)
            Console.fancy_print(f"<GOOD>Successfully retrieved encoder position: {encoder_position}</GOOD>")
            Console.press_enter_pause()

        elif str_selection == 'position mode - absolute move':
            Console.fancy_print("<INFO>\nposition mode - absolute move</INFO>")
            result = servo.absolute_move(position=6000, speed=500, acceleration=100, verbose= True)
            Console.fancy_print(f"<INFO>return code: </INFO><DATA>{result}</DATA>")
            Console.press_enter_pause()

        elif str_selection == 'speed mode - start motor':
            Console.fancy_print("<INFO>\nspeed mode - start motor</INFO>")
            result = servo.send_command(function_code=0xF6, payload=[0x01, 0x40, 0x02],verbose= True)
            Console.fancy_print(f"<INFO>return code: </INFO><DATA>{result}</DATA>")
            Console.press_enter_pause()

        elif str_selection == 'speed mode - stop motor':
            Console.fancy_print("<INFO>\nspeed mode - stop motor</INFO>")
            result = servo.send_command(function_code=0xF6, payload=[0x00, 0x00, 0x02],verbose= True)
            Console.fancy_print(f"<INFO>return code: </INFO><DATA>{result}</DATA>")
            Console.press_enter_pause()

        elif str_selection == 'exit script':
            Console.clear()
            exit(0)