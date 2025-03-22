# Python Script: control_lights.py
import serial
import time

# Importing action modules
import light_painting 

# Replace with your Arduino's actual port (e.g., check in Arduino IDE)
port = '/dev/cu.usbmodem1101'  
baud_rate = 9600  # Must match the Arduino's Serial.begin() value
buffer_size = 1498

def fillBuffers(data):
    commands = data.split('/')
    for command in commands:
        pass
        #print(command)
        #print("\n\n\n\n\n")
    
    current_buffer = 0
    del commands[0]
    del commands[len(commands) - 1]
    buffers = [""]
    
    for command in commands:
        command_with_delim = command + '/'
        if len(buffers[current_buffer]) == 0:
            buffers[current_buffer] += '/'
        if len(buffers[current_buffer]) + len(command_with_delim) < buffer_size:
            buffers[current_buffer] += command_with_delim
        else:
            
            buffers[current_buffer] += 'Q' + '\x00'
            print(buffers[current_buffer])
            #print('\n\n\n')
            buffers.append("")
            current_buffer = current_buffer + 1
    
    buffers[current_buffer] += 'Q' + '\x00'
    print(buffers[current_buffer])
    return buffers


try:
    
    ser = serial.Serial(port, baud_rate)
    print("Connected to Arduino")

    # Wait 2 seconds for Arduino to reset after connection
    time.sleep(2)

    #print(bytes(light_painting.call_test(), "utf-8"))
    data = light_painting.call_test()
    print(data)
    print("\n\n\n")
    buffers = fillBuffers(data)
    
    current_buffer = 0
    #user_input = input()
    #ser.write(buffers[0].encode())
    #ser.flush()
    #ser.write(buffers[1].encode())
    #print(buffers[0])
    #print('\n\n\n')
    #print(buffers[1])

    
    #user_input = input()

    
    ret_val = ser.readline().decode('utf-8').strip()
    print(ret_val + '\n')
    while (ret_val != 'Q'):
        if (ret_val == 'A'):
            ser.write(buffers[current_buffer].encode())
            if (current_buffer < len(buffers) - 1):
                current_buffer += 1
            else:
                break
        print(ret_val + '\n')
        ret_val = ser.readline().decode('utf-8').strip()
    
    
            

    """
    #ser.flush()
    user_input = input()
    to_send = 'aaaaaaaaaaaaaaaaa' + 'Q' + '\x00'
    ser.write(to_send.encode())

    to_send2 = 'bbbbbbbbbbbbbbbbb' + 'Q' + '\x00'
    ser.write(to_send2.encode())

    to_send3 = 'ccccccccccccccc' + 'Q' + '\x00'
    ser.write(to_send3.encode())
    """
     
    #ret_val = ser.read().decode('utf-8').strip()
    #while (ret_val == 0):
    #print(ret_val)
    #    ret_val = ser.readline().decode('utf-8').strip()
    #if (str(ret_val) == 'p'):
    #    print("yay")
    #ser.write(light_painting.call_test().encode())
    #print(len(light_painting.call_test().encode()))
    #ser.write(bytes("/S*H/M*1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,13.0,14.0/Q", "utf-8"))
    #ser.write(bytes("hello penis", "utf-8"))
    #ser.write(b'r')
    #ser.flush()
    #time.sleep(1)
    #ser.write(b"Q")
    
    """
    # Main loop for user input
    while True:
        user_input = input("Enter 'g' to turn on light 1, 'r' to turn on light 2: ")
        if user_input == 'g':
            ser.write(b'g')  # Send 'g' as bytes
        elif user_input == 'r':
            ser.write(b'r')  # Send 'r' as bytes
        else:
            print("Invalid input, please type 'g' or 'r'")
    """
    
    
    #time.sleep(1)

except serial.SerialException as e:
    print(f"Serial error: {e}")
    print("Check the port name, ensure the Arduino is connected, and no other program is using the port.")
except Exception as e:
    print(f"Error: {e}")
finally:
    # Close the serial connection if it was opened
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial connection closed")


"""
def split_for_buffers(data: str, buffer_size: int = 1000, delimiter: str = '/'):
    commands = data.split(delimiter)
    buffers = ["", ""]
    current_buffer = 0
    
    for command in commands:
        command_with_delimiter = command + delimiter
        if len(buffers[current_buffer]) + len(command_with_delimiter) <= buffer_size:
            buffers[current_buffer] += command_with_delimiter
        else:
            current_buffer = 1  # Switch to second buffer
            if len(buffers[current_buffer]) + len(command_with_delimiter) <= buffer_size:
                buffers[current_buffer] += command_with_delimiter
            else:
                raise ValueError("Command too large for buffer size")
    
    return tuple(buffers)
"""


# Example usage:
"""
large_string = "CMD1\nCMD2\nCMD3\n...CMDN"
buffer1, buffer2 = split_for_buffers(large_string)
print(f"Buffer 1: {buffer1}")
print(f"Buffer 2: {buffer2}")
"""