# Python Script: control_lights.py
import serial
import time

# Importing action modules
import light_painting 

# Replace with your Arduino's actual port (e.g., check in Arduino IDE)
port = '/dev/cu.usbmodem101'  
baud_rate = 9600  # Must match the Arduino's Serial.begin() value

try:
    # Open serial connection
    ser = serial.Serial(port, baud_rate)
    print("Connected to Arduino")
    
    # Wait 2 seconds for Arduino to reset after connection
    time.sleep(2)

    #print(bytes(light_painting.call_test(), "utf-8"))
    
    #ser.flush()
    user_input = input()
    ser.write(light_painting.call_test().encode())
    #ser.write(bytes("hello penis", "utf-8"))
    #ser.write(b'r')
    #ser.flush()
    
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
    
    time.sleep(1)

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