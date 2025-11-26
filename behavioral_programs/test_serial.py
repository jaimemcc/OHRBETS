import serial
port = 'COM3'  # Replace with your port (e.g., COM3, /dev/ttyUSB0, etc.)
baud_rate = 115200  # Baud rate (match your Arduino's configuration)
try:
    # Attempt to open the serial port
    ser = serial.Serial(port, baud_rate)
    print(f"Successfully opened {port}. You have the necessary permissions.")
    ser.close()  # Close the port after testing
except serial.SerialException as e:
    print(f"Failed to open {port}. Error: {e}")