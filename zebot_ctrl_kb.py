import serial
import keyboard
import time
#import recipe_getch

print("Zebot Control Keyboard");
print("Use 'w' 's' 'a' 'd' to control");

ser = serial.Serial('/dev/ttyACM0', 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)

while True:
    key = keyboard.read_key()
  #  print(f"You pressed {key}")
    ser.write(bytes(key, "ascii"))
  #  time.sleep(0.01)
  #  ser.write(bytes('p', "ascii"))


