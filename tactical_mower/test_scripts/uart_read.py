#!/usr/bin/env python3
import serial

# Pfad zum USB-Gerät anpassen, z. B. /dev/ttyUSB0 oder /dev/ttyACM0
#DEVICE = "/dev/ttyAMA0"
# DEVICE = "/dev/ttyUSB0"
DEVICE = "/dev/ttyTHS1"
# DEVICE = "/dev/ttyACM0"
BAUDRATE = 19200

def main():
    try:
        with serial.Serial(DEVICE, BAUDRATE, timeout=1) as ser:
            print(f"Verbunden mit {DEVICE} @ {BAUDRATE} Baud")
            while True:
                line = ser.readline().decode(errors='ignore').strip()
                if line:
                    print(f"RX: {line}")
    except serial.SerialException as e:
        print(f"Fehler beim Zugriff auf {DEVICE}: {e}")
    except KeyboardInterrupt:
        print("\nBeendet.")

if __name__ == "__main__":
    main()
