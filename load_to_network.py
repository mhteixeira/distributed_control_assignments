import os
import serial.tools.list_ports
import time

# Change this to the path of your .ino file
ino_file_path = "path/to/your/program.ino"

# Find all connected Pico boards
pico_ports = ['COM10', 'COM11', 'COM17']

# Compile the .ino file and generate a .uf2 file
os.system(f"arduino-cli compile --fqbn raspberrypi:pico {ino_file_path}")
uf2_file_path = os.path.splitext(ino_file_path)[0] + ".build/raspberrypi_pico/firmware.uf2"

# Upload the .uf2 file to each Pico board
for port in pico_ports:
    try:
        with serial.Serial(port, baudrate=1200, timeout=1) as ser:
            ser.setDTR(False)
            time.sleep(0.02)
            ser.setDTR(True)
            time.sleep(0.5)
            with open(uf2_file_path, "rb") as f:
                ser.write(f.read())
    except Exception as e:
        print(f"Failed to upload program file to {port}: {e}")
    else:
        print(f"Successfully uploaded program file to {port}")
