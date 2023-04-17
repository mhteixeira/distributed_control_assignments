import os
# Change this to the path of your .ino file
ino_file_path = ".\Phase2\Phase2.ino"

# Find all connected Pico boards
pico_ports = ['COM17', 'COM11', 'COM10']
# pico_ports = 'COM11'

# Compile the .ino file and generate a .uf2 file
os.system(f'arduino-cli compile --fqbn rp2040:rp2040:rpipico {ino_file_path} --output-dir ".\Phase2"')
for port in pico_ports:
    os.system(f'arduino-cli upload --fqbn rp2040:rp2040:rpipico {ino_file_path} --input-file ".\Phase2\Phase2.ino.uf2" --port {port}')