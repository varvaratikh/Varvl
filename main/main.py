import serial
import time

serial_port = '/dev/cu.usbmodem1101'
baud_rate = 115200

def wait_for_response(serial_device):
    while True:
        if serial_device.in_waiting > 0:
            raw_response = serial_device.readline()
            try:
                # Попробуем декодировать в UTF-8, заменяя некорректные символы
                response = raw_response.decode('utf-8', errors='replace').strip()
                print(f"Decoded response: {response}")
                return response
            except Exception as e:
                print(f"Error decoding response: {e}")
                print(f"Raw response: {raw_response}")
                return None

# Основная функция обработки G-code
def process_gcode(file_path):
    try:
        with serial.Serial(serial_port, baud_rate, timeout=1) as serial_device:
            print(f"Connecting to Arduino on {serial_port} at {baud_rate} baud...")
            time.sleep(4)  # Ждем инициализации серийного соединения
            print("Connection established.")

            with open(file_path, 'r') as file:
                for line in file:
                    line = line.strip()

                    # Обработка команд G0 и G1
                    if line.startswith('G0') or line.startswith('G1'):
                        x_value = None
                        y_value = None
                        parts = line.split()

                        for part in parts:
                            if part.startswith('X'):
                                x_value = float(part[1:])
                            elif part.startswith('Y'):
                                y_value = float(part[1:])

                        if x_value is not None and y_value is not None:
                            # Формируем команду для отправки на Arduino
                            command = f"{'U' if line.startswith('G0') else 'D'} {x_value} {y_value}\n"
                            serial_device.write(command.encode('utf-8'))
                            print(f"Sent to Arduino: {command.strip()}")

                            response = wait_for_response(serial_device)
                            if response:
                                print(f"Response from Arduino: {response}")
                            else:
                                print("No response or decoding error.")

    except serial.SerialException as e:
        print(f"Serial connection error: {e}")
    except FileNotFoundError as e:
        print(f"File not found: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

# Путь к файлу с G-code
gcode_file = 'image-2.gcode'
# Запуск обработки
process_gcode(gcode_file)
