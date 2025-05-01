import serial
import struct
import zlib
import time
import os

# Constants
HEADER = b'\x30\x20\x30\x20\x30\x20\x30\x20'  # 64-bit - 0x3020302030203020
#HEADER = b'\xAD\xAD\xAD\xAD\xAD\xAD\xAD\xAD'
NOT_HEADER = b'\x31\x21\x31\x21\x31\x21\x31\x20'
SERIAL_PORT = '/dev/serial0'           
BAUD_RATE = 115200
TIMEOUT_SEC = 0.5
MIN_MSG_SIZE = 9
MAX_MSG_SIZE = 500

try:
    ser = serial.Serial('/dev/serial0', BAUD_RATE, timeout=TIMEOUT_SEC)
    ser.flushInput()
    ser.flushOutput()
    print('serial established')
except:
    print('error opening serial port')

# Message Types
MESSAGE_TYPE_RESEND_LAST = 0
MESSAGE_TYPE_SEND_NEXT = 1  # Default usage
MESSAGE_TYPE_REQUEST_SPECIFIC = 2
MESSAGE_TYPE_UPDATE_PARAMS = 3
MESSAGE_TYPE_SHUTDOWN = 4
MESSAGE_TYPE_REBOOT = 5
MESSAGE_TYPE_BAD_MESSAGE = 6

def compute_crc32(data: bytes) -> int:
    return zlib.crc32(data)

def build_packet(message_type: int, argument: bytes = b"") -> bytes:
    """
    Builds a send packet in the format of [HEADER][SIZE][TYPE][ARGUMENT][CHECKSUM]
    """
    type_byte = struct.pack('<B', message_type)

    argument_bytes = argument

    payload = type_byte + argument_bytes
    size = len(payload) + 8  # TYPE + ARG + CRC + size
    size_bytes = struct.pack('<I', size)
    packet_without_checksum = size_bytes + payload

    checksum = compute_crc32(packet_without_checksum)
    checksum_bytes = struct.pack('<I', checksum)
    
    if message_type == MESSAGE_TYPE_BAD_MESSAGE:
        return NOT_HEADER + packet_without_checksum + checksum_bytes
    else:
        return HEADER + packet_without_checksum + checksum_bytes

def parse_response_packet(packet: bytes):
    """
    Parse response packet and validate structure and checksum.
    Return the message type and payload.
    """
    if len(packet) < 17:  #packet overhead = 17 bytes
        raise ValueError("Incomplete packet received")

    if packet[:8] != HEADER:
        raise ValueError("Invalid header")

    size = struct.unpack('<I', packet[8:12])[0]
    message_type = struct.unpack('<B', packet[12:13])[0]
    payload = packet[13:-4]
    received_checksum = struct.unpack('<I', packet[-4:])[0]
    computed_checksum = compute_crc32(packet[8:-4])

    if received_checksum != computed_checksum:
        raise ValueError("Checksum mismatch")

    return message_type, payload

def send_request(ser: serial.Serial, message_type: int, argument: bytes = b""):
    #ser.reset_input_buffer() # clear previous data in the buffer
    packet = build_packet(message_type, argument)
    ser.write(packet)
    #ser.flush()
    print(f"[INFO] Sent request: type={message_type}")
    print(packet)
    

def handle_buffer(ser, buffer, waiting_bytes):
    if buffer is None:
        buffer = ser.read(waiting_bytes)
    else:
        buffer = buffer + ser.read(waiting_bytes)
    print(len(buffer))
    while len(buffer) >= 15:
        if buffer[0:8] != HEADER:
            buffer = buffer[1:]
        else:
            print('valid header')
            size = struct.unpack('<I', buffer[8:12])[0]
            print(size)
            if size < MIN_MSG_SIZE or size > MAX_MSG_SIZE:
                print('bad msg size')
                print(buffer)
                buffer = buffer[1:]
                continue
            if len(buffer) < size+8:
                #incomplete message
                return buffer
            else:
                #complete message
                full_packet = buffer[0:size+8]
                buffer = buffer[size+8:]
                print("complete packet rx'd")
                print(full_packet)
                try:
                    group_id, payload = struct.unpack('<B', full_packet[12:13])[0], full_packet[13:-4]
#                     if group_id == 1 2 or 3:
#                         #print prased contents 
                    index = struct.unpack('<I', payload[:4])[0]
                    print(f"[RECEIVED] Group: {group_id}, Index: {index}, Payload Size: {len(payload)} bytes")
                    folder = f"received_logs/group{group_id}"
                    os.makedirs(folder, exist_ok=True)
                    filename = f"{group_id}_{index}.bin"
                    with open(os.path.join(folder, filename), 'wb') as f:
                        f.write(full_packet)
                except Exception as e:
                    print(f"[ERROR] Failed to parse response: {e}")
    return buffer

def read_response(ser: serial.Serial):
    waiting_bytes = ser.in_waiting
    
    if waiting_bytes > 500:
        print("overloaded buffer")
        print(waiting_bytes)
    
    if waiting_bytes >= 17:
        print("checking header...")
        header = ser.read_until(expected = HEADER, size=300)
        if header[-8:] == HEADER:
            print("valid header")
            waiting_bytes = ser.in_waiting
            if waiting_bytes <= 4:
                print("incomplete packet rx'd")
            else:
                size_B = ser.read(4)
                size = struct.unpack('<I', size_B)[0]
                if waiting_bytes < size or size > 999:
                    print("incomplete packet rx''d")
                    print(size)
                else:
                    rest = ser.read(size-4)
                    packet = header[-8:]+size_B+rest
                    print("complete packet rx'd")
                    print(packet)
                    try:
                        group_id, payload = struct.unpack('<B', packet[12:13])[0], packet[13:-4]
                        index = struct.unpack('<I', payload[:4])[0]
                        print(f"[RECEIVED] Group: {group_id}, Index: {index}, Payload Size: {len(payload)} bytes")

                        folder = f"received_logs/group{group_id}"
                        os.makedirs(folder, exist_ok=True)
                        filename = f"{group_id}_{index}.bin"
                        with open(os.path.join(folder, filename), 'wb') as f:
                            f.write(packet)
                    except Exception as e:
                        print(f"[ERROR] Failed to parse response: {e}")

#         header_and_size = ser.read(12)
#         if header_and_size[:8] != HEADER:
#             print("[ERROR] Invalid header")
#             flush = ser.read(waiting_bytes-12)
#             return
#         size = struct.unpack('<I', header_and_size[8:12])[0]
#         remaining = size + 5  # TYPE + PAYLOAD + CRC
#         rest = ser.read(remaining)
#         packet = header_and_size + rest
#         left_over = waiting_butes - 12 - remaining
#         if left_over > 0:
#             flush = ser.read(waiting_bytes)
#    else:
#        print("[INFO] No response received.")

DT_RX_S = 0.01
DT_TX_S = 5
DT_LISTEN_S = 5
def main():
    #print(f"[INFO] Connecting to serial port {SERIAL_PORT} at {BAUD_RATE} baud")
    buffer = None
    try:
        time_iter_s = time.monotonic()
    #with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT_SEC) as ser:
    #listen for 5 s, then take user input
        while True:
            time_iter_s = time.monotonic()
            while time.monotonic() < (time_iter_s + DT_LISTEN_S):
                waiting_bytes = ser.in_waiting
                if waiting_bytes > 0:
                    if buffer is None:
                        buffer = ser.read(waiting_bytes)
                    else:
                        #buffer = buffer + ser.read(waiting_bytes)
                        buffer = handle_buffer(ser, buffer, waiting_bytes)
            print('buffer is')
            print(buffer)
            user_input = input("Enter 0=resend, 1=next, 2=specify, 3=update param, 4=shutdown, 5=reboot (Enter to continue): ").strip()
            if user_input == "":
                pass  # continue waiting

            elif user_input == "0":
                send_request(ser, MESSAGE_TYPE_RESEND_LAST)
                #time.sleep(0.1)
                #read_response(ser)

            elif user_input == "1":
                try:
                    total_seconds = float(input("Enter time to keep sending (seconds): ").strip())
                    start_time = time.monotonic()
                    time_prev_rx_s = time_iter_s
                    time_prev_tx_s = time_iter_s
                    while time_iter_s - start_time < total_seconds:
                        time_iter_s = time.monotonic()
                        waiting_bytes = ser.in_waiting
                        if time_iter_s > time_prev_rx_s + DT_RX_S and waiting_bytes > 0:
                            #read_response(ser)
                            if buffer is None:
                                buffer = ser.read(ser.in_waiting)
                            else:
                                buffer = handle_buffer(ser, buffer, waiting_bytes)
                            time_prev_rx_s = time_iter_s
                        if time_iter_s > time_prev_tx_s + DT_TX_S:
                            send_request(ser, MESSAGE_TYPE_SEND_NEXT)
                            time_prev_tx_s = time_iter_s
                        
                        #time.sleep(5)  # wait 5 seconds between each next request
                except Exception as e:
                    print(f"[ERROR] Invalid input: {e}")

            elif user_input.startswith("2"):
                try:
                    arg = input("Enter group_index (e.g., 2_5): ").strip()
                    argument_bytes = arg.encode('utf-8')
                    send_request(ser, MESSAGE_TYPE_REQUEST_SPECIFIC, argument=argument_bytes)
                    #time.sleep(0.1)
                    #read_response(ser)
                except Exception as e:
                    print(f"[ERROR] Invalid argument: {e}")

            elif user_input.startswith("3"):
                try:
                    param_index = int(input("Enter parameter index (integer): ").strip())
                    param_value = float(input("Enter new parameter value (float): ").strip())
                    param_index_bytes = struct.pack('<B', param_index)
                    param_value_bytes = struct.pack('<f', param_value)
                    argument_bytes = param_index_bytes + param_value_bytes
                    send_request(ser, MESSAGE_TYPE_UPDATE_PARAMS, argument=argument_bytes)
                    #time.sleep(0.1)
                    #read_response(ser)
                except Exception as e:
                    print(f"[ERROR] Invalid parameter update input: {e}")

            elif user_input == "4":
                send_request(ser, MESSAGE_TYPE_SHUTDOWN)
                #time.sleep(0.1)
                #read_response(ser)

            elif user_input == "5":
                send_request(ser, MESSAGE_TYPE_REBOOT)
                #time.sleep(0.1)
                #read_response(ser)
            
            elif user_input == "6":
                send_request(ser, MESSAGE_TYPE_BAD_MESSAGE)
                #time.sleep(0.1)
                #read_response(ser)

            else:
                print("[WARN] Invalid command.")

    except serial.SerialException as e:
        print(f"[ERROR] Serial communication error: {e}")
    except KeyboardInterrupt:
        print("\n[INFO] Script interrupted. Exiting.")

if __name__ == '__main__':
    main()
