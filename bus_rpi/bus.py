import serial
import struct
import zlib
import time
import os

# Constants
HEADER = b'\x30\x20\x30\x20\x30\x20\x30\x20'  # 64-bit - 0x3020302030203020
SERIAL_PORT = '/dev/serial0'           
BAUD_RATE = 115200
TIMEOUT_SEC = 1

# Message Types
MESSAGE_TYPE_RESEND_LAST = 0
MESSAGE_TYPE_SEND_NEXT = 1  # Default usage
MESSAGE_TYPE_REQUEST_SPECIFIC = 2
MESSAGE_TYPE_UPDATE_PARAMS = 3
MESSAGE_TYPE_SHUTDOWN = 4
MESSAGE_TYPE_REBOOT = 5

def compute_crc32(data: bytes) -> int:
    return zlib.crc32(data)

def build_packet(message_type: int, argument: str = "") -> bytes:
    """
    Builds a send packet in the format of [HEADER][SIZE][TYPE][ARGUMENT][CHECKSUM]
    """
    type_byte = struct.pack('<B', message_type)

    if argument:
        argument_bytes = argument.encode('utf-8')  # e.g., b"1_5"
    else:
        argument_bytes = b""

    payload = type_byte + argument_bytes
    size = len(payload) + 8 # TYPE + ARG + CRC + size
    size_bytes = struct.pack('<I', size)
    packet_without_checksum = size_bytes + payload

    checksum = compute_crc32(packet_without_checksum)
    checksum_bytes = struct.pack('<I', checksum)

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

def send_request(ser: serial.Serial, message_type: int = MESSAGE_TYPE_SEND_NEXT, argument: str = ""):
    packet = build_packet(message_type, argument)
    ser.write(packet)
    print(f"[INFO] Sent packet with type={message_type}, argument='{argument}'")

def read_response(ser: serial.Serial, last_argument_sent=None):
    if ser.in_waiting >= 17:
        header_and_size = ser.read(12)
        if header_and_size[:8] != HEADER:
            print("[ERROR] Invalid header")
            return

        size = struct.unpack('<I', header_and_size[8:12])[0]
        remaining = size + 4 + 1  # TYPE + PAYLOAD + CRC
        rest = ser.read(remaining)
        packet = header_and_size + rest

        try:
            group_id, payload = struct.unpack('<B', packet[12:13])[0], packet[13:-4]
            index = struct.unpack('<I', payload[:4])[0]  # get index from payload
            print(f"[RECEIVED] Group: {group_id}, Index: {index}, Payload Length: {len(payload)} bytes")

            filename = f"{group_id}_{index}.bin"
            folder = f"received_logs/group{group_id}"
            os.makedirs(folder, exist_ok=True)
            with open(os.path.join(folder, filename), 'wb') as f:
                f.write(packet)

        except Exception as e:
            print(f"[ERROR] Failed to parse response: {e}")
    else:
        print("[INFO] No response yet.")

def main():
    print(f"[INFO] Connecting to serial port {SERIAL_PORT} at {BAUD_RATE} baud")
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT_SEC) as ser:
            last_argument_sent = None
            while True:
                user_input = input("Enter command (n=next, r=resend, s <type>_<index>, k for kill, b for reboot): ").strip()
                if user_input == 'r':
                    send_request(ser, MESSAGE_TYPE_RESEND_LAST)
                    last_argument_sent = None
                elif user_input == 'n':
                    send_request(ser, MESSAGE_TYPE_SEND_NEXT)
                    last_argument_sent = None
                elif user_input.startswith("s "):
                    argument = user_input.split(" ", 1)[1]
                    send_request(ser, MESSAGE_TYPE_REQUEST_SPECIFIC, argument=argument)
                    last_argument_sent = argument
                elif user_input == 'k':
                    send_request(ser, MESSAGE_TYPE_SHUTDOWN)
                    last_argument_sent = None
                elif user_input == 'b':
                    send_request(ser, MESSAGE_TYPE_REBOOT)
                    last_argument_sent = None
                else:
                    print("[INFO] Unknown command. Use 'n', 'r', 's <type>_<index>', k or b")
                    continue

                time.sleep(1)
                read_response(ser, last_argument_sent=last_argument_sent)
                time.sleep(5)

    except serial.SerialException as e:
        print(f"[ERROR] Serial communication error: {e}")
    except KeyboardInterrupt:
        print("\n[INFO] Script interrupted. Exiting.")

if __name__ == '__main__':
    main()
