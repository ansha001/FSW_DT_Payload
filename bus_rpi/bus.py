import serial
import struct
import zlib
import time

# Constants
HEADER = b'\x30\x20\x30\x20\x30\x20\x30\x20'  # 64-bit - 0x3020302030203020
SERIAL_PORT = '/dev/ttyUSB0'           
BAUD_RATE = 9600
TIMEOUT_SEC = 1
MESSAGE_TYPE_REQUEST_DATA = 1  



def compute_crc32(data: bytes) -> int:
    return zlib.crc32(data)

def build_packet(message_type: int) -> bytes:
    """
    Builds a send packet in the format of [HEADER][SIZE][TYPE][CHECKSUM]
    """
    payload = struct.pack('<B', message_type)  # 1 byte - message type
    size = len(payload) + 4                    # 4 bytes - checksum
    size_bytes = struct.pack('<I', size)       # 4 bytes - message size
    packet_without_checksum = size_bytes + payload

    checksum = compute_crc32(packet_without_checksum)
    checksum_bytes = struct.pack('<I', checksum)
    
    send_packet = HEADER + packet_without_checksum + checksum_bytes
    return send_packet

def parse_response_packet(packet: bytes):
    """
    Parse response packet and validate structure and checksum.
    Return the message type and payload.
    """
    if len(packet) < 17:  # HEADER (8) + SIZE (4) + TYPE (1) + CHECKSUM (4) = 17 bytes
        raise ValueError("Incomplete packet receivd")

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

def send_request(ser: serial.Serial, message_type: int = MESSAGE_TYPE_REQUEST_DATA):
    packet = build_packet(message_type)
    ser.write(packet)
    print(f"[INFO] Sent packet with message type {message_type}")

def read_response(ser: serial.Serial):
    if ser.in_waiting > 0:
        raw_data = ser.read(ser.in_waiting)
        try:
            msg_type, payload = parse_response_packet(raw_data)
            print(f"[RECEIVED] Type: {msg_type}, Payload: {payload}")
        except Exception as e:
            print(f"[ERROR] Failed to parse response: {e}")
    else:
        print("[INFO] No response received")


def main():
    print(f"[INFO] Connecting to serial port {SERIAL_PORT} at {BAUD_RATE} baud")
    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT_SEC) as ser:
            while True:
                send_request(ser)
                time.sleep(1)
                read_response(ser)
                time.sleep(5) 

    except serial.SerialException as e:
        print(f"[ERROR] Serial communication error: {e}")
    except KeyboardInterrupt:
        print("\n[INFO] Script interrupted. Exiting.")


if __name__ == '__main__':
    main()
