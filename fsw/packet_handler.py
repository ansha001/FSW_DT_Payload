import struct
import zlib
import os
import time
import json

HEADER = b'\x30\x20\x30\x20\x30\x20\x30\x20'

MAX_FILE_INDEX = 10**6

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_BASE_DIR = os.path.join(BASE_DIR, 'log')
CONFIG_FILE = os.path.join(BASE_DIR, 'config.json')

with open(CONFIG_FILE, 'r') as f:
    CONFIG = json.load(f)
CHUNK_ENTRIES = CONFIG.get("chunk_entries", {})

# CRC and Packet Builder
def compute_crc32(data: bytes) -> int:
    return zlib.crc32(data)

"""
Build response packet in the format - [HEADER][SIZE][TYPE][PAYLOAD][CRC32]
"""
def build_response_packet(msg_type: int, payload: bytes) -> bytes:
    size = len(payload) + 1 + 4
    size_bytes = struct.pack('<I', size)
    type_byte = struct.pack('<B', msg_type)
    packet_wo_crc = size_bytes + type_byte + payload
    crc = compute_crc32(packet_wo_crc)

    return HEADER + packet_wo_crc + struct.pack('<I', crc)

def get_chunk_folder(msg_type: int) -> str:
    folder = os.path.join(LOG_BASE_DIR, f"type{msg_type}")
    os.makedirs(folder, exist_ok=True)
    return folder

def get_pointer_path(msg_type: int, pointer_name='current') -> str:
    return os.path.join(get_chunk_folder(msg_type), f'.{pointer_name}_pointer')

def read_pointer(msg_type: int, pointer_name='current') -> int:
    path = get_pointer_path(msg_type, pointer_name)
    try:
        with open(path, 'r') as f:
            return int(f.read().strip())
    except (FileNotFoundError, ValueError):
        return 0
    
def write_pointer(msg_type: int, value: int, pointer_name='current'):
    path = get_pointer_path(msg_type, pointer_name)
    with open(path, 'w') as f:
        f.write(str(value % MAX_FILE_INDEX))

def get_current_chunk_file(msg_type: int) -> str:
    folder = get_chunk_folder(msg_type)
    index = read_pointer(msg_type, 'current')
    timestamp = time.strftime("%H%M%S")
    return os.path.join(folder, f"{timestamp}_{index}.bin")

def increment_pointer(msg_type: int):
    index = read_pointer(msg_type, 'current') + 1
    write_pointer(msg_type, index, 'current')


def log_binary_packet(msg_type: int, payload: bytes):
    chunk_file = get_current_chunk_file(msg_type)
    entry = build_response_packet(msg_type, payload)

    with open(chunk_file, 'ab') as f:
        f.write(entry)

    chunk_limit = CHUNK_ENTRIES.get(f"type{msg_type}", 20)
    with open(chunk_file, 'rb') as f:
        count = 0
        while f.read(8):
            size_bytes = f.read(4)
            if len(size_bytes) < 4:
                break
            size = struct.unpack('<I', size_bytes)[0]
            f.read(size)
            count += 1

    if count >= chunk_limit:
        increment_pointer(msg_type)

def build_packet_type_1(time_s: float, voltages, currents, temps) -> bytes:
    return struct.pack('<f3f3f3f', time_s, *voltages, *currents, *temps)

def build_packet_type_2(channels, resets, time_switches, cpu_temp: float, cpu_volt: float) -> bytes:
    return struct.pack(
        '<3B H 3H 3B 3B 3B 2f',
        *[ch.cycle_count for ch in channels],
        resets,
        *time_switches,
        *[ch.test_sequence for ch in channels],
        *[ch.state_code for ch in channels],
        *[ch.mode_code for ch in channels],
        cpu_temp,
        cpu_volt
    )

def build_packet_type_3(estimator) -> bytes:
    # TODO: Implement this function based on the estimator structure
    return struct.pack(    )

def parse_request_packet(packet: bytes):
    if len(packet) < 17 or packet[:8] != HEADER:
        raise ValueError("Invalid or incomplete packet")
    msg_type = struct.unpack('<B', packet[12:13])[0]
    payload = packet[13:-4]
    recv_crc = struct.unpack('<I', packet[-4:])[0]
    calc_crc = compute_crc32(packet[8:-4])
    if recv_crc != calc_crc:
        raise ValueError("Checksum mismatch")
    return msg_type, payload


GLOBAL_POINTER_PATH = os.path.join(LOG_BASE_DIR, '.last_sent_msg_type')

def read_global_pointer() -> int:
    try:
        with open(GLOBAL_POINTER_PATH, 'r') as f:
            return int(f.read().strip())
    except (FileNotFoundError, ValueError):
        return 0

def write_global_pointer(msg_type: int):
    with open(GLOBAL_POINTER_PATH, 'w') as f:
        f.write(str(msg_type % 256))