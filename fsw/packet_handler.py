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
    return os.path.join(folder, f"{msg_type}_{index}.bin")

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

def build_packet_type_3(ch0, ch1, ch2) -> bytes:
#     print(ch0.est_cov_state)
#     print(ch0.est_cov_state[0, 0])
#     print(ch0.est_cov_state[1, 1])
    return struct.pack(
        #'<' + '2f4f4f4f4f' * 3,
        '<' + '1f1f1f1f1f1f' * 3,
        float(ch0.est_soc),
        float(ch0.est_volt_v),
        float(ch0.est_cov_state[0,0]),
        float(ch0.est_cov_state[1,1]),
        float(ch0.est_capacity_as),
        float(ch0.est_cov_param),
        #ch0.ohmic_resistance,
        #ch0.capacitance,
        #ch0.resistance,
        
        float(ch1.est_soc),
        float(ch1.est_volt_v),
        float(ch1.est_cov_state[0,0]),
        float(ch1.est_cov_state[1,1]),
        float(ch1.est_capacity_as),
        float(ch1.est_cov_param),
        #ch1.ohmic_resistance,
        #ch1.capacitance,
        #ch1.resistance,

        float(ch2.est_soc),
        float(ch2.est_volt_v),
        float(ch2.est_cov_state[0,0]),
        float(ch2.est_cov_state[1,1]),
        float(ch2.est_capacity_as),
        float(ch2.est_cov_param)
        #ch2.ohmic_resistance,
        #ch2.capacitance,
        #ch2.resistance
        )

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

def get_file_path(msg_type: int, index: int) -> str:
    folder = get_chunk_folder(msg_type)
    filename = f"{msg_type}_{index}.bin"
    path = os.path.join(folder, filename)
    return path if os.path.exists(path) else None

def get_latest_index(msg_type: int) -> int:
    folder = get_chunk_folder(msg_type)
    max_index = -1
    for f in os.listdir(folder):
        if f.endswith(".bin") and f.startswith(f"{msg_type}_"):
            try:
                idx = int(f.split('_')[1].replace(".bin", ""))
                if idx > max_index:
                    max_index = idx
            except:
                continue
    return max_index

def get_next_packet_to_send() -> bytes:
    for msg_type in [3, 2, 1]:  # Priority order
        last_sent = read_pointer(msg_type, 'last_sent')
        latest = get_latest_index(msg_type)

        if latest > last_sent:
            path = get_file_path(msg_type, last_sent + 1)
            if path:
                with open(path, 'rb') as f:
                    data = f.read()

                write_pointer(msg_type, last_sent + 1, 'last_sent')
                write_global_pointer(msg_type)
                return data
    return None  # No packets ready

def get_last_sent_packet() -> bytes:
    msg_type = read_global_pointer()
    last_sent_index = read_pointer(msg_type, 'last_sent')

    path = get_file_path(msg_type, last_sent_index)
    if path and os.path.exists(path):
        with open(path, 'rb') as f:
            return f.read()
    return None

def handle_request_packet(packet: bytes) -> bytes:
    msg_type, payload = parse_request_packet(packet)

    if msg_type == 0:
        print("Resend request received.")
        return get_last_sent_packet()

    elif msg_type == 2:
        print(f"Specific packet request received with argument: {payload}")
        requested_type, requested_index = parse_specific_request_argument(payload)
        if requested_type is not None and requested_index is not None:
            path = get_file_path(requested_type, requested_index)
            if path:
                with open(path, 'rb') as f:
                    return f.read()
            else:
                print(f"[ERROR] File not found: {requested_type}_{requested_index}.bin")
                return None
        else:
            print("[ERROR] Failed to decode specific request payload.")
            return None

    else:
        print("Next packet request received.")
        return get_next_packet_to_send()

    
    
def parse_specific_request_argument(payload: bytes):
    try:
        decoded = payload.decode('utf-8')
        msg_type_str, index_str = decoded.split('_')
        return int(msg_type_str), int(index_str)
    except Exception:
        print("[ERROR] Invalid specific packet argument. Expected something like '2_3'")
        return None, None
    

