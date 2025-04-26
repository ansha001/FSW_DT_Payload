import struct
import zlib
import os
import time
import json
import numpy as np

HEADER = b'\x30\x20\x30\x20\x30\x20\x30\x20'

MAX_FILE_INDEX = 10**6

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_BASE_DIR = os.path.join(BASE_DIR, 'log')
CONFIG_FILE = os.path.join(BASE_DIR, 'config.json')
BUFFER_BACKUP_FILE = os.path.join(LOG_BASE_DIR, 'buffers_backup.json')


with open(CONFIG_FILE, 'r') as f:
    CONFIG = json.load(f)
BUFFER_ENTRIES = CONFIG.get("buffer_entries", {})

reading_buffers = {
    1: [],
    2: [],
    3: []
}

# Load persisted buffer if it exists
def load_buffer_backup():
    if os.path.exists(BUFFER_BACKUP_FILE):
        try:
            with open(BUFFER_BACKUP_FILE, 'r') as f:
                data = json.load(f)
                for key in reading_buffers:
                    reading_buffers[key] = data.get(str(key), [])
            print("[INFO] Buffer state restored from backup.")
        except Exception as e:
            print(f"[ERROR] Failed to load buffer backup: {e}")

# Save current buffer to backup file
# to be Called periodically or before power cycle
def save_buffer_backup():
    try:
        with open(BUFFER_BACKUP_FILE, 'w') as f:
            json.dump(reading_buffers, f)
        #print("[INFO] Buffer state saved.")
    except Exception as e:
        print(f"[ERROR] Failed to save buffer backup: {e}")

load_buffer_backup()

# CRC and Packet Builder
def compute_crc32(data: bytes) -> int:
    return zlib.crc32(data)

# Build response packet in the format - [HEADER][SIZE][TYPE][PAYLOAD][CRC32]
def build_response_packet(msg_type: int, payload: bytes) -> bytes:
    size = len(payload) + 1 + 4
    size_bytes = struct.pack('<I', size)
    type_byte = struct.pack('<B', msg_type)
    packet_wo_crc = size_bytes + type_byte + payload
    crc = compute_crc32(packet_wo_crc)

    return HEADER + packet_wo_crc + struct.pack('<I', crc)

def get_group_folder(group_id: int) -> str:
    folder = os.path.join(LOG_BASE_DIR, f"group{group_id}")
    os.makedirs(folder, exist_ok=True)
    return folder

def log_binary_packet(group_id: int, payload: bytes):
    folder = get_group_folder(group_id)
    index = get_latest_index(group_id) + 1
    filename = f"{group_id}_{index}.bin"
    full_path = os.path.join(folder, filename)

    # Include index inside the payload
    index_bytes = struct.pack('<I', index) # pack index as 4 bytes (uint32)
    payload_with_index = index_bytes + payload

    # Wrap into packet with group_id as message type
    entry = build_response_packet(group_id, payload_with_index)

    with open(full_path, 'wb') as f:
        f.write(entry)

    print(f"[LOG] Group {group_id}: Written to {full_path}")


# Build packets for message groups
def build_packet_group_1(reading_list):
    payload = b''
    for reading in reading_list:
        # (time_s, volt0, volt1, volt2, curr0, curr1, curr2, temp0, temp1, temp2) = reading
        # arr = np.array([time_s, volt0, volt1, volt2, curr0, curr1, curr2, temp0, temp1, temp2], dtype=np.float16)
        arr = np.array(reading, dtype=np.float16)
        payload += arr.tobytes()
    return payload

def build_packet_group_2(reading_list):
    payload = b''
    for reading in reading_list:
        (
            time_s, 
            cyc0, cyc1, cyc2, 
            resets, 
            tms0, tms1, tms2, 
            seq0, seq1, seq2, 
            state0, state1, state2, 
            mode0, mode1, mode2, 
            cpu_temp, cpu_volt, cpu_freq
        ) = reading

        int_part = struct.pack('<3B H 3H 3B 3B 3B', cyc0, cyc1, cyc2, resets, tms0, tms1, tms2, seq0, seq1, seq2, state0, state1, state2, mode0, mode1, mode2)
        float_part = np.array([time_s, cpu_temp, cpu_volt, cpu_freq], dtype=np.float16).tobytes()
        payload += float_part + int_part
    return payload

def build_packet_group_3(reading_list):
    payload = b''
    for reading in reading_list:
        (
            time_s, 
            soc0, volt0, cov00_0, cov11_0, cap0, param0, cap_cyc0, cap_rpt0,
            pr_ekf_one0, pr_ekf_two0, pr_cyc_one0, pr_cyc_two0,
            soc1, volt1, cov00_1, cov11_1, cap1, param1, cap_cyc1, cap_rpt1,
            pr_ekf_one1, pr_ekf_two1, pr_cyc_one1, pr_cyc_two1,
            soc2, volt2, cov00_2, cov11_2, cap2, param2, cap_cyc2, cap_rpt2,
            pr_ekf_one2, pr_ekf_two2, pr_cyc_one2, pr_cyc_two2,
        ) = reading

        timestamp_bytes = np.array([time_s], dtype=np.float16).tobytes()
        estimator_data = struct.pack('<36f', soc0, volt0, cov00_0, cov11_0, cap0, param0, cap_cyc0, cap_rpt0, pr_ekf_one0, pr_ekf_two0, pr_cyc_one0, pr_cyc_two0,
                                     soc1, volt1, cov00_1, cov11_1, cap1, param1, cap_cyc1, cap_rpt1, pr_ekf_one1, pr_ekf_two1, pr_cyc_one1, pr_cyc_two1,
                                     soc2, volt2, cov00_2, cov11_2, cap2, param2, cap_cyc2, cap_rpt2, pr_ekf_one2, pr_ekf_two2, pr_cyc_one2, pr_cyc_two2)
        payload += timestamp_bytes + estimator_data
    return payload

def buffer_and_log_reading(group_id: int, reading: tuple):
    reading_buffers[group_id].append(reading)
    buffer_limit = BUFFER_ENTRIES.get(f"group{group_id}", 5)

    if len(reading_buffers[group_id]) >= buffer_limit:
        if group_id == 1:
            payload = build_packet_group_1(reading_buffers[group_id])
        elif group_id == 2:
            payload = build_packet_group_2(reading_buffers[group_id])
        elif group_id == 3:
            payload = build_packet_group_3(reading_buffers[group_id])
        else:
            raise ValueError("Invalid group ID")
        
        log_binary_packet(group_id, payload)
        reading_buffers[group_id].clear()

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

GLOBAL_POINTER_PATH = os.path.join(LOG_BASE_DIR, '.last_sent_group')

def read_global_pointer() -> int:
    try:
        with open(GLOBAL_POINTER_PATH, 'r') as f:
            return int(f.read().strip())
    except (FileNotFoundError, ValueError):
        return 0

def write_global_pointer(group_id: int):
    with open(GLOBAL_POINTER_PATH, 'w') as f:
        f.write(str(group_id % 256))

# for specific requests
def get_file_path(group_id: int, index: int) -> str:
    folder = get_group_folder(group_id)
    filename = f"{group_id}_{index}.bin"
    path = os.path.join(folder, filename)
    return path if os.path.exists(path) else None

def get_latest_index(group_id: int) -> int:
    folder = get_group_folder(group_id)
    max_index = -1
    for f in os.listdir(folder):
        if f.endswith(".bin") and f.startswith(f"{group_id}_"):
            try:
                idx = int(f.split('_')[1].replace(".bin", ""))
                if idx > max_index:
                    max_index = idx
            except:
                continue
    return max_index

def get_pointer_path(group_id: int) -> str:
    return os.path.join(get_group_folder(group_id), f".last_sent_pointer")

def read_pointer(group_id: int) -> int:
    path = get_pointer_path(group_id)
    try:
        with open(path, 'r') as f:
            return int(f.read().strip())
    except (FileNotFoundError, ValueError):
        return 0

def write_pointer(group_id: int, value: int):
    path = get_pointer_path(group_id)
    with open(path, 'w') as f:
        f.write(str(value % MAX_FILE_INDEX))

def get_next_packet_to_send() -> bytes:
    for group_id in [3, 2, 1]:  # Priority order
        last_sent = read_pointer(group_id)
        latest = get_latest_index(group_id)

        if latest > last_sent:
            path = get_file_path(group_id, last_sent + 1)
            if path:
                with open(path, 'rb') as f:
                    data = f.read()

                write_pointer(group_id, last_sent + 1)
                write_global_pointer(group_id)
                print(f"[DEBUG] last_sent = {last_sent}, latest = {latest} for group {group_id}")
                return data
    return None  # No packets ready

def get_last_sent_packet() -> bytes:
    group_id = read_global_pointer()
    last_sent_index = read_pointer(group_id)

    path = get_file_path(group_id, last_sent_index)
    if path and os.path.exists(path):
        with open(path, 'rb') as f:
            return f.read()
    return None

def handle_request_packet(packet: bytes) -> bytes:
    req_type, payload = parse_request_packet(packet)
    
    if req_type == 4:
        print("Shutdown request received.")
        return -1
    
    elif req_type == 5:
        print("Reboot request received.")
        return -2
    
    elif req_type == 0:
        print("Resend request received.")
        return get_last_sent_packet()

    elif req_type == 3:
        print("Update parameter request received.")
        # TODO parse payload
        param_index = struct.unpack('<B', payload[0:1])[0]
        param_value = struct.unpack('<f', payload[-4:])[0]
        param_name = PARAMS.fetch_parameter_name(param_index)
        print('Parameter: '+param_name+', Value: %8.5f' % param_value)
        if param_name is not None:
            print("Updating param")
            PARAMS.update_parameter(params_file, param_name, param_value)
        else:
            print("Invalid param index number")
        return None
    
    elif req_type == 2:
        print(f"Specific packet request received with argument: {payload}")
        requested_group, requested_index = parse_specific_request_argument(payload)
        if requested_group is not None and requested_index is not None:
            path = get_file_path(requested_group, requested_index)
            if path:
                with open(path, 'rb') as f:
                    return f.read()
            else:
                print(f"[ERROR] File not found: {requested_group}_{requested_index}.bin")
                return None
        else:
            print("[ERROR] Failed to decode specific request payload.")
            return None

    else: #should be message type 1
        print("Next packet request received.")
        return get_next_packet_to_send()

def parse_specific_request_argument(payload: bytes):
    try:
        decoded = payload.decode('utf-8')
        group_str, index_str = decoded.split('_')
        return int(group_str), int(index_str)
    except Exception:
        print("[ERROR] Invalid specific packet argument. Expected something like '2_3'")
        return None, None
    
