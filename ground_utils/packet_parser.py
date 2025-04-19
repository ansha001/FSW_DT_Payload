import struct
import os

HEADER = b'\x30\x20\x30\x20\x30\x20\x30\x20'

STATE_NAMES = {
    0: 'CHG',
    1: 'DIS',
    2: 'REST',
    3: 'CHG_REST',
    4: 'DIS_REST',
    5: 'CHG_LOW',
    6: 'DIS_LOW',
    255: 'UNKNOWN'
}

MODE_NAMES = {
    0: 'CYCLE',
    1: 'TEST',
    255: 'UNKNOWN'
}

def parse_group_1_packet(payload: bytes):
    if len(payload) != 20:  # 10 float16 values = 20 bytes
        raise ValueError("Payload size for group 1 must be 20 bytes")
    values = struct.unpack('<10e', payload)
    return {
        "time": values[0],
        "voltages": values[1:4],
        "currents": values[4:7],
        "temps": values[7:10]
    }

def parse_group_2_packet(payload: bytes):
    time_s = struct.unpack('<e', payload[:2])[0]
    unpacked = struct.unpack('<3B H 3H 3B 3B 3B', payload[2:-4])
    cycle_counts = unpacked[0:3]
    resets = unpacked[3]
    time_switches = unpacked[4:7]
    test_sequences = unpacked[7:10]
    state_codes = unpacked[10:13]
    mode_codes = unpacked[13:16]
    cpu_temp, cpu_volt = struct.unpack('<2e', payload[-4:])

    return {
        "time": time_s,
        "cycle_counts": cycle_counts,
        "resets": resets,
        "time_switches": time_switches,
        "test_sequences": test_sequences,
        "states": [STATE_NAMES.get(code, 'UNKNOWN') for code in state_codes],
        "modes": [MODE_NAMES.get(code, 'UNKNOWN') for code in mode_codes],
        "cpu_temp": cpu_temp,
        "cpu_volt": cpu_volt
    }

def parse_group_3_packet(payload: bytes):
    if len(payload) != 74:  # 1 float16 (2 bytes) + 18 floats (72 bytes)
        raise ValueError("Payload size for group 3 must be 74 bytes")
    time_s = struct.unpack('<e', payload[:2])[0]
    estimator_data = struct.unpack('<18f', payload[2:])
    channels = []
    for i in range(3):
        offset = i * 6
        channels.append({
            "estimated_soc": estimator_data[offset + 0],
            "estimated_voltage": estimator_data[offset + 1],
            "cov_state_00": estimator_data[offset + 2],
            "cov_state_11": estimator_data[offset + 3],
            "capacity": estimator_data[offset + 4],
            "cov_param": estimator_data[offset + 5]
        })
    return {"time": time_s, "channels": channels}

def parse_packet(packet: bytes):
    if packet[:8] != HEADER:
        raise ValueError("Invalid packet header")
    size = struct.unpack('<I', packet[8:12])[0]
    group_id = struct.unpack('<B', packet[12:13])[0]
    index = struct.unpack('<I', packet[13:17])[0]  # new index field
    payload = packet[17:17+size-8]  # skip 1 byte group_id + 4 byte index + 4 byte CRC using size
    crc_recv = struct.unpack('<I', packet[17+size-8:17+size-4])[0]
    return group_id, index, payload

def parse_bin_file(file_path):
    with open(file_path, 'rb') as f:
        while True:
            header = f.read(8)
            if not header:
                break  # EOF
            size_bytes = f.read(4)
            if len(size_bytes) < 4:
                break
            size = struct.unpack('<I', size_bytes)[0]
            packet = header + size_bytes + f.read(size)
            group_id, index, payload = parse_packet(packet)
            if group_id == 1:
                parsed = parse_group_1_packet(payload)
            elif group_id == 2:
                parsed = parse_group_2_packet(payload)
            elif group_id == 3:
                parsed = parse_group_3_packet(payload)
            else:
                parsed = {"group_id": group_id, "raw_payload": payload.hex()}
            print(f"Parsed Group {group_id}, Index {index}:", parsed)
            print("-"*60)

def parse_folder(folder_path):
    for fname in sorted(os.listdir(folder_path)):
        if fname.endswith(".bin"):
            print(f"--- Parsing {fname} ---")
            parse_bin_file(os.path.join(folder_path, fname))

if __name__ == '__main__':
    file_path = input("Enter file_path:").strip()
    parse_bin_file(file_path)
