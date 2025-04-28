import struct
import os
import json
import csv
from collections import defaultdict

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

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(BASE_DIR, 'config.json')

with open(CONFIG_FILE, 'r') as f:
    config = json.load(f)
BUFFER_ENTRIES = {
    1: config.get("buffer_entries", {}).get("group1", 1),
    2: config.get("buffer_entries", {}).get("group2", 1),
    3: config.get("buffer_entries", {}).get("group3", 1)
}

def parse_group_1_packet(payload: bytes):
    entry_size = 20  # 10 float16 values = 20 bytes
    total_entries = len(payload) // entry_size
    parsed_entries = []

    for i in range(total_entries):
        entry = payload[i*entry_size:(i+1)*entry_size]
        values = struct.unpack('<10e', entry)
        parsed_entries.append({
            "time": values[0],
            "voltages": values[1:4],
            "currents": values[4:7],
            "temps": values[7:10]
        })

    return parsed_entries

def parse_group_2_packet(payload: bytes):
    entry_size = struct.calcsize('<4e 3B H 3H 3B 3B 3B')  # 4 float16s + ints
    parsed_entries = []
    total_entries = len(payload) // entry_size

    for i in range(total_entries):
        entry = payload[i * entry_size:(i + 1) * entry_size]

        time_s, cpu_temp, cpu_volt, cpu_freq = struct.unpack('<4e', entry[:8])
        unpacked = struct.unpack('<3B H 3H 3B 3B 3B', entry[8:])

        cycle_counts = unpacked[0:3]
        resets = unpacked[3]
        time_switches = unpacked[4:7]
        test_sequences = unpacked[7:10]
        state_codes = unpacked[10:13]
        mode_codes = unpacked[13:16]

        parsed_entries.append({
            "time": time_s,
            "cpu_temp": cpu_temp,
            "cpu_volt": cpu_volt,
            "cpu_freq": cpu_freq,
            "cycle_counts": cycle_counts,
            "resets": resets,
            "time_switches": time_switches,
            "test_sequences": test_sequences,
            "states": [STATE_NAMES.get(code, 'UNKNOWN') for code in state_codes],
            "modes": [MODE_NAMES.get(code, 'UNKNOWN') for code in mode_codes],
        })

    return parsed_entries

def parse_group_3_packet(payload: bytes):
    entry_size = 2 + struct.calcsize('<36f')  # timestamp + 36 float32s
    total_entries = len(payload) // entry_size
    parsed_entries = []

    for i in range(total_entries):
        entry = payload[i*entry_size:(i+1)*entry_size]
        time_s = struct.unpack('<e', entry[:2])[0]
        estimator_data = struct.unpack('<36f', entry[2:])
        channels = []
        for j in range(3):
            offset = j * 12
            channels.append({
                "estimated_soc": estimator_data[offset + 0],
                "estimated_voltage": estimator_data[offset + 1],
                "cov_state_00": estimator_data[offset + 2],
                "cov_state_11": estimator_data[offset + 3],
                "capacity": estimator_data[offset + 4],
                "cov_param": estimator_data[offset + 5],
                "cap_cyc": estimator_data[offset+6],
                "cap_rpt": estimator_data[offset+7],
                "pred_ekf_one": estimator_data[offset + 8],
                "pred_ekf_two": estimator_data[offset + 9],
                "pred_cyc_one": estimator_data[offset + 10],
                "pred_cyc_two": estimator_data[offset + 11],
            })
        parsed_entries.append({"time": time_s, "channels": channels})

    return parsed_entries

def parse_packet(packet: bytes):
    if packet[:8] != HEADER:
        raise ValueError("Invalid packet header")
    size = struct.unpack('<I', packet[8:12])[0]
    group_id = struct.unpack('<B', packet[12:13])[0]
    index = struct.unpack('<I', packet[13:17])[0]  # new index field
    payload = packet[17:17+size-8]  
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

            if parsed is not None:
                print(f"Parsed Group {group_id}, Index {index}: {parsed}")
                write_csv(group_id, parsed)
            else:
                print(f"Unknown group {group_id}, skipping")
            print("-" * 60)


def write_csv(group_id, entries, output_folder='parsed_csv'):
    os.makedirs(output_folder, exist_ok=True)
    csv_path = os.path.join(output_folder, f"group{group_id}_parsed.csv")
    print(f"Writing {len(entries)} entries to {csv_path}")

    if not entries:
        return  # Nothing to write

    file_exists = os.path.exists(csv_path)

    with open(csv_path, mode='a', newline='') as f:
        writer = csv.writer(f)

        # Write header only if file didn't exist
        if not file_exists:
            if group_id == 1:
                writer.writerow(['time', 'v0', 'v1', 'v2', 'c0', 'c1', 'c2', 't0', 't1', 't2'])
            elif group_id == 2:
                writer.writerow([
                    'time', 'cycle0', 'cycle1', 'cycle2', 'resets',
                    'time_switch0', 'time_switch1', 'time_switch2',
                    'test_seq0', 'test_seq1', 'test_seq2',
                    'state0', 'state1', 'state2',
                    'mode0', 'mode1', 'mode2',
                    'cpu_temp', 'cpu_volt'
                ])
            elif group_id == 3:
                writer.writerow(['time'] + [
                    f"ch{ch}_{field}"
                    for ch in range(3)
                    for field in ['soc', 'volt', 'cov00', 'cov11', 'cap', 'covp']
                ])

        # Write the actual data
        for entry in entries:
            if group_id == 1:
                writer.writerow([
                    entry["time"],
                    *entry["voltages"],
                    *entry["currents"],
                    *entry["temps"]
                ])
            elif group_id == 2:
                writer.writerow([
                    entry['time'],
                    *entry['cycle_counts'],
                    entry['resets'],
                    *entry['time_switches'],
                    *entry['test_sequences'],
                    *entry['states'],
                    *entry['modes'],
                    entry['cpu_temp'],
                    entry['cpu_volt']
                ])
            elif group_id == 3:
                flat = []
                for ch in entry['channels']:
                    flat.extend([
                        ch["estimated_soc"], ch["estimated_voltage"],
                        ch["cov_state_00"], ch["cov_state_11"],
                        ch["capacity"], ch["cov_param"]
                    ])
                writer.writerow([entry["time"]] + flat)


def parse_folder(folder_path):
    for fname in sorted(os.listdir(folder_path)):
        if fname.endswith(".bin"):
            print(f"--- Parsing {fname} ---")
            parse_bin_file(os.path.join(folder_path, fname))

if __name__ == '__main__':
    file_path = input("Enter folder path:").strip()
    parse_folder(file_path)
