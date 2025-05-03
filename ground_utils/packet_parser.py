import struct
import os
import json
import csv
from collections import defaultdict

HEADER = b'\x89\x89\x89\x89\x89\x89\x89\x89'

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
    entry_size = 22  # 1 float 32 and 9 float16 values = 22 bytes
    total_entries = len(payload) // entry_size
    parsed_entries = []

    for i in range(total_entries):
        entry = payload[i*entry_size:(i+1)*entry_size]
        values = struct.unpack('<1f 9e', entry)
        parsed_entries.append({
            "time": values[0],
            "voltages": values[1:4],
            "currents": values[4:7],
            "temps": values[7:10]
        })

    return parsed_entries

def parse_group_2_packet(payload: bytes):
    entry_size = struct.calcsize('<1f 3e 3B H 3H 3B 3B 3B')  # 1 float32, 3 float16s + ints
    parsed_entries = []
    total_entries = len(payload) // entry_size

    for i in range(total_entries):
        entry = payload[i * entry_size:(i + 1) * entry_size]

        time_s, cpu_temp, cpu_volt, cpu_freq = struct.unpack('<1f 3e', entry[:10])
        unpacked = struct.unpack('<3B H 3H 3B 3B 3B', entry[10:])

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
    entry_size = 4 + struct.calcsize('<33f')  # timestamp + 33 float32s
    total_entries = len(payload) // entry_size
    parsed_entries = []

    for i in range(total_entries):
        entry = payload[i*entry_size:(i+1)*entry_size]
        time_s = struct.unpack('<f', entry[:4])[0]
        estimator_data = struct.unpack('<33f', entry[4:])
        channels = []
        for j in range(3):
            offset = j * 11
            channels.append({
                "est_soc": estimator_data[offset + 0],
                "volt": estimator_data[offset + 1],
                "cov00": estimator_data[offset + 2],
                "cov11": estimator_data[offset + 3],
                "est_cap": estimator_data[offset + 4],
                "covp": estimator_data[offset + 5],
                "cap_cyc": estimator_data[offset+6],
                "cap_rpt": estimator_data[offset+7],
                "pred_cyc": estimator_data[offset + 8],
                "pred_ekf": estimator_data[offset + 9],
                "pred_beta": estimator_data[offset + 10],
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
                    'cpu_temp', 'cpu_volt', 'cpu_freq'
                ])
            elif group_id == 3:
                writer.writerow(['time'] + [
                    f"ch{ch}_{field}"
                    for ch in range(3)
                    for field in ['est_soc','volt','cov00','cov11','est_cap','covp','cap_cyc','cap_rpt',
                                  'pred_cyc','pred_ekf','pred_beta']
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
                    entry['cpu_volt'],
                    entry['cpu_freq']
                ])
            elif group_id == 3:
                flat = []
                for ch in entry['channels']:
                    flat.extend([
                        ch["est_soc"], ch["volt"],
                        ch["cov00"], ch["cov11"],
                        ch["est_cap"], ch["covp"],
                        ch["cap_cyc"], ch["cap_rpt"], ch["pred_cyc"],
                        ch["pred_ekf"], ch["pred_beta"]
                    ])
                writer.writerow([entry["time"]] + flat)

def parse_folder(folder_path):
    # files need to be sorted by number, not how sorted fxn normally works.
    badly_sorted_list = sorted(os.listdir(folder_path))
    first = 10e7
    last = 1
    for fname in badly_sorted_list:
        group_str = fname.split('_')[0].replace(".bin","")
        index = int(fname.split('_')[1].replace(".bin",""))
        first = min(first, index)
        last  = max(last,  index)
    for i in range(first, last, 1):
        fname = folder_path + '/' + group_str + '_' + str(i) + '.bin'
        if os.path.exists(fname):
            print(f"--- Parsing {fname} ---")
            parse_bin_file(os.path.join(folder_path, fname))
        
    
#     for fname in sorted(os.listdir(folder_path)):
#         if fname.endswith(".bin"):
#             print(f"--- Parsing {fname} ---")
#             parse_bin_file(os.path.join(folder_path, fname))

if __name__ == '__main__':
    file_path = input("Enter folder path:").strip()
    parse_folder(file_path)
