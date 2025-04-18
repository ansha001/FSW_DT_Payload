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

def parse_packet_type_1(payload: bytes):
    time_s, *values = struct.unpack('<f3f3f3f', payload)
    voltages = values[0:3]
    currents = values[3:6]
    temps = values[6:9]
    return {
        "time": time_s,
        "voltages": voltages,
        "currents": currents,
        "temps": temps
    }

def parse_packet_type_2(payload: bytes):
    unpacked = struct.unpack('<3B H 3H 3B 3B 3B 2f', payload)
    cycle_counts = unpacked[0:3]
    resets = unpacked[3]
    time_switches = unpacked[4:7]
    test_sequences = unpacked[7:10]
    state_codes = unpacked[10:13]
    mode_codes = unpacked[13:16]
    cpu_temp, cpu_volt = unpacked[16:18]

    return {
        "cycle_counts": cycle_counts,
        "resets": resets,
        "time_switches": time_switches,
        "test_sequences": test_sequences,
        "states": [STATE_NAMES.get(code, 'UNKNOWN') for code in state_codes],
        "modes": [MODE_NAMES.get(code, 'UNKNOWN') for code in mode_codes],
        "cpu_temp": cpu_temp,
        "cpu_volt": cpu_volt
    }

def parse_packet_type_3(payload: bytes):
    fields_per_channel = struct.unpack('<2f4f4f4f4f'*3, payload)
    channels = []
    for i in range(3):
        offset = i * 18  # 2 + 4 + 4 + 4 + 4 + 4 = 18 floats per channel
        channel_data = fields_per_channel[offset:offset+18]
        channels.append({
            "estimated_soc": channel_data[0],
            "estimated_voltage": channel_data[1],
            "cov_state": channel_data[2:6],
            "capacity": channel_data[6],
            "ohmic_resistance": channel_data[7],
            "capacitance": channel_data[8],
            "resistance": channel_data[9],
            "cov_param": channel_data[10:]
        })
    return channels

def parse_packet(packet: bytes):
    if packet[:8] != HEADER:
        raise ValueError("Invalid packet header")
    size = struct.unpack('<I', packet[8:12])[0]
    msg_type = struct.unpack('<B', packet[12:13])[0]
    payload = packet[13:13+size-5]  # size excludes header but includes type + CRC
    crc_recv = struct.unpack('<I', packet[13+size-5:13+size-1])[0]
    return msg_type, payload

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
            msg_type, payload = parse_packet(packet)
            if msg_type == 1:
                parsed = parse_packet_type_1(payload)
            elif msg_type == 2:
                parsed = parse_packet_type_2(payload)
            elif msg_type == 3:
                parsed = parse_packet_type_3(payload)
            else:
                parsed = {"msg_type": msg_type, "raw_payload": payload.hex()}
            print(f"Parsed Packet Type {msg_type}:\n", parsed)
            print("-"*60)

def parse_folder(folder_path):
    for fname in sorted(os.listdir(folder_path)):
        if fname.endswith(".bin"):
            print(f"--- Parsing {fname} ---")
            parse_bin_file(os.path.join(folder_path, fname))

if __name__ == '__main__':
    folder = "log_files"  
    parse_folder(folder)
