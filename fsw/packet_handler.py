import struct
import zlib
import os
import time

HEADER = b'\x30\x20\x30\x20\x30\x20\x30\x20'  # 64-bit header
LOG_BASE_DIR = '..\log'

# CRC and Packet Builder
def compute_crc32(data: bytes) -> int:
    return zlib.crc32(data)

"""
Build response packet in the format - [HEADER][SIZE][TYPE][PAYLOAD][CRC32]
"""
def build_response_packet(msg_type: int, payload: bytes) -> bytes:
    size = len(payload) + 1 + 4  # msg_type + CRC
    size_bytes = struct.pack('<I', size)
    type_byte = struct.pack('<B', msg_type)
    packet_wo_crc = size_bytes + type_byte + payload
    crc = compute_crc32(packet_wo_crc)
    
    return HEADER + packet_wo_crc + struct.pack('<I', crc)

def log_binary_packet(msg_type: int, payload: bytes):
    folder = os.path.join(LOG_BASE_DIR, f"type{msg_type}")
    os.makedirs(folder, exist_ok=True)
    timestamp = int(time.time() * 1000)
    filename = os.path.join(folder, f"{timestamp}.bin")
    with open(filename, 'wb') as f:
        f.write(build_response_packet(msg_type, payload))

""" Type 1: Time + 3 Voltages + 3 Currents (all float32) """
def build_packet_type_1(time_s: float, voltages: list, currents: list) -> bytes:
    payload = struct.pack('<f3f3f', time_s, *voltages, *currents)
    return build_response_packet(1, payload)

""" Type 2: 3 Channel Temps + 1 CPU Temp (all float32) """
def build_packet_type_2(temps: list, cpu_temp: float) -> bytes:
    payload = struct.pack('<4f', *temps, cpu_temp)
    return build_response_packet(2, payload)

""" 
    Type 3:Battery state estimator output (float32)
    est_soc, est_volt_v, est_cov[4] 
"""
def build_packet_type_3(ch) -> bytes:
    ## placeholder for state estimator output 
    payload=0.0
    return build_response_packet(3, payload)

"""
Type 4:Health summary
cycle_count (3x uint8), resets (uint16), time_switch (3x uint16), test_sequence (3x uint8)
"""
def build_packet_type_4(channels: list, resets: int, time_switch: int) -> bytes:
    payload = struct.pack('<3B H 3H 3B',
        *[ch.cycle_count for ch in channels],
        resets,
        *[time_switch]*3,
        *[ch.test_sequence for ch in channels]
    )
    return build_response_packet(4, payload)

""" 
Type 5: (System mode + state summary)
state: CHG, DIS, REST, CHG_LOW, DIS_LOW (encoded uint8)
mode: CYCLE, TEST (encoded uint8)
test_sequence: (uint8) 
"""
def build_packet_type_5(channels: list) -> bytes:
    state_map = {'CHG': 0, 'DIS': 1, 'REST': 2, 'CHG_LOW': 3, 'DIS_LOW': 4}
    mode_map = {'CYCLE': 0, 'TEST': 1}
    payload = struct.pack('<3B 3B 3B',
        *[state_map.get(ch.state, 255) for ch in channels],
        *[mode_map.get(ch.mode, 255) for ch in channels],
        *[ch.test_sequence for ch in channels]
    )
    return build_response_packet(5, payload)

# Request Packet Parser
def parse_request_packet(packet: bytes):
    if len(packet) < 17:
        raise ValueError("Incomplete packet")
    if packet[:8] != HEADER:
        raise ValueError("Invalid header")
    
    msg_type = struct.unpack('<B', packet[12:13])[0]
    payload = packet[13:-4]
    recv_crc = struct.unpack('<I', packet[-4:])[0]
    calc_crc = compute_crc32(packet[8:-4])

    if recv_crc != calc_crc:
        raise ValueError("Checksum mismatch")
    
    return msg_type, payload
