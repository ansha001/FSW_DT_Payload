import pandas as pd
import numpy as np
import os

actual_log_path = 'ground_utils/actual_log.csv'
actual = pd.read_csv(actual_log_path)
actual.rename(columns={"Time (s)": "time"}, inplace=True)
actual['time'] = actual['time'].astype(float)

def find_nearest_row(df, target_time):
    idx = (df['time'] - target_time).abs().idxmin()
    return df.loc[idx]

# === GROUP 1 COMPARISON ===
group1 = pd.read_csv("ground_utils/parsed_csv/group1_parsed.csv")
group1_results = []

for _, row in group1.iterrows():
    parsed_time = row['time']
    actual_row = find_nearest_row(actual, parsed_time)
    result = {
        "parsed_time": parsed_time,
        "actual_time": actual_row['time'],
        "delta_time": abs(parsed_time - actual_row['time'])
    }
    for j in range(3):
        result[f'v{j}_parsed'] = row[f'v{j}']
        result[f'v{j}_actual'] = actual_row[f'Volt{j}']
        result[f'c{j}_parsed'] = row[f'c{j}']
        result[f'c{j}_actual'] = actual_row[f'Curr{j}']
        result[f't{j}_parsed'] = row[f't{j}']
        result[f't{j}_actual'] = actual_row[f'Temp{j}']
    group1_results.append(result)

pd.DataFrame(group1_results).to_csv("ground_utils/group1_comparison.csv", index=False)


# === GROUP 2 COMPARISON ===
group2 = pd.read_csv("parsed_csv/group2_parsed.csv")
group2_results = []

for _, row in group2.iterrows():
    parsed_time = row['time']
    actual_row = find_nearest_row(actual, parsed_time)
    result = {
        "parsed_time": parsed_time,
        "actual_time": actual_row['time'],
        "delta_time": abs(parsed_time - actual_row['time']),
        "resets": row['resets'],
        "cpu_temp_parsed": row['cpu_temp'],
        "cpu_temp_actual": actual_row['Temp_CPU'],
        "cpu_volt_parsed": row['cpu_volt'],
        "cpu_volt_actual": actual_row['Volt_CPU']
    }
    for j in range(3):
        result[f'cycle{j}_parsed'] = row[f'cycle{j}']
        result[f'seq{j}_parsed'] = row[f'test_seq{j}']
        result[f'state{j}_parsed'] = row[f'state{j}']
        result[f'mode{j}_parsed'] = row[f'mode{j}']
    group2_results.append(result)

pd.DataFrame(group2_results).to_csv("ground_utils/group2_comparison.csv", index=False)


# === GROUP 3 COMPARISON ===
group3 = pd.read_csv("ground_utils/parsed_csv/group3_parsed.csv")
group3_results = []

for _, row in group3.iterrows():
    parsed_time = row['time']
    actual_row = find_nearest_row(actual, parsed_time)
    result = {
        "parsed_time": parsed_time,
        "actual_time": actual_row['time'],
        "delta_time": abs(parsed_time - actual_row['time'])
    }
    for j in range(3):
        result[f'soc{j}_parsed'] = row[f'ch{j}_soc']
        result[f'volt{j}_parsed'] = row[f'ch{j}_volt']
        result[f'cap{j}_parsed'] = row[f'ch{j}_cap']
        result[f'est_cap{j}_actual'] = actual_row[f'estCAP{j}']
        result[f'est_soc{j}_actual'] = actual_row[f'estSOC{j}']
        result[f'est_volt{j}_actual'] = actual_row[f'estV{j}']
    group3_results.append(result)

pd.DataFrame(group3_results).to_csv("ground_utils/group3_comparison.csv", index=False)
