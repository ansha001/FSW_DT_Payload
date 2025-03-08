import time
from sensors import SensorReader
from gpio_handler import GPIOHandler
from potentiometer import PotentiometerControl
from bin_logger import DataLogger
from config import (
    DT_LOG, TEMP_MAX, TEMP_MIN, V_CHG_LIMIT, V_DIS_LIMIT, 
    TEMP_CHG_UPPER, TEMP_CHG_LOWER, CHG_UPPER_SETPT, CHG_LOWER_SETPT,
    CHG_SETPT_DELTA, DIS_SETPT, DIS_SETPT_DELTA, INA_ADDRS, TMP_ADDRS,
    POT_ADDRS, POT_REGS, EN_CHG_GPIO_LIST, EN_DIS_GPIO_LIST, EN_CUR_GPIO_LIST
)

def main():
    # Initialize Components
    gpio_handler = GPIOHandler()
    sensor_reader = SensorReader()
    logger = DataLogger()
    pot_control = PotentiometerControl()

    gpio_handler.init_gpio()
    gpio_handler.safe_board()

    # Initialize charging and discharging values for potentiometer
    chg_vals = [9, 9, 9] 
    dis_vals = [76, 76, 76] 

    #Initial Setup
    cycle_count = [0, 20, 20]
    cell_mode = ['CYCLE', 'IDLE', 'IDLE']
    cell_state = ['CHG', 'REST', 'REST']

    time_prev = time.monotonic()

    while True:
        time_iter = time.monotonic()

        if time_iter > time_prev + DT_LOG:
            sensor_data = sensor_reader.ping_sensors(cell_state[0])
            
            if len(sensor_data) == 11:
                logger.log_data(time_iter, sensor_data)
            else:
                print("Sensor data format incorrect. Skipping logging.")

            temp_data = sensor_data[:3]
            voltages = sensor_data[4:7]  # Voltages of cells (excluding CPU voltage)
            currents = sensor_data[8:11] 

            for i in range(1): 
                if temp_data[i] > TEMP_MAX or temp_data[i] < TEMP_MIN:
                    cell_mode[i] = 'IDLE'
                    cell_state[i] = 'IDLE'
                    gpio_handler.safe_board(i)
                print("Temperature out of range")

            for i in range(2): 
                if cycle_count[i] < 20:
                    cell_mode[i] = 'CYCLE'
                    if cell_state[i] == 'CHG' and voltages[i] < V_CHG_LIMIT:
                        cell_state[i] = 'CHG'
                    elif cell_state[i] == 'CHG' and voltages[i] >= V_CHG_LIMIT:
                        cell_state[i] = 'DIS'
                        gpio_handler.safe_board(i)
                        print(f'Cell {i}: Switching to Discharge')
                    elif cell_state[i] == 'DIS' and voltages[i] > V_DIS_LIMIT:
                        cell_state[i] = 'DIS'
                    elif cell_state[i] == 'DIS' and voltages[i] <= V_DIS_LIMIT:
                        cell_state[i] = 'CHG'
                        gpio_handler.safe_board(i)
                        cycle_count[i] += 1
                        print(f'Cell {i}: Switching to Charge')
                else:
                    cell_mode[i] = 'TEST'

                # Manage Charging & Discharging Setpoints
                if cell_mode[i] == 'CYCLE' and cell_state[i] == 'CHG':
                    if temp_data[i] > TEMP_CHG_UPPER:
                        charge_setpoint = CHG_UPPER_SETPT
                    elif temp_data[i] < TEMP_CHG_LOWER:
                        charge_setpoint = CHG_LOWER_SETPT
                    else:
                        charge_setpoint = CHG_LOWER_SETPT + (
                            (temp_data[i] - TEMP_CHG_LOWER) * (CHG_UPPER_SETPT - CHG_LOWER_SETPT)
                        ) / (TEMP_CHG_UPPER - TEMP_CHG_LOWER)

                    # Adjust charging potntiometer
                    if currents[i] < charge_setpoint - CHG_SETPT_DELTA:
                        chg_vals[i] += 1
                        pot_control.set_pot(i, chg_vals[i])
                    elif currents[i] > charge_setpoint + CHG_SETPT_DELTA:
                        chg_vals[i] -= 1
                        pot_control.set_pot(i, chg_vals[i])

                    gpio_handler.set_gpio(i, 'ON', EN_CHG_GPIO_LIST)

                elif cell_mode[i] == 'CYCLE' and cell_state[i] == 'DIS':
                    discharge_setpoint = DIS_SETPT

                    # Adjust discharging potentiometer
                    if currents[i] < discharge_setpoint - DIS_SETPT_DELTA:
                        dis_vals[i] += 1
                        pot_control.set_wiper(POT_ADDRS[3], POT_REGS[i], dis_vals[i])
                    elif currents[i] > discharge_setpoint + DIS_SETPT_DELTA:
                        dis_vals[i] -= 1
                        pot_control.set_wiper(POT_ADDRS[3], POT_REGS[i], dis_vals[i])

                    gpio_handler.set_gpio(i, 'ON', EN_DIS_GPIO_LIST)
                    gpio_handler.set_gpio(i, 'HIGH', EN_CUR_GPIO_LIST)
                    print(f"Cell {i}: Discharging configuration set")

                elif cell_mode[i] == 'TEST':
                    cycle_count[i] = 0
                    cell_mode[i] = 'CYCLE'
                    cell_state[i] = 'CHG'
                    gpio_handler.safe_board(i)
                else:
                    gpio_handler.safe_board(i)

            time_prev = time_iter
            print("heartbeat")

if __name__ == "__main__":
    main()
