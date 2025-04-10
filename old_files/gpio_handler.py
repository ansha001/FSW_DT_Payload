import RPi.GPIO as GPIO
from config import (
    EN_CHG_GPIO_LIST, EN_DIS_GPIO_LIST, EN_CUR_GPIO_LIST, 
    POT_ADDRS, POT_REGS
)

class GPIOHandler:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.init_gpio()

    def init_gpio(self):
        for pin_group in [EN_CHG_GPIO_LIST, EN_DIS_GPIO_LIST, EN_CUR_GPIO_LIST]:
            for pin in pin_group:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)

    def set_gpio(self, cell_num, state, gpio_list):
        GPIO.output(gpio_list[cell_num], GPIO.HIGH if state in ['ON', 'HIGH'] else GPIO.LOW)

    def safe_board(self, cell=-1):
        if cell == -1:
            # Safe settings for all
            for i in range(3):
                self.set_gpio(i, 'OFF', EN_CHG_GPIO_LIST)
                self.set_gpio(i, 'OFF', EN_DIS_GPIO_LIST)
                self.set_gpio(i, 'LOW', EN_CUR_GPIO_LIST)
            
            for pot in range(4): # 4 potentiometer
                for reg in range(4):  # 4 wiper registers per potentiometer
                    self.pot_control.set_wiper(POT_ADDRS[pot], POT_REGS[reg], 256 if pot < 3 else 0)
            
            print("Board safe")

        else:
            self.set_gpio(cell, 'OFF', EN_CHG_GPIO_LIST)
            self.set_gpio(cell, 'OFF', EN_DIS_GPIO_LIST)
            self.set_gpio(cell, 'LOW', EN_CUR_GPIO_LIST)
