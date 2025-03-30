class FSW_ADDRS:
    def __init__(self):        
        self.INA_ADDRS = [0x40, 0x41, 0x43]        #INA 0,1,2
        self.TMP_ADDRS = [0x48, 0x49, 0x4B]        #TMP 0,1,2
        self.POT_ADDRS = [0x2C, 0x2E, 0x2D, 0x2F]  #POT 0,1,2,3
        self.POT_REGS  = [0x00, 0x01, 0x06, 0x07]  #
        self.EN_CHG_GPIO_LIST = [23,24,25]         #GPIO 23,24,25 is physical pins 16,18,22
        self.EN_DIS_GPIO_LIST = [ 5, 6,13]         #GPIO  5, 6,13 is physical pins 29,31,33
        self.EN_CUR_GPIO_LIST = [12,16,26]         #GPIO 12,16,26 is physical pins 32,36,37
        self.EN_HEATER_GPIO   = 18                 #GPIO 18 is physical pin 12