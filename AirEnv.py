# coding=utf-8
from smbus2 import SMBus
import serial
import time


class AirEnv(object):
    """docstring for AirEnv."""

    BUS_NUMBER = 1
    I2C_ADDRESS = 0x76

    osrs_t = 1          # Temperature oversampling x 1
    osrs_p = 1          # Pressure oversampling x 1
    osrs_h = 1          # Humidity oversampling x 1
    mode = 3            # Normal mode
    t_sb = 5            # Tstandby 1000ms
    filter = 0          # Filter off
    spi3w_en = 0

    digT = []
    digP = []
    digH = []

    def __init__(self):
        self.bus = SMBus(self.BUS_NUMBER)
        self.setup()
        self.get_calib_param()

    def writeReg(self, reg_address, data):
        self.bus.write_byte_data(self.I2C_ADDRESS, reg_address, data)

    def setup(self):
        ctrl_meas_reg = (self.osrs_t << 5) | (self.osrs_p << 2) | self.mode
        config_reg = (self.t_sb << 5) | (self.filter << 2) | self.spi3w_en
        ctrl_hum_reg = self.osrs_h

        self.writeReg(0xF2, ctrl_hum_reg)
        self.writeReg(0xF4, ctrl_meas_reg)
        self.writeReg(0xF5, config_reg)

    def get_calib_param(self):
        calib = []

        for i in range(0x88, 0x88+24):
            calib.append(self.bus.read_byte_data(self.I2C_ADDRESS, i))
        calib.append(self.bus.read_byte_data(self.I2C_ADDRESS, 0xA1))
        for i in range(0xE1, 0xE1+7):
            calib.append(self.bus.read_byte_data(self.I2C_ADDRESS, i))

        for i in range(0, 5, 2):
            self.digT.append((calib[1+i] << 8) | calib[i])
        for i in range(0, 17, 2):
            self.digP.append((calib[7+i] << 8) | calib[6+i])
        self.digH.append(calib[24])
        self.digH.append((calib[26] << 8) | calib[25])
        self.digH.append(calib[27])
        self.digH.append((calib[28] << 4) | (0x0F & calib[29]))
        self.digH.append((calib[30] << 4) | ((calib[29] >> 4) & 0x0F))
        self.digH.append(calib[31])

        for i in range(1, 2):
            if self.digT[i] & 0x8000:
                self.digT[i] = (-self.digT[i] ^ 0xFFFF) + 1
        for i in range(1, 8):
            if self.digP[i] & 0x8000:
                self.digP[i] = (-self.digP[i] ^ 0xFFFF) + 1
        for i in range(0, 6):
            if self.digH[i] & 0x8000:
                self.digH[i] = (-self.digH[i] ^ 0xFFFF) + 1

    def mh_z19(self):
        ser = serial.Serial('/dev/ttyAMA0',
                          baudrate=9600,
                          bytesize=serial.EIGHTBITS,
                          parity=serial.PARITY_NONE,
                          stopbits=serial.STOPBITS_ONE,
                          timeout=1.0)
        # while 1:
        result = ser.write(b"\xff\x01\x86\x00\x00\x00\x00\x00\x79")
        s = ser.read(result)
        ser.close()
        if bytes(s[0]) == b"\xff" and bytes(s[1]) == b"\x86":
            co2 = int(s[2].encode('hex'), 16) * 256 + int(s[3].encode('hex'), 16)
            return {'co2': co2}

    def readData(self):
        data = []
        t_fine = 0.0
        pressure = 0.0

        for i in range(0xF7, 0xF7+8):
            data.append(self.bus.read_byte_data(self.I2C_ADDRESS, i))
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw = (data[6] << 8) | data[7]

        v1 = (temp_raw / 16384.0 - self.digT[0] / 1024.0) * self.digT[1]
        v2 = (temp_raw / 131072.0 - self.digT[0] / 8192.0) * (temp_raw / 131072.0 - self.digT[0] / 8192.0) * self.digT[2]
        t_fine = v1 + v2
        temperature = t_fine / 5120.0

        v1 = (t_fine / 2.0) - 64000.0
        v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * self.digP[5]
        v2 = v2 + ((v1 * self.digP[4]) * 2.0)
        v2 = (v2 / 4.0) + (self.digP[3] * 65536.0)
        v1 = (((self.digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((self.digP[1] * v1) / 2.0)) / 262144
        v1 = ((32768 + v1) * self.digP[0]) / 32768

        if v1 == 0:
            return 0
        pressure = ((1048576 - pres_raw) - (v2 / 4096)) * 3125
        if pressure < 0x80000000:
            pressure = (pressure * 2.0) / v1
        else:
            pressure = (pressure / v1) * 2
        v1 = (self.digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
        v2 = ((pressure / 4.0) * self.digP[7]) / 8192.0
        pressure = pressure + ((v1 + v2 + self.digP[6]) / 16.0)

        # print "pressure : %7.2f hPa" % (pressure/100)
        var_h = t_fine - 76800.0
        if var_h != 0:
            var_h = (hum_raw - (self.digH[3] * 64.0 + self.digH[4]/16384.0 * var_h)) * (self.digH[1] / 65536.0 * (1.0 + self.digH[5] / 67108864.0 * var_h * (1.0 + self.digH[2] / 67108864.0 * var_h)))
        else:
            return 0
        var_h = var_h * (1.0 - self.digH[0] * var_h / 524288.0)
        if var_h > 100.0:
            var_h = 100.0
        elif var_h < 0.0:
            var_h = 0.0
        return {
            "temperature": temperature,
            "pressure": pressure/100,
            "humidity": var_h
        }


if __name__ == '__main__':
    air = AirEnv()
    try:
        print(air.readData())
        print(air.mh_z19())
    except KeyboardInterrupt:
        pass
