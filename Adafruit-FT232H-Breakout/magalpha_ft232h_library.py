import Adafruit_GPIO as GPIO
import Adafruit_GPIO.FT232H as FT232H
import time

class MagAlpha:
    """"MagAlpha Communication Library for 3rd generation sensors (MA302, MA800, ...)."""

    def __init__(self):
        FT232H.use_FT232H()
        self.ft232h = FT232H.FT232H()
        self.spiClockFreqInHz = 1000000
        self.chipSelectPinOnFt232h = 8
        self.spiMode = 0
        self.spi = FT232H.SPI(self.ft232h, cs=self.chipSelectPinOnFt232h, max_speed_hz=self.spiClockFreqInHz, mode=self.spiMode, bitorder=FT232H.MSBFIRST)
        self.__rawToDegreeConvertionRatio = 360.0/65536.0

    def readAngle(self, printEnabled=False):
        """Return the angle [0-65535] (raw sensor output value)."""
        response = self.spi.transfer([0x00, 0x00])
        angularPositionRaw = (response[0]<<8)+response[1]
        if (printEnabled):
            print 'Angular Position [raw] : {0}'.format(angularPositionRaw)
        return angularPositionRaw

    def readAngleInDegree(self, printEnabled=False):
        """Return the angle in degree [0-360] (raw sensor output converted in degree)."""
        response = self.spi.transfer([0x00, 0x00])
        angularPositionRaw = (response[0]<<8)+response[1]
        angularPositionDegree = float(angularPositionRaw)*self.__rawToDegreeConvertionRatio
        if (printEnabled):
            print 'Angular Position [degree] : {0}'.format(angularPositionDegree)
        return angularPositionDegree

    def readAngleAdvanced(self, printEnabled=False):
        """Return the angle in raw and degree format."""
        response = self.spi.transfer([0x00, 0x00])
        angularPositionRaw = (response[0]<<8)+response[1]
        angularPositionDegree = float(angularPositionRaw)*self.__rawToDegreeConvertionRatio
        if (printEnabled):
            print 'Angular Position [raw] : {0} \t, [degree] : {1}'.format(angularPositionRaw, angularPositionDegree)
        return angularPositionRaw, angularPositionDegree

    def readRegister(self, address, printEnabled=False):
        """Return sensor register value."""
        command = 0b01000000 | (address & 0x1F)
        self.spi.transfer([command, 0x00])
        response = self.spi.transfer([0x00, 0x00])
        registerValue = response[0]
        if (printEnabled):
            print 'Read Register [{0}] \t=\t{1}'.format(address, registerValue)
        return registerValue

    def writeRegister(self, address, value,  printEnabled=False):
        """Return sensor written register value."""
        command = 0b10000000 | (address & 0x1F)
        registerWriteValue = (value & 0xFF)
        self.spi.transfer([command, registerWriteValue])
        #wait for 20ms
        time.sleep(0.02)
        response = self.spi.transfer([0x00, 0x00])
        registerReadValue = response[0]
        if (printEnabled):
            print 'Write Register [{0}] \t=\t{1}, \tReadback Value = {2}'.format(address, registerWriteValue, registerReadValue)
        return registerReadValue

if __name__ == "__main__":
    magAlpha = MagAlpha()
    for i in range(0, 10):
        magAlpha.readRegister(i, True)
    magAlpha.readRegister(0, True)
    magAlpha.readRegister(1, True)
    magAlpha.readAngle(True)
    magAlpha.readAngleInDegree(True)
    magAlpha.writeRegister(0, 0xFF, True)
    magAlpha.writeRegister(1, 0x7F, True)
    rawAngle = magAlpha.readAngle(True)
    degreeAngle = magAlpha.readAngleInDegree(True)
    print 'Test handle return value for readAngle and  readAngleInDegree [raw] : {0} \t, [degree] : {1}'.format(rawAngle, degreeAngle)
    rawAngle, degreeAngle = magAlpha.readAngleAdvanced(True)
    print 'Test handle return value for readAngleAdvanced [raw] : {0} \t, [degree] : {1}'.format(rawAngle, degreeAngle)
    for i in range(0, 20):
        magAlpha.readAngle(True)
        time.sleep(0.1)
