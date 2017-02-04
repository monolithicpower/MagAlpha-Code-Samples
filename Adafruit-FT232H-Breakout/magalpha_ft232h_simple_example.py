from magalpha_ft232h_library import MagAlpha
import time

# Create an instance of the MagAlpha Class
magAlpha = MagAlpha()

#Example of register settings (some settings may not be available on every sensor model)
#Set zero setting to 0 (Reg 0 and 1)
magAlpha.writeRegister(0, 0)
magAlpha.writeRegister(1, 0)
#Set Rotation Direction to Clockwise by writting 0 to register 9
magAlpha.writeRegister(9, 0)

#Read raw angle value until the user press on Ctr+C (Keyboard interrupt) to exit the program
while True:
    angle = magAlpha.readAngle(True)
    time.sleep(0.1)
