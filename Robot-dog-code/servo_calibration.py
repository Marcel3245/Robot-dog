import board
import time
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# Set up PCA9685
i2c = board.I2C()
pca = PCA9685(i2c)
pca.frequency = 50


currentPosition_x = 0.00
servo_x = servo.Servo(pca.channels[3], actuation_range=180,  min_pulse=500, max_pulse=2500)
servo_x.angle = currentPosition_x
time.sleep(.1)
