import time
import board
import adafruit_bmp3xx

# Initialize I2C and BMP390X sensor
i2c = board.I2C()
bmp = adafruit_bmp3xx.BMP3XX_I2C(i2c)

# Set the baseline pressure (adjust this based on your known sea level pressure)
SEA_LEVEL_PRESSURE = 1013.25  # hPa, adjust as needed for your location

# Rolling list to store the last 5 altitude readings
altitude_readings = []
false_flag = 0
# Function to calculate altitude


def set_ground():
    print("Setting ground zero pressure in 5 seconds")
    time.sleep(5)
    ground_pressure = bmp.pressure
    return ground_pressure


def get_height(ground_pressure):
    pressure = bmp.pressure  # hPa
    altitude = round(44330 * (1.0 - (pressure / ground_pressure) ** (1/5.255)),2)
    
    return altitude
    




