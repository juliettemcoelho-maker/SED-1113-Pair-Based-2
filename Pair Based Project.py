from machine import Pin, PWM, I2C, UART
import time

# === CONFIG ===
DUTY_DEFAULT = 32768
DEBOUNCE = 0.2

# I2C / ADS1015
I2C_SDA = 14
I2C_SCL = 15
ADS1015_ADDR = 0x48
ADS1015_PWM = 2        # ADC channel for RC-filtered PWM

# === HARDWARE SETUP ===
pwm = PWM(Pin(16))
pwm.freq(1000)
pwm.duty_u16(32768)

button = Pin(13, Pin.IN, Pin.PULL_DOWN)
uart = UART(1, 115200, tx=Pin(8), rx=Pin(9))

# I2C + ADS1015 setup
i2c = I2C(1, sda=Pin(14), scl=Pin(15))
from ads1x15 import ADS1015  # Make sure ads1x15.py is uploaded
adc = ADS1015(i2c, ADS1015_ADDR)

# Scan for I2C devices (optional debugging)
for addr in i2c.scan():
    print("I2C device found at:", hex(addr))

# === HELPER FUNCTIONS ===
def send_pwm_value(value):
    uart.write(f"{value}\n")

def receive_pwm_value():
    if uart.any():
        line = uart.readline()
        if line:
            try:
                return int(line.strip())
            except:
                return None
    return None

def read_filtered_pwm():
    """Read the PWM value via ADS1015."""
    return adc.read(0, ADS1015_PWM)

def button_pressed():
    if button.value():
        time.sleep(DEBOUNCE)
        return True
    return False

# === MAIN LOOP ===
while True:
    if button_pressed():
        # Send desired PWM
        send_pwm_value(DUTY_DEFAULT)

        # Wait to receive partner's measured PWM
        partner_value = None
        while partner_value is None:
            partner_value = receive_pwm_value()

        # Read filtered PWM via ADS1015
        measured_value = read_filtered_pwm()

        # Send measured value back
        send_pwm_value(measured_value)

        # Compute difference and print
        difference = abs(DUTY_DEFAULT - measured_value)
        print(f"Desired PWM: {DUTY_DEFAULT}, Measured: {measured_value}, Difference: {difference}")

        time.sleep(0.5)

