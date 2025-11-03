from machine import Pin, PWM, I2C, UART
import time

# === CONFIG ===
PWM_PIN = 16
BUTTON_PIN = 13
UART_TX = 8
UART_RX = 9

DUTY_DEFAULT = 32768
DEBOUNCE = 0.2

# I2C / ADS1015
I2C_SDA = 14
I2C_SCL = 15
ADS1015_ADDR = 0x48
ADS1015_PWM = 1   # AIN1 = correct channel

# === HARDWARE SETUP ===
pwm = PWM(Pin(PWM_PIN))
pwm.freq(1000)
pwm.duty_u16(DUTY_DEFAULT)

button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_DOWN)
uart = UART(1, 115200, tx=Pin(UART_TX), rx=Pin(UART_RX))

# I2C + ADS1015 setup
i2c = I2C(1, sda=Pin(I2C_SDA), scl=Pin(I2C_SCL))
from ads1x15 import ADS1015
adc = ADS1015(i2c, ADS1015_ADDR)

# === FUNCTIONS ===
def send_value(value):
    uart.write(str(value) + "\n")

def receive_value():
    if uart.any():
        try:
            return int(uart.readline().strip())
        except:
            return None
    return None

def read_filtered_pwm():
    return adc.read(0, ADS1015_PWM)

def button_pressed():
    if button.value():
        time.sleep(DEBOUNCE)
        return True
    return False

# === MAIN LOOP ===
while True:
    if button_pressed():

        # Send desired PWM duty to partner
        send_value(DUTY_DEFAULT)

        # Wait for partner's measurement
        partner_measure = None
        while partner_measure is None:
            partner_measure = receive_value()

        # Measure our PWM
        my_measure = read_filtered_pwm()

        # Send our measurement back to partner
        send_value(my_measure)

        # Compute difference
        difference = abs(DUTY_DEFAULT - my_measure)

        print("\n=== RESULT ===")
        print("Partner measured:", partner_measure)
        print("You measured:    ", my_measure)
        print("Difference:      ", difference)

        time.sleep(0.5)
