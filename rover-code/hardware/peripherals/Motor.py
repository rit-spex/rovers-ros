import time

from hardware.peripherals.GPIO_Peripheral import GPIO_Peripheral


class Motor:
    def __init__(self, pwm_pin=None):
        self._pwm_pin = pwm_pin
        self.duty_cycle = 0.0

    def select_pwm_pin(self, pin_number):
        self._pwm_pin = GPIO_Peripheral(pin_number, "out")

    def update_pwm(self):
        current_time = time.monotonic()
        if current_time >= self.next_toggle_time:
            if GPIO_Peripheral.input(self._pwm_pin) == GPIO_Peripheral.HIGH:
                off_time = 1 / self.frequency * (1 - (self.duty_cycle / 100))
                self.next_toggle_time = current_time + off_time
                GPIO_Peripheral.output(self._pwm_pin, GPIO_Peripheral.LOW)
            else:
                on_time = 1 / self.frequency * (self.duty_cycle / 100)
                self.next_toggle_time = current_time + on_time
                GPIO_Peripheral.output(self._pwm_pin, GPIO_Peripheral.HIGH)
