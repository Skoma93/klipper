# - PCA9685 PWM frequency is global for the whole chip (all channels).
# - This is I2C-based; suitable for fans/LEDs, not hard real-time waveforms.

import logging
import pins
from . import bus

MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06

RESTART = 0x80
SLEEP = 0x10
AI = 0x20      # auto-increment enable
OUTDRV = 0x04


def _clamp01(v: float) -> float:
    if v < 0.0:
        return 0.0
    if v > 1.0:
        return 1.0
    return v


class PCA9685Pin:
    def __init__(self, chip, channel: int, pin_type: str, pin_params: dict):
        if pin_type not in ("pwm", "digital_out"):
            raise pins.error("pca9685_pwm supports only: pwm, digital_out")
        if not (0 <= channel <= 15):
            raise pins.error("pca9685_pwm channel must be 0..15")

        self._chip = chip
        self._channel = channel
        self._invert = bool(pin_params.get("invert", False))

        self._start_value = 1.0 if self._invert else 0.0
        self._shutdown_value = 1.0 if self._invert else 0.0

        self._chip._register_output(self)

    def get_mcu(self):
        return self._chip.i2c.get_mcu()

    def setup_max_duration(self, max_duration):
        pass

    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        if hardware_pwm:
            raise pins.error("pca9685_pwm does not support hardware_pwm")

    def setup_start_value(self, start_value, shutdown_value):
        sv = float(start_value)
        shv = float(shutdown_value)
        if self._invert:
            sv = 1.0 - sv
            shv = 1.0 - shv
        self._start_value = _clamp01(sv)
        self._shutdown_value = _clamp01(shv)
        self._chip._set_channel(self._channel, self._start_value)

    def set_pwm(self, print_time, value):
        v = float(value)
        if self._invert:
            v = 1.0 - v
        self._chip._set_channel(self._channel, _clamp01(v))

    def set_digital(self, print_time, value):
        self.set_pwm(print_time, 1.0 if value else 0.0)

    def _apply_start(self):
        self._chip._set_channel(self._channel, self._start_value)

    def _apply_shutdown(self):
        self._chip._set_channel(self._channel, self._shutdown_value)


class PCA9685Chip:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]

        ppins = self.printer.lookup_object("pins")
        ppins.register_chip(self.name, self)

        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=64, default_speed=100000
        )

        self.freq_hz = config.getfloat("frequency", 250.0, above=1.0)
        self.min_interval = config.getfloat("min_interval", 0.0, minval=0.0)

        self._outputs = []
        self._cache = [None] * 16
        self._last_write_time = 0.0
        self._connected = False

        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)

    def _register_output(self, out: PCA9685Pin):
        self._outputs.append(out)

    def setup_pin(self, pin_type, pin_params):
        p = pin_params.get("pin")
        try:
            channel = int(p)
        except Exception:
            raise pins.error("pca9685_pwm pin must be an integer channel 0..15")
        return PCA9685Pin(self, channel, pin_type, pin_params)

    def _i2c_write(self, reg: int, *data: int):
        payload = [reg] + [d & 0xFF for d in data]
        self.i2c.i2c_write(payload)

    def _set_frequency(self, freq_hz: float):
        # Datasheet formula with nominal 25MHz oscillator:
        # prescale = round(osc/(4096*freq)) - 1
        osc = 25_000_000.0
        prescale = int(round((osc / (4096.0 * freq_hz)) - 1.0))
        prescale = max(3, min(255, prescale))

        # Put chip to sleep, set prescale, wake, then restart + autoinc.
        self._i2c_write(MODE1, SLEEP)
        self._i2c_write(PRESCALE, prescale)
        self._i2c_write(MODE1, AI)  # wake (sleep cleared)
        self.reactor.pause(self.reactor.monotonic() + 0.005)
        self._i2c_write(MODE1, AI | RESTART)
        self._i2c_write(MODE2, OUTDRV)

    def _handle_connect(self):
        try:
            self._set_frequency(self.freq_hz)
            self._connected = True
            self._cache = [None] * 16
            for out in self._outputs:
                out._apply_start()
            logging.info("pca9685_pwm '%s' ready (addr=%d freq=%.1fHz)",
                         self.name, self.i2c.get_i2c_address(), self.freq_hz)
        except Exception as e:
            self._connected = False
            raise pins.error(f"pca9685_pwm init failed: {e}")

    def _handle_shutdown(self):
        try:
            if self._connected:
                for out in self._outputs:
                    out._apply_shutdown()
        except Exception:
            pass
        self._connected = False

    def _set_channel(self, ch: int, duty: float):
        if not self._connected:
            return

        now = self.reactor.monotonic()
        if self.min_interval > 0.0 and (now - self._last_write_time) < self.min_interval:
            return

        duty = _clamp01(duty)
        if duty <= 0.0:
            data = (0x00, 0x00, 0x00, 0x10)  # FULL_OFF
        elif duty >= 1.0:
            data = (0x00, 0x10, 0x00, 0x00)  # FULL_ON
        else:
            off = int(round(duty * 4095.0))
            on = 0
            data = (
                on & 0xFF,
                (on >> 8) & 0x0F,
                off & 0xFF,
                (off >> 8) & 0x0F,
            )

        if self._cache[ch] == data:
            return

        reg = LED0_ON_L + 4 * ch
        self._i2c_write(reg, *data)
        self._cache[ch] = data
        self._last_write_time = now


def load_config_prefix(config):
    return PCA9685Chip(config)