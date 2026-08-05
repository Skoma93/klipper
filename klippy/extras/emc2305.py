import logging
import pins
from . import bus

EMC2305_I2C_ADDR = 0x4D

EMC2305_PULL_IO_VALUE = 31
EMC2305_SMBUS_DISABLE_VALUE = 192

EMC2305_BASE_FREQUENCIES = (
    (26000., 0),
    (19531., 1),
    (4882., 2),
    (2441., 3),
)

EMC2305_REGS = {
    'REG_FAN_PWM_OUTPUT': 0x2b,
    'REG_FAN_CONFIG': 0x20,

    'REG_PWM_BASE45': 0x2c,
    'REG_PWM_BASE123': 0x2d,

    'EMC2305_FAN0_SETTING_REG': 0x30,
    'EMC2305_FAN1_SETTING_REG': 0x40,
    'EMC2305_FAN2_SETTING_REG': 0x50,
    'EMC2305_FAN3_SETTING_REG': 0x60,
    'EMC2305_FAN4_SETTING_REG': 0x70,
    'EMC2305_FAN0_DIVIDE_REG': 0x31,
    'EMC2305_FAN1_DIVIDE_REG': 0x41,
    'EMC2305_FAN2_DIVIDE_REG': 0x51,
    'EMC2305_FAN3_DIVIDE_REG': 0x61,
    'EMC2305_FAN4_DIVIDE_REG': 0x71,
}


def _select_pwm_frequency(requested_hz):
    candidates = []
    for base_hz, base_code in EMC2305_BASE_FREQUENCIES:
        divider = max(1, min(255, int(round(base_hz / requested_hz))))
        actual_hz = base_hz / divider
        candidates.append((abs(actual_hz - requested_hz), base_code,
                           divider, actual_hz))
    _error, base_code, divider, actual_hz = min(candidates)
    return base_code, divider, actual_hz


class EMC2305:
    def __init__(self, config):
        self._printer = config.get_printer()

        name_parts = config.get_name().split()
        self._name = name_parts[1] if len(name_parts) > 1 else "default"

        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=EMC2305_I2C_ADDR, default_speed=100000,
            async_write_only=True
        )

        self._ppins = self._printer.lookup_object("pins")
        self._ppins.register_chip("emc2305_" + self._name, self)

        self.chip_registers = EMC2305_REGS
        self._last_pwm = [None] * 5
        self._frequencies = [None] * 5

        self._printer.register_event_handler("klippy:connect", self._handle_connect)

    def _handle_connect(self):
        self.write_register('REG_FAN_PWM_OUTPUT', EMC2305_PULL_IO_VALUE)
        self.write_register('REG_FAN_CONFIG', EMC2305_SMBUS_DISABLE_VALUE)

        base_codes = []
        for channel in range(5):
            frequency = self._frequencies[channel]
            if frequency is None:
                frequency = _select_pwm_frequency(26000.)
            base_code, divider, actual_hz = frequency
            base_codes.append(base_code)
            self.write_register('EMC2305_FAN%d_DIVIDE_REG' % channel,
                                divider)
            logging.info('EMC2305 %s PWM%d frequency %.3fHz',
                         self._name, channel + 1, actual_hz)
        self.write_register('REG_PWM_BASE123',
                            base_codes[0] | base_codes[1] << 2
                            | base_codes[2] << 4)
        self.write_register('REG_PWM_BASE45',
                            base_codes[3] | base_codes[4] << 2)

        # Force all channels to off on connect
        for i in range(5):
            self.set_fan(i, 0)

    def setup_pin(self, pin_type, pin_params):
        if pin_type == 'pwm' and pin_params['pin'].startswith("PIN_"):
            return EMC2305_pwm(self, pin_params)
        raise pins.error("Wrong pin or incompatible type: %s with type %s!" % (
            pin_params['pin'], pin_type))

    def get_mcu(self):
        return self.i2c.get_mcu()

    def set_frequency(self, fan_index, cycle_time):
        requested_hz = 1. / cycle_time
        frequency = _select_pwm_frequency(requested_hz)
        previous = self._frequencies[fan_index]
        if previous is not None and previous[:2] != frequency[:2]:
            raise pins.error('Conflicting EMC2305 PWM%d frequencies'
                             % (fan_index + 1,))
        self._frequencies[fan_index] = frequency

    def write_register(self, reg_name, data):
        if not isinstance(data, (list, tuple)):
            data = [data]
        reg = self.chip_registers[reg_name]
        payload = [reg] + [(int(x) & 0xFF) for x in data]
        self.i2c.i2c_write(payload)

    def set_fan(self, fan_index, value):
        if fan_index < 0 or fan_index > 4:
            raise pins.error("EMC2305 fan index out of range: %d" % (fan_index,))
        val = int(value) & 0xFF

        if self._last_pwm[fan_index] == val:
            return
        self._last_pwm[fan_index] = val

        self.write_register('EMC2305_FAN%d_SETTING_REG' % fan_index, val)


class EMC2305_pwm:
    def __init__(self, emc2305, pin_params):
        self._emc2305 = emc2305
        self._mcu = emc2305.get_mcu()
        self._emcpin = int(pin_params['pin'].split('_')[1])
        self._invert = bool(pin_params.get('invert', False))

        self._start_value = float(self._invert)
        self._shutdown_value = float(self._invert)
        self._is_static = False
        self._cycle_time = 0.

    def get_mcu(self):
        return self._mcu

    def setup_max_duration(self, max_duration):
        # If a heater tries to use it, fail explicitly
        if max_duration:
            raise pins.error("EMC2305 pins are not suitable for heaters")

    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        self._cycle_time = cycle_time
        self._emc2305.set_frequency(self._emcpin, cycle_time)

    def setup_start_value(self, start_value, shutdown_value, is_static=False):
        if is_static and start_value != shutdown_value:
            raise pins.error("Static pin can not have shutdown value")

        if self._invert:
            start_value = 1.0 - start_value
            shutdown_value = 1.0 - shutdown_value

        self._start_value = max(0.0, min(1.0, float(start_value)))
        self._shutdown_value = max(0.0, min(1.0, float(shutdown_value)))
        self._is_static = bool(is_static)

    def set_pwm(self, print_time, value, cycle_time=None):
        v = max(0.0, min(1.0, float(value)))
        send_val = int(round(v * 255.0))

        if self._invert:
            send_val = 255 - send_val

        self._emc2305.set_fan(self._emcpin, send_val)


def load_config_prefix(config):
    return EMC2305(config)
