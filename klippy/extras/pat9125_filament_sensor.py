# PAT9125 Filament Motion Sensor Module
#
# Copyright (C) 2026  Oliver
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import bus, filament_switch_sensor

PAT9125_I2C_ADDR = 0x75
REG_PRODUCT_ID1 = 0x00
REG_PRODUCT_ID2 = 0x01
REG_MOTION_STATUS = 0x02
REG_DELTA_XY_HIGH = 0x12
PRODUCT_ID1 = 0x31
PRODUCT_ID2 = 0x90
MOTION_DETECTED = 0x80
SAMPLE_TIME = .100


def _decode_12bit(low, high):
    value = low | (high << 8)
    if value & 0x800:
        value -= 0x1000
    return value


class PAT9125FilamentSensor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.i2c = bus.MCU_I2C_from_config(
            config, default_addr=PAT9125_I2C_ADDR, default_speed=100000)
        self.extruder_name = config.get('extruder')
        self.axis = config.getchoice('axis', {'x': 0, 'y': 1}, default='y')
        self.counts_per_mm = config.getfloat('counts_per_mm', above=0.)
        self.detection_length = config.getfloat(
            'detection_length', 7., above=0.)
        self.minimum_motion = config.getfloat(
            'minimum_motion', .1, above=0.)
        self.runout_helper = filament_switch_sensor.RunoutHelper(config)
        self.name = config.get_name().split()[-1]
        self.configfile = self.printer.lookup_object('configfile')
        self.calibration_start = None
        self.extruder = self.estimated_print_time = None
        self.filament_runout_pos = None
        self.pending_motion = 0.
        self.x_counts = self.y_counts = 0
        self.sample_timer = self.reactor.register_timer(self._sample_sensor)
        self.printer.register_event_handler('klippy:ready',
                                            self._handle_ready)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command(
            'PAT9125_CALIBRATE', 'SENSOR', self.name,
            self.cmd_PAT9125_CALIBRATE,
            desc='Calibrate PAT9125 counts per millimetre')

    def _read_reg(self, reg, length=1):
        params = self.i2c.i2c_read([reg], length)
        return bytearray(params['response'])

    def _handle_ready(self):
        product_id1 = self._read_reg(REG_PRODUCT_ID1)[0]
        product_id2 = self._read_reg(REG_PRODUCT_ID2)[0]
        if (product_id1 != PRODUCT_ID1
                or product_id2 & 0xf0 != PRODUCT_ID2):
            raise self.printer.command_error(
                'Invalid PAT9125 id (got %02x:%02x vs 31:9x). This is '
                'generally indicative of connection problems, an incorrect '
                'I2C address, or a faulty chip.'
                % (product_id1, product_id2))
        self.extruder = self.printer.lookup_object(self.extruder_name)
        self.estimated_print_time = (
            self.printer.lookup_object('mcu').estimated_print_time)
        self._update_filament_runout_pos()
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def _get_extruder_pos(self, eventtime):
        print_time = self.estimated_print_time(eventtime)
        return self.extruder.find_past_position(print_time)

    def _update_filament_runout_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        self.filament_runout_pos = (
            self._get_extruder_pos(eventtime) + self.detection_length)

    def _read_motion(self):
        motion = self._read_reg(REG_MOTION_STATUS, 3)
        if not motion[0] & MOTION_DETECTED:
            return 0, 0
        high = self._read_reg(REG_DELTA_XY_HIGH)[0]
        return (_decode_12bit(motion[1], high >> 4),
                _decode_12bit(motion[2], high & 0x0f))

    def _sample_sensor(self, eventtime):
        delta_x, delta_y = self._read_motion()
        self.x_counts += delta_x
        self.y_counts += delta_y
        axis_delta = (delta_x, delta_y)[self.axis]
        self.pending_motion += abs(axis_delta) / self.counts_per_mm
        if self.pending_motion >= self.minimum_motion:
            self.pending_motion = 0.
            self._update_filament_runout_pos(eventtime)
            self.runout_helper.note_filament_present(eventtime, True)
        extruder_pos = self._get_extruder_pos(eventtime)
        self.runout_helper.note_filament_present(
            eventtime, extruder_pos < self.filament_runout_pos)
        return eventtime + SAMPLE_TIME

    def get_status(self, eventtime):
        status = self.runout_helper.get_status(eventtime)
        status.update({
            'x_counts': self.x_counts,
            'y_counts': self.y_counts,
            'motion': (self.x_counts, self.y_counts)[self.axis]
                      / self.counts_per_mm,
        })
        return status

    def cmd_PAT9125_CALIBRATE(self, gcmd):
        action = gcmd.get('ACTION').upper()
        counts = (self.x_counts, self.y_counts)[self.axis]
        if action == 'START':
            self.calibration_start = counts
            gcmd.respond_info(
                'PAT9125 %s calibration started at %d counts; move a known '
                'filament length, then run ACTION=FINISH LENGTH=<mm>'
                % (self.name, counts))
            return
        if action != 'FINISH':
            raise gcmd.error('ACTION must be START or FINISH')
        if self.calibration_start is None:
            raise gcmd.error('Run PAT9125_CALIBRATE ACTION=START first')
        length = gcmd.get_float('LENGTH', above=0.)
        delta = abs(counts - self.calibration_start)
        if not delta:
            raise gcmd.error('PAT9125 measured no motion')
        value = delta / length
        self.counts_per_mm = value
        self.calibration_start = None
        self.configfile.set(
            'pat9125_filament_sensor ' + self.name,
            'counts_per_mm', '%.6f' % value)
        gcmd.respond_info(
            'PAT9125 %s measured %d counts over %.3fmm: %.6f counts/mm. '
            'Run SAVE_CONFIG to store it.'
            % (self.name, delta, length, value))


def load_config_prefix(config):
    return PAT9125FilamentSensor(config)
