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
REG_CONFIG = 0x06
REG_RESOLUTION_X = 0x0d
REG_RESOLUTION_Y = 0x0e
REG_DELTA_XY_HIGH = 0x12
PRODUCT_ID1 = 0x31
PRODUCT_ID2 = 0x90
MOTION_DETECTED = 0x80
SAMPLE_TIME = .100
MIN_DETECTION_LENGTH = .2
MAX_DETECTION_LENGTH = 20.
MIN_FLOW_PERCENTAGE = 1.
MAX_FLOW_PERCENTAGE = 100.


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
        self.x_resolution = config.getint(
            'x_resolution', 240, minval=0, maxval=255)
        self.y_resolution = config.getint(
            'y_resolution', 240, minval=0, maxval=255)
        self.detection_length = config.getfloat(
            'detection_length', 4., minval=MIN_DETECTION_LENGTH,
            maxval=MAX_DETECTION_LENGTH)
        self.minimum_motion = config.getfloat(
            'minimum_motion', .1, above=0.)
        self.minimum_flow = config.getfloat(
            'minimum_flow', 70., minval=MIN_FLOW_PERCENTAGE,
            maxval=MAX_FLOW_PERCENTAGE)
        self.runout_helper = filament_switch_sensor.RunoutHelper(config)
        self.name = config.get_name().split()[-1]
        self.configfile = self.printer.lookup_object('configfile')
        self.calibration_start = None
        self.product_id = None
        self.extruder = self.estimated_print_time = None
        self.pause_resume = None
        self.was_paused = False
        self.pending_motion = self.unmatched_extrusion = 0.
        self.window_extrusion = self.window_motion = 0.
        self.flow_percentage = 100.
        self.motion_direction = 0
        self.extruder_high_water = None
        self.x_counts = self.y_counts = 0
        self.sample_timer = self.reactor.register_timer(self._sample_sensor)
        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command(
            'FMS_SENSOR_CALIBRATE', 'SENSOR', self.name,
            self.cmd_FMS_SENSOR_CALIBRATE,
            desc='Calibrate PAT9125 counts per millimetre')
        self.gcode.register_mux_command(
            'SET_FMS_SENSITIVITY', 'SENSOR', self.name,
            self.cmd_SET_FMS_SENSITIVITY,
            desc='Set PAT9125 flow threshold and evaluation window')

    def _read_reg(self, reg, length=1):
        params = self.i2c.i2c_read([reg], length)
        return bytearray(params['response'])

    def _write_reg(self, reg, value):
        self.i2c.i2c_write([reg, value])

    def _handle_connect(self):
        product_id1 = self._read_reg(REG_PRODUCT_ID1)[0]
        product_id2 = self._read_reg(REG_PRODUCT_ID2)[0]
        if (product_id1 != PRODUCT_ID1
                or product_id2 & 0xf0 != PRODUCT_ID2):
            raise self.printer.command_error(
                'Invalid PAT9125 id (got %02x:%02x vs 31:9x). This is '
                'generally indicative of connection problems, an incorrect '
                'I2C address, or a faulty chip.'
                % (product_id1, product_id2))
        self._write_reg(REG_CONFIG, 0x97)
        self.reactor.pause(self.reactor.monotonic() + .001)
        self._write_reg(REG_CONFIG, 0x17)
        self._write_reg(REG_RESOLUTION_X, self.x_resolution)
        self._write_reg(REG_RESOLUTION_Y, self.y_resolution)
        actual_x = self._read_reg(REG_RESOLUTION_X)[0]
        actual_y = self._read_reg(REG_RESOLUTION_Y)[0]
        if (actual_x != self.x_resolution
                or actual_y != self.y_resolution):
            raise self.printer.command_error(
                'Unable to set PAT9125 resolution (requested %d:%d, got '
                '%d:%d)' % (self.x_resolution, self.y_resolution,
                             actual_x, actual_y))
        self.product_id = (product_id1, product_id2)
        self.extruder = self.printer.lookup_object(self.extruder_name)
        self.pause_resume = self.printer.lookup_object('pause_resume', None)
        self.estimated_print_time = (
            self.printer.lookup_object('mcu').estimated_print_time)
        self.extruder_high_water = self._get_extruder_pos(
            self.reactor.monotonic())
        self.runout_helper.note_filament_present(
            self.reactor.monotonic(), True)
        self.reactor.update_timer(self.sample_timer, self.reactor.NOW)

    def reset_watchdog(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        if self.extruder is not None:
            self.extruder_high_water = self._get_extruder_pos(eventtime)
        self.pending_motion = self.unmatched_extrusion = 0.
        self.window_extrusion = self.window_motion = 0.
        self.flow_percentage = 100.
        self.runout_helper.note_filament_present(eventtime, True)

    def get_motion_count(self):
        return (self.x_counts, self.y_counts)[self.axis]

    def _get_extruder_pos(self, eventtime):
        print_time = self.estimated_print_time(eventtime)
        return self.extruder.find_past_position(print_time)

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
        is_paused = (self.pause_resume is not None
                     and self.pause_resume.is_paused)
        if is_paused:
            self.reset_watchdog(eventtime)
            self.was_paused = True
            return eventtime + SAMPLE_TIME
        if self.was_paused:
            self.reset_watchdog(eventtime)
            self.was_paused = False
            return eventtime + SAMPLE_TIME
        extruder_pos = self._get_extruder_pos(eventtime)
        forward = max(0., extruder_pos - self.extruder_high_water)
        if forward:
            self.extruder_high_water = extruder_pos
        axis_motion = (delta_x, delta_y)[self.axis] / self.counts_per_mm
        directed_motion = 0.
        if forward:
            if self.motion_direction:
                directed_motion = max(
                    0., axis_motion * self.motion_direction)
            else:
                self.pending_motion += axis_motion
                if abs(self.pending_motion) >= self.minimum_motion:
                    self.motion_direction = (
                        1 if self.pending_motion > 0. else -1)
                    directed_motion = abs(self.pending_motion)
                    self.pending_motion = 0.
        self.window_extrusion += forward
        if forward:
            self.window_motion += directed_motion
        self.unmatched_extrusion = max(
            0., self.window_extrusion - self.window_motion)
        if self.window_extrusion >= self.detection_length:
            self.flow_percentage = min(
                100., 100. * self.window_motion / self.window_extrusion)
            is_present = self.flow_percentage >= self.minimum_flow
            self.window_extrusion = self.window_motion = 0.
            self.pending_motion = self.unmatched_extrusion = 0.
            self.runout_helper.note_filament_present(
                eventtime, is_present)
        return eventtime + SAMPLE_TIME

    def get_status(self, eventtime):
        status = self.runout_helper.get_status(eventtime)
        status.update({
            'x_counts': self.x_counts,
            'y_counts': self.y_counts,
            'motion': (self.x_counts, self.y_counts)[self.axis]
                      / self.counts_per_mm,
            'counts_per_mm': self.counts_per_mm,
            'detection_length': self.detection_length,
            'minimum_motion': self.minimum_motion,
            'minimum_flow': self.minimum_flow,
            'flow_percentage': self.flow_percentage,
            'window_extrusion': self.window_extrusion,
            'window_motion': self.window_motion,
            'unmatched_extrusion': self.unmatched_extrusion,
            'motion_direction': self.motion_direction,
            'calibrating': self.calibration_start is not None,
            'product_id': self.product_id,
            'x_resolution': self.x_resolution,
            'y_resolution': self.y_resolution,
        })
        return status

    def cmd_SET_FMS_SENSITIVITY(self, gcmd):
        flow = gcmd.get_float(
            'FLOW', None, minval=MIN_FLOW_PERCENTAGE,
            maxval=MAX_FLOW_PERCENTAGE)
        distance = gcmd.get_float(
            'DISTANCE', None, minval=MIN_DETECTION_LENGTH,
            maxval=MAX_DETECTION_LENGTH)
        if flow is None and distance is None:
            raise gcmd.error('Specify FLOW and/or DISTANCE')
        if flow is not None:
            self.minimum_flow = flow
        if distance is not None:
            self.detection_length = distance
        self.reset_watchdog()
        gcmd.respond_info(
            'PAT9125 %s minimum flow %.1f%% over a %.1fmm window'
            % (self.name, self.minimum_flow, self.detection_length))

    def cmd_FMS_SENSOR_CALIBRATE(self, gcmd):
        action = gcmd.get('ACTION').upper()
        counts = (self.x_counts, self.y_counts)[self.axis]
        if action == 'QUERY':
            product_id = ('unavailable' if self.product_id is None else
                          '%02x:%02x' % self.product_id)
            gcmd.respond_info(
                'PAT9125 %s id=%s, x=%d, y=%d, selected=%d counts, '
                'scale=%.6f counts/mm, resolution=%d:%d, calibrating=%s'
                % (self.name, product_id, self.x_counts, self.y_counts,
                   counts, self.counts_per_mm,
                   self.x_resolution, self.y_resolution,
                   'yes' if self.calibration_start is not None else 'no'))
            return
        if action == 'START':
            self.calibration_start = counts
            gcmd.respond_info(
                'PAT9125 %s calibration started at %d counts; move a known '
                'filament length, then run ACTION=FINISH LENGTH=<mm>'
                % (self.name, counts))
            return
        if action != 'FINISH':
            raise gcmd.error('ACTION must be START, FINISH, or QUERY')
        if self.calibration_start is None:
            raise gcmd.error('Run FMS_SENSOR_CALIBRATE ACTION=START first')
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
