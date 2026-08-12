# Shared bed-presence ADC and digital probe input
#
# Copyright (C) 2026  Klipper developers
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import mcu
import pins

ADC_REPORT_TIME = 0.100
ADC_SAMPLE_TIME = 0.001
ADC_SAMPLE_COUNT = 8


class SharedBedPin:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        ppins = self.printer.lookup_object('pins')
        pin_params = ppins.lookup_pin(
            config.get('pin'), can_invert=True, can_pullup=True)
        self.mcu = pin_params['chip']
        self.pin = pin_params['pin']
        self.pullup = pin_params['pullup']
        self.invert = pin_params['invert']
        self.oid = self.mcu.create_oid()
        self.dispatch = mcu.TriggerDispatch(self.mcu)
        self.mcu.register_config_callback(self._build_config)
        self.home_cmd = self.query_cmd = None
        self.rest_ticks = 0
        self.adc_max = 0.
        self.adc_voltage = config.getfloat(
            'adc_voltage', 3.3, above=0.)
        self.present_min_voltage = config.getfloat(
            'present_min_voltage', minval=0., maxval=self.adc_voltage)
        self.present_max_voltage = config.getfloat(
            'present_max_voltage', minval=self.present_min_voltage,
            maxval=self.adc_voltage)
        self.last_sample_time = 0.
        self.voltage = None
        self.present = None
        self.state = 'unknown'
        self.monitoring = True
        self.digital_mode = False
        self.pause_requested = False
        self.printing = False
        self.enabled = config.getboolean('enabled', True)
        self.gcode = self.printer.lookup_object('gcode')
        self.printer.load_object(config, 'pause_resume')
        self.printer.load_object(config, 'idle_timeout')
        self.printer.register_event_handler('idle_timeout:printing',
                                            self._handle_printing)
        self.printer.register_event_handler('idle_timeout:ready',
                                            self._handle_not_printing)
        self.printer.register_event_handler('idle_timeout:idle',
                                            self._handle_not_printing)
        self.gcode.register_command(
            'SET_BED_PRESENCE', self.cmd_SET_BED_PRESENCE,
            desc=self.cmd_SET_BED_PRESENCE_help)
        self.gcode.register_command(
            'QUERY_BED_PRESENCE', self.cmd_QUERY_BED_PRESENCE,
            desc=self.cmd_QUERY_BED_PRESENCE_help)
        self.gcode.register_command(
            'REQUIRE_BED_PRESENT', self.cmd_REQUIRE_BED_PRESENT,
            desc=self.cmd_REQUIRE_BED_PRESENT_help)
        ppins.register_chip('bed_presence', self)

    def _build_config(self):
        self.mcu.add_config_cmd(
            'config_shared_adc_endstop oid=%d pin=%s pull_up=%d'
            % (self.oid, self.pin, self.pullup))
        clock = self.mcu.get_query_slot(self.oid)
        sample_ticks = self.mcu.seconds_to_clock(ADC_SAMPLE_TIME)
        rest_ticks = self.mcu.seconds_to_clock(ADC_REPORT_TIME)
        self.mcu.add_config_cmd(
            'query_shared_adc oid=%d clock=%d sample_ticks=%d'
            ' sample_count=%d rest_ticks=%d'
            % (self.oid, clock, sample_ticks, ADC_SAMPLE_COUNT, rest_ticks),
            is_init=True)
        self.mcu.add_config_cmd(
            'shared_endstop_home oid=%d clock=0 sample_ticks=0'
            ' sample_count=0 rest_ticks=0 pin_value=0 trsync_oid=0'
            ' trigger_reason=0 pull_up=%d' % (self.oid, self.pullup),
            on_restart=True)
        queue = self.dispatch.get_command_queue()
        self.home_cmd = self.mcu.lookup_command(
            'shared_endstop_home oid=%c clock=%u sample_ticks=%u'
            ' sample_count=%c rest_ticks=%u pin_value=%c trsync_oid=%c'
            ' trigger_reason=%c pull_up=%c', cq=queue)
        self.query_cmd = self.mcu.lookup_query_command(
            'shared_endstop_query_state oid=%c pull_up=%c',
            'shared_endstop_state oid=%c homing=%c next_clock=%u'
            ' pin_value=%c', oid=self.oid, cq=queue)
        self.adc_max = self.mcu.get_constant_float('ADC_MAX')
        self.mcu.register_serial_response(
            self._handle_adc_state, 'shared_adc_state oid=%c'
            ' next_clock=%u value=%hu', self.oid)

    def _handle_printing(self, print_time):
        self.printing = True
        if self.enabled and self.monitoring and self.present is not True:
            self._request_pause()

    def _handle_not_printing(self, print_time):
        self.printing = False
        self.pause_requested = False

    def _is_printing(self, eventtime):
        return self.printing

    def _request_pause(self):
        if self.pause_requested:
            return
        self.pause_requested = True
        self.reactor.register_async_callback(self._pause_callback)

    def _pause_callback(self, eventtime):
        if (not self.enabled or not self._is_printing(eventtime)
                or self.present is True):
            self.pause_requested = False
            return
        self.gcode.run_script('PAUSE')

    def _update_presence(self, voltage):
        if voltage < self.present_min_voltage:
            self.state = 'error'
            self.present = False
        elif voltage <= self.present_max_voltage:
            self.state = 'present'
            self.present = True
        else:
            self.state = 'missing'
            self.present = False

    def _handle_adc_state(self, params):
        if self.digital_mode:
            return
        next_clock = self.mcu.clock32_to_clock64(params['next_clock'])
        self.last_sample_time = self.mcu.clock_to_print_time(
            next_clock - self.mcu.seconds_to_clock(ADC_REPORT_TIME))
        self.voltage = params['value'] * self.adc_voltage / self.adc_max
        self._update_presence(self.voltage)
        eventtime = self.reactor.monotonic()
        if self.enabled and self._is_printing(eventtime):
            if self.present is not True:
                self._request_pause()
        else:
            self.pause_requested = False

    cmd_SET_BED_PRESENCE_help = (
        'Enable or disable bed-presence print enforcement')

    def cmd_SET_BED_PRESENCE(self, gcmd):
        enabled = bool(gcmd.get_int('ENABLE', minval=0, maxval=1))
        self.enabled = enabled
        if not enabled:
            self.pause_requested = False
        elif (self.printing and self.monitoring
              and self.present is not True):
            self._request_pause()
        gcmd.respond_info(
            'Bed presence sensing %s'
            % ('enabled' if enabled else 'disabled'))

    def _state_text(self):
        return self.state

    cmd_QUERY_BED_PRESENCE_help = 'Report the current bed-presence state'

    def cmd_QUERY_BED_PRESENCE(self, gcmd):
        voltage = 'unknown' if self.voltage is None else '%.3f V' % self.voltage
        mode = 'digital' if self.digital_mode else 'adc'
        gcmd.respond_info(
            'Bed presence: %s, voltage: %s, mode: %s, enforcement: %s'
            % (self._state_text(), voltage, mode,
               'enabled' if self.enabled else 'disabled'))

    cmd_REQUIRE_BED_PRESENT_help = (
        'Abort the current G-Code script unless the bed is present')

    def cmd_REQUIRE_BED_PRESENT(self, gcmd):
        if self.digital_mode or not self.monitoring:
            raise gcmd.error(
                'Bed presence is unavailable while PC27 is in digital mode')
        if self.present is None:
            raise gcmd.error('Bed presence is unknown; no valid ADC sample')
        if self.state == 'error':
            raise gcmd.error('Bed presence input is below 2.00 V (short circuit)')
        if not self.present:
            raise gcmd.error('Bed plate is not present')

    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' or pin_params['pin'] != 'virtual_endstop':
            raise pins.error(
                'bed_presence only supports bed_presence:virtual_endstop')
        if pin_params['invert'] or pin_params['pullup']:
            raise pins.error(
                'Can not invert or pull up bed_presence:virtual_endstop')
        return self

    def get_mcu(self):
        return self.mcu

    def add_stepper(self, stepper):
        self.dispatch.add_stepper(stepper)

    def get_steppers(self):
        return self.dispatch.get_steppers()

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        clock = self.mcu.print_time_to_clock(print_time)
        self.rest_ticks = (
            self.mcu.print_time_to_clock(print_time + rest_time) - clock)
        completion = self.dispatch.start(print_time)
        self.monitoring = False
        self.digital_mode = True
        self.home_cmd.send(
            [self.oid, clock, self.mcu.seconds_to_clock(sample_time),
             sample_count, self.rest_ticks, triggered ^ self.invert,
             self.dispatch.get_oid(), mcu.MCU_trsync.REASON_ENDSTOP_HIT,
             self.pullup], reqclock=clock)
        return completion

    def _restore_adc(self):
        self.home_cmd.send(
            [self.oid, 0, 0, 0, 0, 0, 0, 0, self.pullup])
        self.digital_mode = False
        self.monitoring = True
        self.present = None
        self.state = 'unknown'

    def home_wait(self, home_end_time):
        try:
            self.dispatch.wait_end(home_end_time)
        except:
            self._restore_adc()
            self.dispatch.stop()
            raise
        self._restore_adc()
        result = self.dispatch.stop()
        if result >= mcu.MCU_trsync.REASON_COMMS_TIMEOUT:
            raise self.printer.command_error(
                'Communication timeout during shared-pin probing')
        if result != mcu.MCU_trsync.REASON_ENDSTOP_HIT:
            return 0.
        if self.mcu.is_fileoutput():
            return home_end_time
        params = self.query_cmd.send([self.oid, self.pullup])
        next_clock = self.mcu.clock32_to_clock64(params['next_clock'])
        return self.mcu.clock_to_print_time(next_clock - self.rest_ticks)

    def query_endstop(self, print_time):
        clock = self.mcu.print_time_to_clock(print_time)
        if self.mcu.is_fileoutput():
            return 0
        self.monitoring = False
        self.digital_mode = True
        try:
            params = self.query_cmd.send(
                [self.oid, self.pullup], minclock=clock)
            return params['pin_value'] ^ self.invert
        finally:
            self.digital_mode = False
            self.monitoring = True
            self.present = None
            self.state = 'unknown'

    def get_status(self, eventtime):
        return {
            'enabled': self.enabled,
            'voltage': self.voltage,
            'state': self.state,
            'present': self.present,
            'monitoring': self.monitoring,
            'digital_mode': self.digital_mode,
            'pause_requested': self.pause_requested,
            'last_sample_time': self.last_sample_time,
        }


class BedPresence:
    def __init__(self, config):
        self.shared_pin = SharedBedPin(config)

    def get_status(self, eventtime):
        return self.shared_pin.get_status(eventtime)


def load_config(config):
    return BedPresence(config)