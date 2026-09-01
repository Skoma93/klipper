# Support for protected high-side switch channels
#
# Copyright (C) 2026  Klipper developers
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import pins

class PMICPin:
    def __init__(self, channel, pin_type, pin_params):
        if pin_type not in ('digital_out', 'pwm'):
            raise pins.error("pmic pins support only digital_out and pwm")
        if pin_params['pin'] != 'OUT':
            raise pins.error("pmic virtual pin must be named OUT")
        self.channel = channel
        self.pin_type = pin_type
        self.invert = bool(pin_params.get('invert', False))
        self.mcu_pin = channel.ppins.setup_pin(
            pin_type, channel.output_pin_name)
        channel.register_output(self)
    def get_mcu(self):
        return self.mcu_pin.get_mcu()
    def setup_max_duration(self, max_duration):
        self.mcu_pin.setup_max_duration(max_duration)
    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        self.mcu_pin.setup_cycle_time(cycle_time, hardware_pwm)
    def setup_start_value(self, start_value, shutdown_value, is_static=False):
        start_value = self._logical(start_value)
        shutdown_value = self.channel.shutdown_value
        if is_static and start_value != self.channel.shutdown_value:
            raise pins.error("Static pin can not have shutdown value")
        self.channel.commanded_value = start_value
        self.mcu_pin.setup_start_value(start_value, shutdown_value)
    def _logical(self, value):
        value = max(0., min(1., float(value)))
        return 1. - value if self.invert else value
    def set_pwm(self, print_time, value, cycle_time=None):
        self.channel.set_value(print_time, self._logical(value), cycle_time)
    def set_digital(self, print_time, value):
        self.channel.set_value(print_time, self._logical(bool(value)))

class PMICChannel:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.ppins = self.printer.lookup_object('pins')
        self.output_pin_name = config.get('output_pin')
        self.shutdown_value = config.getfloat(
            'shutdown_value', 0., minval=0., maxval=1.)
        self.error_severity = config.getchoice(
            'error_severity',
            {'silent': 'silent', 'warning': 'warning', 'critical': 'critical'},
            default='warning')
        self.off_delay = config.getfloat('fault_off_delay', 0., minval=0.)
        self.retry_interval = config.getfloat(
            'retry_interval', 1., above=0.)
        self.toggle_delay = config.getfloat(
            'retry_toggle_delay', .1, minval=0.)
        if self.toggle_delay >= self.retry_interval:
            raise config.error(
                "retry_toggle_delay must be less than retry_interval")
        self.output = None
        self.commanded_value = 0.
        self.retry_value = 0.
        self.fault_active = self.fault_latched = False
        self.fault_time = self.fault_deadline = None
        self.retry_timer = None
        self.retry_count = 0
        self.ppins.register_chip('pmic_' + self.name, self)
        buttons = self.printer.load_object(config, 'buttons')
        buttons.register_buttons([config.get('fault_pin')], self._fault_event)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command(
            'CLEAR_PMIC_FAULT', 'PMIC', self.name,
            self.cmd_CLEAR_PMIC_FAULT, desc=self.cmd_CLEAR_PMIC_FAULT_help)
        self.printer.register_event_handler(
            'klippy:shutdown', self._handle_shutdown)
    def register_output(self, output):
        if self.output is not None:
            raise pins.error("pmic channel '%s' used more than once"
                             % (self.name,))
        self.output = output
    def setup_pin(self, pin_type, pin_params):
        return PMICPin(self, pin_type, pin_params)
    def _write(self, print_time, value, cycle_time=None):
        if self.output.pin_type == 'digital_out':
            self.output.mcu_pin.set_digital(print_time, bool(value))
        elif cycle_time is None:
            self.output.mcu_pin.set_pwm(print_time, value)
        else:
            self.output.mcu_pin.set_pwm(print_time, value, cycle_time)
    def _print_time(self, eventtime):
        mcu = self.output.get_mcu()
        # Button/fault callbacks carry the MCU receive timestamp.  By the time
        # the callback runs in the host thread that timestamp may be too old to
        # safely schedule an immediate output change.  Use current host time
        # when it is later so emergency-off writes do not trigger "Timer too
        # close" on the MCU.
        eventtime = max(eventtime, self.reactor.monotonic())
        return mcu.estimated_print_time(eventtime + mcu.min_schedule_time())
    def _cancel_retry(self):
        if self.retry_timer is not None:
            self.reactor.unregister_timer(self.retry_timer)
            self.retry_timer = None
    def set_value(self, print_time, value, cycle_time=None):
        if value and self.fault_latched:
            raise self.printer.command_error(
                "PMIC channel '%s' has a latched fault" % (self.name,))
        if not value and self.fault_latched:
            self._cancel_retry()
            self.retry_value = 0.
            self.commanded_value = 0.
            self._write(print_time, 0., cycle_time)
            self.retry_timer = self.reactor.register_timer(
                self._finish_fault, self.fault_deadline)
            return
        self.commanded_value = value
        self._write(print_time, value, cycle_time)
    def _fault_event(self, eventtime, state):
        self.fault_active = bool(state)
        if (not self.fault_active or not self.commanded_value
                or self.fault_latched):
            return
        self.fault_latched = True
        self.fault_time = eventtime
        self.fault_deadline = eventtime + self.off_delay
        self.retry_value = self.commanded_value
        self.retry_count = 0
        msg = "PMIC channel '%s' latched a %s fault" % (
            self.name, self.error_severity)
        if self.error_severity == 'critical':
            logging.error(msg)
        elif self.error_severity == 'warning':
            logging.warning(msg)
        self._write(self._print_time(eventtime), 0.)
        self.retry_timer = self.reactor.register_timer(
            self._retry_on_event,
            min(self.fault_deadline, eventtime + self.toggle_delay))
    def _retry_on_event(self, eventtime):
        if eventtime >= self.fault_deadline:
            return self._finish_fault(eventtime)
        self.retry_count += 1
        self._write(self._print_time(eventtime), self.retry_value)
        self.retry_timer = self.reactor.register_timer(
            self._retry_off_event,
            min(self.fault_deadline, eventtime + self.retry_interval
                - self.toggle_delay))
        return self.reactor.NEVER
    def _retry_off_event(self, eventtime):
        if eventtime >= self.fault_deadline:
            return self._finish_fault(eventtime)
        self._write(self._print_time(eventtime), 0.)
        self.retry_timer = self.reactor.register_timer(
            self._retry_on_event,
            min(self.fault_deadline, eventtime + self.toggle_delay))
        return self.reactor.NEVER
    def _finish_fault(self, eventtime):
        self.retry_timer = None
        self.commanded_value = self.retry_value = 0.
        self._write(self._print_time(eventtime), 0.)
        msg = "PMIC channel '%s' disabled after fault retry window" % (
            self.name,)
        if self.error_severity == 'critical':
            self.printer.invoke_shutdown(msg)
        elif self.error_severity == 'warning':
            logging.warning(msg)
        return self.reactor.NEVER
    def _handle_shutdown(self):
        self._cancel_retry()
    def get_status(self, eventtime):
        remaining = 0.
        if self.fault_latched and self.fault_deadline is not None:
            remaining = max(0., self.fault_deadline - eventtime)
        return {'value': self.commanded_value, 'fault': self.fault_active,
                'fault_latched': self.fault_latched,
                'retry_count': self.retry_count,
                'error_severity': self.error_severity,
                'shutdown_value': self.shutdown_value,
                'off_delay_remaining': remaining}
    cmd_CLEAR_PMIC_FAULT_help = "Clear a PMIC channel's latched fault"
    def cmd_CLEAR_PMIC_FAULT(self, gcmd):
        if self.fault_active:
            raise gcmd.error("PMIC channel '%s' fault is still active"
                             % (self.name,))
        self._cancel_retry()
        if self.output is not None:
            eventtime = self.reactor.monotonic()
            self._write(self._print_time(eventtime), 0.)
        self.commanded_value = 0.
        self.fault_latched = False
        self.fault_time = self.fault_deadline = None
        self.retry_value = 0.
        gcmd.respond_info("PMIC channel '%s' fault latch cleared"
                          % (self.name,))

def load_config_prefix(config):
    return PMICChannel(config)
