# Bounded FLOW solenoid door actuator and position sensor
#
# Copyright (C) 2026  Klipper developers
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging


class DoorHandler:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.open_state = config.getint('open_state', 1, minval=0, maxval=1)
        self.initial_state = config.getint(
            'initial_state', 1 - self.open_state, minval=0, maxval=1)
        self.max_on_time = config.getfloat(
            'max_on_time', 2., above=0., maxval=2.)
        self.off_margin = config.getfloat(
            'off_margin', .2, above=0., below=self.max_on_time)
        self.host_on_time = self.max_on_time - self.off_margin
        self.work_window = config.getfloat('work_window', 60., above=0.)
        self.max_work_time = config.getfloat(
            'max_work_time', 6., above=0., maxval=self.work_window)
        if self.max_work_time < self.max_on_time:
            raise config.error("max_work_time must be at least max_on_time")
        self.sensor_state = self.initial_state
        self.solenoid_active = False
        self.solenoid_start = None
        self.work_history = []
        self.timed_out = False
        self.open_timer = None

        ppins = self.printer.lookup_object('pins')
        self.solenoid = ppins.setup_pin(
            'digital_out', config.get('solenoid_pin'))
        self.solenoid.setup_max_duration(self.max_on_time)
        self.solenoid.setup_start_value(0., 0.)

        buttons = self.printer.load_object(config, 'buttons')
        buttons.register_debounce_button(
            config.get('sensor_pin'), self._sensor_event, config)

        gcode = self.printer.lookup_object('gcode')
        gcode.register_command(
            'OPEN_DOOR', self.cmd_OPEN_DOOR, desc=self.cmd_OPEN_DOOR_help)
        gcode.register_command(
            'QUERY_DOOR', self.cmd_QUERY_DOOR, desc=self.cmd_QUERY_DOOR_help)
        self.printer.register_event_handler(
            'klippy:shutdown', self._handle_shutdown)

    def _print_time(self, eventtime):
        mcu = self.solenoid.get_mcu()
        return mcu.estimated_print_time(eventtime + mcu.min_schedule_time())

    def _set_solenoid(self, eventtime, active):
        active = bool(active)
        if active:
            if self.solenoid_active:
                return
            # The PMIC virtual pin may reject activation due to a latched
            # fault.  Do not publish an active state until that write succeeds.
            self.solenoid.set_digital(self._print_time(eventtime), True)
            self.solenoid_start = eventtime
            self.solenoid_active = True
            return
        if not self.solenoid_active:
            return
        try:
            self.solenoid.set_digital(self._print_time(eventtime), False)
        finally:
            # A PMIC fault may already have forced the physical output off.
            # Internal state and thermal accounting must still be finalized.
            self.work_history.append((self.solenoid_start, eventtime))
            self.solenoid_start = None
            self.solenoid_active = False

    def _cancel_timer(self):
        if self.open_timer is not None:
            self.reactor.unregister_timer(self.open_timer)
            self.open_timer = None

    def _is_open(self):
        return (self.sensor_state is not None
                and self.sensor_state == self.open_state)

    def _thermal_status(self, eventtime):
        cutoff = eventtime - self.work_window
        self.work_history = [(start, end) for start, end in self.work_history
                             if end > cutoff]
        intervals = list(self.work_history)
        if self.solenoid_active:
            intervals.append((self.solenoid_start, eventtime))
        used = sum(end - max(start, cutoff) for start, end in intervals)
        cooldown = 0.
        if used + self.host_on_time > self.max_work_time:
            excess = used + self.host_on_time - self.max_work_time
            for start, end in intervals:
                duration = end - max(start, cutoff)
                if duration >= excess:
                    cooldown = max(0., end + self.work_window - eventtime)
                    break
                excess -= duration
        return used, cooldown

    def _sensor_event(self, eventtime, state):
        self.sensor_state = int(bool(state))
        if self.solenoid_active and self._is_open():
            self._cancel_timer()
            self._set_solenoid(eventtime, False)
            logging.info("Door opened; solenoid disabled")

    def _timeout_event(self, eventtime):
        self.open_timer = None
        if not self.solenoid_active:
            return self.reactor.NEVER
        self._set_solenoid(eventtime, False)
        self.timed_out = not self._is_open()
        if self.timed_out:
            logging.warning(
                "Door did not open within %.3f seconds; solenoid disabled",
                self.host_on_time)
        return self.reactor.NEVER

    def _handle_shutdown(self):
        self._cancel_timer()
        self.solenoid_active = False
        self.solenoid_start = None

    def get_status(self, eventtime):
        used, cooldown = self._thermal_status(eventtime)
        if self.sensor_state is None:
            state = 'unknown'
        elif self._is_open():
            state = 'open'
        else:
            state = 'closed'
        return {'state': state, 'is_open': self._is_open(),
                'solenoid_active': self.solenoid_active,
                'timed_out': self.timed_out,
                'max_on_time': self.max_on_time,
                'host_on_time': self.host_on_time,
                'work_time': used, 'max_work_time': self.max_work_time,
                'work_window': self.work_window,
                'cooldown_remaining': cooldown}

    cmd_OPEN_DOOR_help = "Pulse the door solenoid until open or timeout"
    def cmd_OPEN_DOOR(self, gcmd):
        if self.sensor_state is None:
            raise gcmd.error("Door sensor state is not available")
        if self._is_open():
            gcmd.respond_info("Door is already open")
            return
        if self.solenoid_active:
            raise gcmd.error("Door opening is already in progress")
        eventtime = self.reactor.monotonic()
        used, cooldown = self._thermal_status(eventtime)
        if cooldown:
            raise gcmd.error(
                "Door solenoid cooling down; retry in %.1f seconds"
                % (cooldown,))
        self.timed_out = False
        self._set_solenoid(eventtime, True)
        self.open_timer = self.reactor.register_timer(
            self._timeout_event, eventtime + self.host_on_time)
        gcmd.respond_info(
            "Door solenoid enabled for %.3f seconds (%.3f second MCU limit)"
            % (self.host_on_time, self.max_on_time))

    cmd_QUERY_DOOR_help = "Report door sensor and solenoid state"
    def cmd_QUERY_DOOR(self, gcmd):
        status = self.get_status(self.reactor.monotonic())
        gcmd.respond_info(
            "Door: %s; solenoid: %s; timed_out: %s; "
            "work: %.3f/%.3f seconds per %.1f seconds; cooldown: %.1f seconds"
            % (
                status['state'],
                'on' if status['solenoid_active'] else 'off',
                'yes' if status['timed_out'] else 'no',
                status['work_time'], status['max_work_time'],
                status['work_window'], status['cooldown_remaining']))


def load_config(config):
    return DoorHandler(config)
