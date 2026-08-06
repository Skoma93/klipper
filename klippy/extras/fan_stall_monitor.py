# Generic fan stall warning monitor
#
# Copyright (C) 2026  Oliver
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class FanStallMonitor:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.name = config.get_name().split()[1]
        self.fan_name = config.get('fan')
        self.tachometer_name = config.get('tachometer', self.fan_name)
        self.check_interval = config.getfloat('check_interval', 5., above=0.)
        self.spin_up_time = config.getfloat('spin_up_time', .5, minval=0.)
        self.startup_delay = config.getfloat('startup_delay', 2., minval=0.)
        self.fan_object = self.fan = self.tachometer = None
        self.enabled_since = None
        self.stalled = False
        self.rpm = 0.
        self.timer = self.reactor.register_timer(self._check_fan)
        self.printer.register_event_handler('klippy:connect', self._connect)

    def _connect(self):
        self.fan_object = self.printer.lookup_object(self.fan_name)
        self.fan = self.fan_object.fan
        self.tachometer = self.printer.lookup_object(self.tachometer_name)
        self.reactor.update_timer(
            self.timer, self.reactor.monotonic() + self.check_interval)

    def _set_result(self, rotating, rpm):
        self.rpm = rpm
        if not rotating and not self.stalled:
            self.gcode.respond_info(
                'WARNING: %s is enabled but no tach rotation was detected'
                % self.name)
        self.stalled = not rotating

    def _check_fan(self, eventtime):
        fan_status = self.fan.get_status(eventtime)
        if fan_status['speed'] <= 0.:
            self.enabled_since = None
            self.stalled = False
            return eventtime + self.check_interval
        if fan_status.get('starting', False):
            self.enabled_since = None
            return eventtime + self.check_interval
        if self.enabled_since is None:
            self.enabled_since = eventtime
            return eventtime + self.startup_delay
        if eventtime - self.enabled_since < self.startup_delay:
            return eventtime + self.check_interval
        tachometer_fan = getattr(self.tachometer, 'fan', None)
        measure = getattr(tachometer_fan, 'measure', None)
        if measure is not None:
            measure(0., self._set_result)
        else:
            status = self.tachometer.get_status(eventtime)
            rpm = status.get('rpm')
            self._set_result(rpm is not None and rpm > 0., rpm or 0.)
        return eventtime + self.check_interval

    def get_status(self, eventtime):
        return {'rpm': round(self.rpm, 1), 'stalled': self.stalled}


def load_config_prefix(config):
    return FanStallMonitor(config)
