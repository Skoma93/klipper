# Support fans that are enabled when a heater is on
#
# Copyright (C) 2016-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import fan

PIN_MIN_TIME = 0.100

class PrinterHeaterFan:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.load_object(config, 'heaters')
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.heater_names = config.getlist("heater", ("extruder",))
        self.heater_temp = config.getfloat("heater_temp", 50.0)
        self.heaters = []
        self.fan = fan.Fan(config, default_shutdown_speed=1.)
        self.fan_speed = config.getfloat("fan_speed", 1., minval=0., maxval=1.)
        self.speed_points = config.getlists(
            'speed_points', None, seps=(',', '\n'), parser=float, count=2)
        if self.speed_points is not None:
            prev_temp = None
            for temp, speed in self.speed_points:
                if prev_temp is not None and temp <= prev_temp:
                    raise config.error("Speed point temperatures must be "
                                       "strictly increasing")
                if speed < 0. or speed > 1.:
                    raise config.error("Speed point value %.3f outside range "
                                       "0.0 to 1.0" % (speed,))
                prev_temp = temp
            if len(self.speed_points) < 2:
                raise config.error("Option 'speed_points' must contain at "
                                   "least two temperature, speed pairs")
        self.last_speed = 0.
    def handle_ready(self):
        pheaters = self.printer.lookup_object('heaters')
        self.heaters = [pheaters.lookup_heater(n) for n in self.heater_names]
        reactor = self.printer.get_reactor()
        reactor.register_timer(self.callback, reactor.monotonic()+PIN_MIN_TIME)
    def get_status(self, eventtime):
        return self.fan.get_status(eventtime)
    def _curve_speed(self, temp):
        if temp <= self.speed_points[0][0]:
            return self.speed_points[0][1]
        if temp >= self.speed_points[-1][0]:
            return self.speed_points[-1][1]
        for (low_temp, low_speed), (high_temp, high_speed) in zip(
                self.speed_points, self.speed_points[1:]):
            if temp <= high_temp:
                fraction = (temp - low_temp) / (high_temp - low_temp)
                return low_speed + fraction * (high_speed - low_speed)
    def callback(self, eventtime):
        speed = 0.
        if self.speed_points is not None:
            current_temp = max(h.get_temp(eventtime)[0] for h in self.heaters)
            speed = self._curve_speed(current_temp)
        else:
            for heater in self.heaters:
                current_temp, target_temp = heater.get_temp(eventtime)
                if target_temp or current_temp > self.heater_temp:
                    speed = self.fan_speed
        if speed != self.last_speed:
            self.last_speed = speed
            self.fan.set_speed(speed)
        return eventtime + 1.

def load_config_prefix(config):
    return PrinterHeaterFan(config)
