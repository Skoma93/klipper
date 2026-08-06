# Sampled tachometer for a low-side-switched fan
#
# Copyright (C) 2026  Oliver
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import fan, pulse_counter


class SampledTachometer:
    def __init__(self, config, controlled_fan):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.fan = controlled_fan
        self.ppr = config.getint('tachometer_ppr', 2, minval=1)
        self.pulse_timeout = config.getfloat('pulse_timeout', .5, above=0.)
        self.sampling_pwm = config.getfloat(
            'sampling_pwm', 1., above=0., maxval=self.fan.max_power)
        poll_time = config.getfloat(
            'tachometer_poll_interval', .0015, above=0.)
        self.report_time = config.getfloat(
            'tachometer_report_interval', .050,
            above=poll_time, maxval=self.pulse_timeout)
        self.counter = pulse_counter.FrequencyCounter(
            self.printer, config.get('tachometer_pin'),
            self.report_time, poll_time)
        self.measuring = False
        self.rpm = 0.
        self.callback = self.override_token = None
        self.sample_timer = self.reactor.register_timer(self._sample)
        self.timeout_timer = self.reactor.register_timer(self._timeout)

    def measure(self, spin_up_time, callback):
        if self.measuring:
            return
        self.callback = callback
        self.measuring = True
        self.override_token = self.fan.begin_power_override(self.sampling_pwm)
        now = self.reactor.monotonic()
        self.reactor.update_timer(
            self.sample_timer, now + spin_up_time + 2. * self.report_time)
        self.reactor.update_timer(
            self.timeout_timer, now + spin_up_time + self.pulse_timeout)

    def _sample(self, eventtime):
        if not self.measuring:
            return self.reactor.NEVER
        frequency = self.counter.get_frequency()
        if frequency > 0.:
            self.rpm = frequency * 30. / self.ppr
            self._finish(eventtime, True)
            return self.reactor.NEVER
        return eventtime + self.report_time

    def _timeout(self, eventtime):
        if self.measuring:
            self._finish(eventtime, False)
        return self.reactor.NEVER

    def _finish(self, eventtime, rotating):
        self.measuring = False
        self.fan.end_speed_override(self.override_token)
        self.override_token = None
        self.reactor.update_timer(self.sample_timer, self.reactor.NEVER)
        self.reactor.update_timer(self.timeout_timer, self.reactor.NEVER)
        callback = self.callback
        self.callback = None
        if callback is not None:
            callback(rotating, self.rpm if rotating else 0.)

    def get_status(self, eventtime):
        return {'rpm': round(self.rpm, 1), 'measuring': self.measuring}


class SampledTachometerFan(fan.Fan):
    def __init__(self, config):
        fan.Fan.__init__(self, config, default_shutdown_speed=0.,
                         setup_tachometer=False)
        self.tachometer = SampledTachometer(config, self)

    def measure(self, spin_up_time, callback):
        self.tachometer.measure(spin_up_time, callback)

    def get_status(self, eventtime):
        status = fan.Fan.get_status(self, eventtime)
        status.update(self.tachometer.get_status(eventtime))
        return status
class NativeTachometer:
    def __init__(self, config):
        self.ppr = config.getint('tachometer_ppr', 2, minval=1)
        poll_time = config.getfloat(
            'tachometer_poll_interval', .0015, above=0.)
        sample_time = config.getfloat(
            'tachometer_report_interval', 1., above=poll_time)
        self.counter = pulse_counter.FrequencyCounter(
            config.get_printer(), config.get('tachometer_pin'),
            sample_time, poll_time)

    def get_status(self, eventtime):
        rpm = self.counter.get_frequency() * 30. / self.ppr
        return {'rpm': round(rpm, 1)}


def load_config_prefix(config):
    return NativeTachometer(config)