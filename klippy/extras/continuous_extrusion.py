# Continuous manual extrusion with a client keepalive
#
# Copyright (C) 2026  Craftunique Ltd.
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class ContinuousExtrusion:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.queue_horizon = config.getfloat(
            'queue_horizon', .300, minval=.100, maxval=1.)
        self.watchdog_timeout = config.getfloat(
            'watchdog_timeout', .600, above=self.queue_horizon, maxval=2.)
        self.client = None
        self.direction = 0.
        self.speed = 0.
        self.last_keepalive = 0.
        self.queued_until = 0.
        self.toolhead = None
        self.watchdog_timer = self.reactor.register_timer(
            self._watchdog_event)
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler(
            'klippy:shutdown', self._handle_shutdown)
        self.gcode.register_command(
            'MANUAL_EXTRUDE_START', self.cmd_MANUAL_EXTRUDE_START,
            desc=self.cmd_MANUAL_EXTRUDE_START_help)
        self.gcode.register_command(
            'MANUAL_EXTRUDE_KEEPALIVE', self.cmd_MANUAL_EXTRUDE_KEEPALIVE,
            desc=self.cmd_MANUAL_EXTRUDE_KEEPALIVE_help)
        self.gcode.register_command(
            'MANUAL_EXTRUDE_STOP', self.cmd_MANUAL_EXTRUDE_STOP,
            desc=self.cmd_MANUAL_EXTRUDE_STOP_help)
        webhooks = self.printer.lookup_object('webhooks')
        webhooks.register_endpoint(
            'continuous_extrusion/keepalive', self._handle_keepalive_request)

    def _handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')

    def _handle_shutdown(self):
        self._clear()

    def _clear(self):
        self.client = None
        self.direction = 0.
        self.speed = 0.
        self.last_keepalive = 0.
        self.queued_until = 0.
        self.reactor.update_timer(self.watchdog_timer, self.reactor.NEVER)

    def _check_print_state(self, gcmd):
        print_stats = self.printer.lookup_object('print_stats', None)
        if print_stats is None:
            return
        state = print_stats.get_status(self.reactor.monotonic()).get('state')
        if state in ('printing', 'paused'):
            raise gcmd.error(
                'Manual extrusion is unavailable while printing or paused')

    def _check_client_value(self, request, client):
        if client != self.client:
            raise request.error('Manual extrusion is owned by another client')

    def _check_client(self, gcmd):
        self._check_client_value(gcmd, gcmd.get('CLIENT'))

    def _queue_motion(self, gcmd, eventtime):
        self._check_print_state(gcmd)
        extruder = self.toolhead.get_extruder()
        if not extruder.get_heater().can_extrude:
            raise gcmd.error(
                "Extrude below minimum temp\n"
                "See the 'min_extrude_temp' config option for details")
        queued_time = max(0., self.queued_until - eventtime)
        missing_time = self.queue_horizon - queued_time
        if missing_time <= .010:
            return
        distance = self.direction * self.speed * missing_time
        position = self.toolhead.get_position()
        position[3] += distance
        self.toolhead.manual_move(position, self.speed)
        self.queued_until = max(eventtime, self.queued_until) + missing_time

    def _refresh(self, gcmd):
        eventtime = self.reactor.monotonic()
        self.last_keepalive = eventtime
        self.reactor.update_timer(
            self.watchdog_timer, eventtime + self.watchdog_timeout)
        try:
            self._queue_motion(gcmd, eventtime)
        except:
            self._clear()
            raise

    def _watchdog_event(self, eventtime):
        if self.client is None:
            return self.reactor.NEVER
        deadline = self.last_keepalive + self.watchdog_timeout
        if eventtime < deadline:
            return deadline
        self._clear()
        return self.reactor.NEVER

    cmd_MANUAL_EXTRUDE_START_help = (
        'Start watchdog-controlled manual extrusion')
    def cmd_MANUAL_EXTRUDE_START(self, gcmd):
        if self.toolhead is None:
            raise gcmd.error('Printer is not ready')
        client = gcmd.get('CLIENT')
        if not client or len(client) > 64:
            raise gcmd.error('CLIENT must contain between 1 and 64 characters')
        if self.client is not None and self.client != client:
            raise gcmd.error('Manual extrusion is already active')
        self._check_print_state(gcmd)
        direction = gcmd.get_int('DIRECTION', minval=-1, maxval=1)
        if direction == 0:
            raise gcmd.error('DIRECTION must be -1 or 1')
        speed = gcmd.get_float('SPEED', above=0.)
        self.client = client
        self.direction = float(direction)
        self.speed = speed
        self._refresh(gcmd)

    cmd_MANUAL_EXTRUDE_KEEPALIVE_help = (
        'Extend an active manual extrusion session')
    def cmd_MANUAL_EXTRUDE_KEEPALIVE(self, gcmd):
        self._check_client(gcmd)
        self._refresh(gcmd)

    def _handle_keepalive_request(self, web_request):
        self._check_client_value(
            web_request, web_request.get_str('client'))
        self._refresh(web_request)

    cmd_MANUAL_EXTRUDE_STOP_help = 'Stop extending manual extrusion motion'
    def cmd_MANUAL_EXTRUDE_STOP(self, gcmd):
        self._check_client(gcmd)
        self._clear()

    def get_status(self, eventtime):
        remaining = 0.
        if self.client is not None:
            remaining = max(
                0., self.last_keepalive + self.watchdog_timeout - eventtime)
        return {
            'active': self.client is not None,
            'direction': int(self.direction),
            'speed': self.speed,
            'watchdog_remaining': remaining,
            'queue_horizon': self.queue_horizon,
            'watchdog_timeout': self.watchdog_timeout,
        }


def load_config(config):
    return ContinuousExtrusion(config)
