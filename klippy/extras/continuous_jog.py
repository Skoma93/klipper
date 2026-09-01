# Watchdog-controlled continuous manual XYZ jogging
#
# Copyright (C) 2026  Craftunique Ltd.
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math


class ContinuousJog:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.watchdog_timeout = config.getfloat(
            'watchdog_timeout', .150, minval=.050, maxval=2.)
        self.idex_safe_distance = config.getfloat(
            'idex_safe_distance', 0., minval=0.)
        legacy_jog_accel = config.getfloat('jog_accel', 300., above=0.)
        self.jog_xy_accel = config.getfloat(
            'jog_xy_accel', legacy_jog_accel, above=0.)
        self.jog_z_accel = config.getfloat(
            'jog_z_accel', legacy_jog_accel, above=0.)
        self.client = None
        self.direction = [0., 0., 0.]
        self.speed = 0.
        self.last_keepalive = 0.
        self.toolhead = None
        self.drip_completion = None
        self.watchdog_timer = self.reactor.register_timer(
            self._watchdog_event)
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler(
            'klippy:shutdown', self._handle_shutdown)
        self.printer.register_event_handler(
            'klippy:disconnect', self._handle_shutdown)
        self.gcode.register_command(
            'MANUAL_JOG_START', self.cmd_MANUAL_JOG_START,
            desc=self.cmd_MANUAL_JOG_START_help)
        self.gcode.register_command(
            'MANUAL_JOG_KEEPALIVE', self.cmd_MANUAL_JOG_KEEPALIVE,
            desc=self.cmd_MANUAL_JOG_KEEPALIVE_help)
        self.gcode.register_command(
            'MANUAL_JOG_STOP', self.cmd_MANUAL_JOG_STOP,
            desc=self.cmd_MANUAL_JOG_STOP_help)
        webhooks = self.printer.lookup_object('webhooks')
        webhooks.register_endpoint(
            'continuous_jog/start', self._handle_start_request)
        webhooks.register_endpoint(
            'continuous_jog/keepalive', self._handle_keepalive_request)
        webhooks.register_endpoint(
            'continuous_jog/stop', self._handle_stop_request)

    def _handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')

    def _handle_shutdown(self):
        self._clear()

    def _clear(self):
        if self.drip_completion is not None:
            self.drip_completion.complete(1)
        self.client = None
        self.direction = [0., 0., 0.]
        self.speed = 0.
        self.last_keepalive = 0.
        self.drip_completion = None
        self.reactor.update_timer(self.watchdog_timer, self.reactor.NEVER)

    def _check_print_state(self, request):
        print_stats = self.printer.lookup_object('print_stats', None)
        if print_stats is None:
            return
        state = print_stats.get_status(self.reactor.monotonic()).get('state')
        if state in ('printing', 'paused'):
            raise request.error(
                'Manual jogging is unavailable while printing or paused')

    def _check_homed_axes(self, request):
        homed_axes = self.toolhead.get_kinematics().get_status(
            self.reactor.monotonic()).get('homed_axes', '')
        names = 'xyz'
        missing = [names[i] for i, value in enumerate(self.direction)
                   if value and names[i] not in homed_axes]
        if missing:
            raise request.error(
                'Must home axis first: %s' % (','.join(missing).upper(),))

    def _check_client_value(self, request, client):
        if client != self.client:
            raise request.error('Manual jogging is owned by another client')

    def _check_client(self, gcmd):
        self._check_client_value(gcmd, gcmd.get('CLIENT'))

    def _get_target(self, request):
        eventtime = self.reactor.monotonic()
        status = self.toolhead.get_status(eventtime)
        position = self.toolhead.get_position()
        axis_minimum = status['axis_minimum']
        axis_maximum = status['axis_maximum']
        x_minimum = axis_minimum[0]
        x_maximum = axis_maximum[0]
        dual_carriage = self.printer.lookup_object('dual_carriage', None)
        if dual_carriage is not None and self.direction[0]:
            carriages = dual_carriage.get_status(eventtime)['carriages']
            carriage_positions = dual_carriage.save_dual_carriage_state()[
                'carriage_positions']
            primary_active = (carriages.get('stepper_x') != 'INACTIVE'
                              and carriages.get('dual_carriage') == 'INACTIVE')
            secondary_active = (carriages.get('stepper_x') == 'INACTIVE'
                                and carriages.get('dual_carriage') != 'INACTIVE')
            if primary_active:
                inactive_x = carriage_positions.get('dual_carriage')
                if inactive_x is None:
                    raise request.error(
                        'Unable to determine inactive T1 carriage position')
                x_maximum = min(
                    x_maximum, inactive_x - self.idex_safe_distance)
            elif secondary_active:
                inactive_x = carriage_positions.get('stepper_x')
                if inactive_x is None:
                    raise request.error(
                        'Unable to determine inactive T0 carriage position')
                x_minimum = max(
                    x_minimum, inactive_x + self.idex_safe_distance)
            else:
                raise request.error(
                    'Manual X jogging requires NORMAL single-carriage mode')
        minimums = [x_minimum, axis_minimum[1], axis_minimum[2]]
        maximums = [x_maximum, axis_maximum[1], axis_maximum[2]]
        distances = []
        for axis in range(3):
            component = self.direction[axis]
            if component > 0.:
                distances.append(
                    (maximums[axis] - position[axis]) / component)
            elif component < 0.:
                distances.append(
                    (minimums[axis] - position[axis]) / component)
        distance = min(distances)
        if distance <= 0.:
            raise request.error('Jog direction is already at its travel limit')
        target = list(position)
        for axis in range(3):
            target[axis] += self.direction[axis] * distance
        return target

    def _resync_position(self):
        self.toolhead.flush_step_generation()
        kin = self.toolhead.get_kinematics()
        stepper_positions = {
            stepper.get_name(): stepper.get_commanded_position()
            for stepper in kin.get_steppers()}
        halt_position = kin.calc_position(stepper_positions)
        current = self.toolhead.get_position()
        for axis in range(3):
            current[axis] = halt_position[axis]
        self.toolhead.set_position(current)

    def _run_motion(self, request):
        self._check_print_state(request)
        self._check_homed_axes(request)
        target = self._get_target(request)
        completion = self.reactor.completion()
        self.drip_completion = completion
        max_velocity, max_accel = self.toolhead.get_max_velocity()
        jog_accel = (self.jog_z_accel if self.direction[2]
                     else self.jog_xy_accel)
        move_accel = min(jog_accel, max_accel)
        try:
            self.toolhead.set_max_velocities(
                None, move_accel, None, None)
            self.toolhead.drip_move(target, self.speed, completion)
        finally:
            self.toolhead.set_max_velocities(
                None, max_accel, None, None)
            self._resync_position()
            self._clear()

    def _refresh(self, request):
        eventtime = self.reactor.monotonic()
        self.last_keepalive = eventtime
        self.reactor.update_timer(
            self.watchdog_timer, eventtime + self.watchdog_timeout)

    def _watchdog_event(self, eventtime):
        if self.client is None:
            return self.reactor.NEVER
        deadline = self.last_keepalive + self.watchdog_timeout
        if eventtime < deadline:
            return deadline
        self._clear()
        return self.reactor.NEVER

    cmd_MANUAL_JOG_START_help = 'Start watchdog-controlled manual XYZ jogging'
    def _start(self, request, client, direction, speed):
        if self.toolhead is None:
            raise request.error('Printer is not ready')
        if not client or len(client) > 64:
            raise request.error('CLIENT must contain between 1 and 64 characters')
        if self.client is not None and self.client != client:
            raise request.error('Manual jogging is already active')
        self._check_print_state(request)
        magnitude = math.sqrt(sum(value * value for value in direction))
        if not magnitude:
            raise request.error('At least one of X, Y, or Z must be nonzero')
        self.client = client
        self.direction = [value / magnitude for value in direction]
        self.speed = speed
        self._refresh(request)
        self._run_motion(request)

    def cmd_MANUAL_JOG_START(self, gcmd):
        direction = [gcmd.get_float(axis.upper(), 0., minval=-1., maxval=1.)
                     for axis in 'xyz']
        self._start(
            gcmd, gcmd.get('CLIENT'), direction,
            gcmd.get_float('SPEED', above=0.))

    def _handle_start_request(self, web_request):
        direction = [web_request.get_float(axis, 0.) for axis in 'xyz']
        if any(value < -1. or value > 1. for value in direction):
            raise web_request.error('Direction values must be from -1 to 1')
        speed = web_request.get_float('speed')
        if speed <= 0.:
            raise web_request.error('SPEED must be above zero')
        self._start(
            web_request, web_request.get_str('client'), direction, speed)

    cmd_MANUAL_JOG_KEEPALIVE_help = 'Extend an active manual XYZ jog session'
    def cmd_MANUAL_JOG_KEEPALIVE(self, gcmd):
        self._check_client(gcmd)
        self._refresh(gcmd)

    def _handle_keepalive_request(self, web_request):
        self._check_client_value(
            web_request, web_request.get_str('client'))
        self._refresh(web_request)

    cmd_MANUAL_JOG_STOP_help = 'Stop extending manual XYZ jog motion'
    def cmd_MANUAL_JOG_STOP(self, gcmd):
        self._check_client(gcmd)
        self._clear()

    def _handle_stop_request(self, web_request):
        self._check_client_value(
            web_request, web_request.get_str('client'))
        self._clear()

    def get_status(self, eventtime):
        remaining = 0.
        if self.client is not None:
            remaining = max(
                0., self.last_keepalive + self.watchdog_timeout - eventtime)
        return {
            'active': self.client is not None,
            'direction': list(self.direction),
            'speed': self.speed,
            'watchdog_remaining': remaining,
            'watchdog_timeout': self.watchdog_timeout,
            'idex_safe_distance': self.idex_safe_distance,
            'jog_accel': self.jog_xy_accel,
            'jog_xy_accel': self.jog_xy_accel,
            'jog_z_accel': self.jog_z_accel,
        }


def load_config(config):
    return ContinuousJog(config)
