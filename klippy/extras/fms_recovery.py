# Filament motion sensor clog recovery
#
# Copyright (C) 2026  Oliver
#
# This file may be distributed under the terms of the GNU GPLv3 license.

STATE_IDLE = 'idle'
STATE_RETRACTING = 'retracting'
STATE_EXTRUDING = 'extruding'
STATE_CLEANING = 'cleaning'
STATE_FAILED = 'failed'


class FMSRecovery:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.sensor_names = config.getlist('sensors')
        self.cleaning_gcodes = config.getlist('cleaning_gcodes')
        if len(self.sensor_names) != len(self.cleaning_gcodes):
            raise config.error(
                'sensors and cleaning_gcodes must contain the same number '
                'of entries')
        self.cleaning_by_sensor = dict(zip(
            self.sensor_names, self.cleaning_gcodes))
        self.max_attempts = config.getint(
            'max_attempts', 3, minval=1, maxval=20)
        self.retract_distance = config.getfloat(
            'retract_distance', 5., above=0.)
        self.extrude_distance = config.getfloat(
            'extrude_distance', 8., above=0.)
        self.retract_speed = config.getfloat(
            'retract_speed', 10., above=0.)
        self.extrude_speed = config.getfloat(
            'extrude_speed', 1.5, above=0.)
        self.check_distance = config.getfloat(
            'check_distance', .2, above=0.)
        if self.check_distance > self.extrude_distance:
            raise config.error(
                'check_distance must not exceed extrude_distance')
        self.repair_z_hop = config.getfloat(
            'repair_z_hop', .2, minval=0.)
        self.cleaning_z_hop = config.getfloat(
            'cleaning_z_hop', 10., minval=self.repair_z_hop)
        self.z_speed = config.getfloat('z_speed', 5., above=0.)
        self.travel_speed = config.getfloat('travel_speed', 100., above=0.)
        self.pause_x = config.getfloat('pause_x', None)
        self.pause_y = config.getfloat('pause_y', None)
        if (self.pause_x is None) != (self.pause_y is None):
            raise config.error('pause_x and pause_y must be specified together')
        self.state = STATE_IDLE
        self.sensor_name = self.error = None
        self.sensor = self.pause_resume = self.toolhead = None
        self.has_dual_carriage = False
        self.attempt = 0
        self.attempt_extruded = self.missing_distance = 0.
        self.attempt_motion = self.window_extrusion = 0.
        self.window_motion = 0.
        self.flow_percentage = 100.
        self.motion_baseline = None
        self.motion_seen = False
        self.timer = self.reactor.register_timer(self._process)
        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)
        self.gcode.register_command(
            'START_FMS_RECOVERY', self.cmd_START_FMS_RECOVERY,
            desc='Attempt automatic filament clog recovery')
        self.gcode.register_command(
            'ABORT_FMS_RECOVERY', self.cmd_ABORT_FMS_RECOVERY,
            desc='Abort automatic filament clog recovery')

    def _handle_connect(self):
        self.pause_resume = self.printer.lookup_object('pause_resume')
        self.toolhead = self.printer.lookup_object('toolhead')
        self.has_dual_carriage = (
            self.printer.lookup_object('dual_carriage', None) is not None)
        for name in self.sensor_names:
            sensor = self.printer.lookup_object(
                'pat9125_filament_sensor ' + name, None)
            if sensor is None:
                raise self.printer.config_error(
                    'fms_recovery sensor %s is not configured' % (name,))

    def _run_relative_move(self, distance, speed):
        self.gcode.run_script_from_command(
            'SAVE_GCODE_STATE NAME=FMS_RECOVERY_MOVE\n'
            'M83\nG1 E%.6f F%.3f\nM400\n'
            'RESTORE_GCODE_STATE NAME=FMS_RECOVERY_MOVE'
            % (distance, speed * 60.))

    def _run_z_move(self, distance):
        if not distance:
            return
        self.gcode.run_script_from_command(
            'SAVE_GCODE_STATE NAME=FMS_RECOVERY_Z\n'
            'G91\nG1 Z%.6f F%.3f\nM400\n'
            'RESTORE_GCODE_STATE NAME=FMS_RECOVERY_Z'
            % (distance, self.z_speed * 60.))

    def _move_heads_to_pause(self):
        if self.pause_x is None:
            return
        modes = self.printer.lookup_object('flow_idex_modes', None)
        if modes is not None:
            modes.park_for_recovery(
                self.pause_x, self.pause_y, self.travel_speed)
            return
        self.gcode.run_script_from_command(
            'SAVE_GCODE_STATE NAME=FMS_RECOVERY_PARK\n'
            'G90\nG1 X%.6f Y%.6f F%.3f\nM400\n'
            'RESTORE_GCODE_STATE NAME=FMS_RECOVERY_PARK'
            % (self.pause_x, self.pause_y, self.travel_speed * 60.))

    def _fail(self, message, permit_failover=False):
        self.state = STATE_FAILED
        self.error = message
        modes = self.printer.lookup_object('flow_idex_modes', None)
        if modes is not None:
            modes.end_recovery()
        if self.sensor is not None:
            self.sensor.reset_watchdog()
        if permit_failover and self.sensor_name is not None:
            if modes is not None:
                try:
                    if modes.attempt_failover(self.sensor_name):
                        failed_sensor = self.sensor_name
                        self.state = STATE_IDLE
                        self.sensor = self.sensor_name = None
                        self.gcode.run_script_from_command('RESUME')
                        self.gcode.respond_info(
                            'FMS recovery failed for %s; FLOW backup '
                            'failover succeeded' % (failed_sensor,))
                        return self.reactor.NEVER
                except Exception as error:
                    message += '; backup failover rejected: %s' % (error,)
                    self.error = message
        self.gcode.respond_info(
            'FMS recovery stopped: %s. Print remains paused.' % (message,))
        return self.reactor.NEVER

    def _validate_start(self, sensor_name):
        if self.state not in (STATE_IDLE, STATE_FAILED):
            raise self.printer.command_error('FMS recovery is already active')
        if sensor_name not in self.cleaning_by_sensor:
            raise self.printer.command_error(
                'Unknown FMS recovery sensor %s' % (sensor_name,))
        if not self.pause_resume.is_paused:
            raise self.printer.command_error(
                'Pause the print before starting FMS recovery')
        sensor = self.printer.lookup_object(
            'pat9125_filament_sensor ' + sensor_name)
        modes = self.printer.lookup_object('flow_idex_modes', None)
        synchronized = modes is not None and modes.is_synchronized()
        if (not synchronized
                and self.toolhead.get_extruder().get_name()
                != sensor.extruder_name):
            raise self.printer.command_error(
                'Sensor %s does not match the active extruder'
                % (sensor_name,))
        status = self.toolhead.get_status(self.reactor.monotonic())
        if status.get('homed_axes') != 'xyz':
            raise self.printer.command_error(
                'FMS recovery requires homed X, Y, and Z axes')
        extruder_status = sensor.extruder.get_status(
            self.reactor.monotonic())
        if not extruder_status.get('can_extrude', False):
            raise self.printer.command_error(
                'Active extruder is below its minimum extrusion temperature')
        position = self.toolhead.get_position()
        axis_maximum = status.get('axis_maximum')
        # Cleaning macros raise Z by another 1mm while wiping.
        if (axis_maximum is None
                or position[2] + self.cleaning_z_hop + 1. > axis_maximum[2]):
            raise self.printer.command_error(
                'Insufficient Z clearance for FMS recovery cleaning')
        return sensor

    def cmd_START_FMS_RECOVERY(self, gcmd):
        sensor_name = gcmd.get('SENSOR')
        self.sensor = self._validate_start(sensor_name)
        self.sensor_name = sensor_name
        self.error = None
        self.attempt = 0
        self.attempt_extruded = self.missing_distance = 0.
        self.attempt_motion = self.window_extrusion = 0.
        self.window_motion = 0.
        self.flow_percentage = 100.
        self.motion_baseline = self.sensor.get_motion_count()
        self.motion_seen = False
        self.sensor.reset_watchdog()
        modes = self.printer.lookup_object('flow_idex_modes', None)
        if modes is not None:
            modes.begin_recovery(sensor_name)
        self.gcode.run_script_from_command(
            'SAVE_GCODE_STATE NAME=FMS_RECOVERY_POSITION')
        if self.has_dual_carriage:
            self.gcode.run_script_from_command(
                'SAVE_DUAL_CARRIAGE_STATE NAME=FMS_RECOVERY_CARRIAGES')
        self._run_z_move(self.repair_z_hop)
        self._move_heads_to_pause()
        self.state = STATE_RETRACTING
        self.reactor.update_timer(self.timer, self.reactor.NOW)
        gcmd.respond_info('FMS recovery started for %s' % (sensor_name,))

    def cmd_ABORT_FMS_RECOVERY(self, gcmd):
        if self.state in (STATE_IDLE, STATE_FAILED):
            gcmd.respond_info('FMS recovery is not active')
            return
        self._fail('aborted by operator')

    def _measured_motion(self):
        count = self.sensor.get_motion_count()
        delta = count - self.motion_baseline
        self.motion_baseline = count
        direction = self.sensor.motion_direction
        motion = (abs(delta) if not direction else delta * direction)
        motion = max(0., motion / self.sensor.counts_per_mm)
        if motion >= self.sensor.minimum_motion:
            self.motion_seen = True
        return motion

    def _process(self, eventtime):
        try:
            if not self.pause_resume.is_paused:
                return self._fail('printer is no longer paused')
            if self.state == STATE_RETRACTING:
                self.attempt += 1
                self._run_relative_move(
                    -self.retract_distance, self.retract_speed)
                self.attempt_extruded = self.missing_distance = 0.
                self.attempt_motion = self.window_extrusion = 0.
                self.window_motion = 0.
                self.flow_percentage = 100.
                self.motion_baseline = self.sensor.get_motion_count()
                self.motion_seen = False
                self.state = STATE_EXTRUDING
                return self.reactor.NOW
            if self.state == STATE_EXTRUDING:
                distance = min(self.check_distance,
                               self.extrude_distance
                               - self.attempt_extruded)
                self._run_relative_move(distance, self.extrude_speed)
                self.attempt_extruded += distance
                measured = self._measured_motion()
                self.attempt_motion += measured
                self.window_extrusion += distance
                self.window_motion += measured
                self.missing_distance = max(
                    0., self.window_extrusion - self.window_motion)
                if self.window_extrusion >= self.sensor.detection_length:
                    self.flow_percentage = min(
                        100., 100. * self.window_motion
                        / self.window_extrusion)
                    healthy = (
                        self.flow_percentage >= self.sensor.minimum_flow)
                    self.window_extrusion = self.window_motion = 0.
                    self.missing_distance = 0.
                    if not healthy:
                        if self.attempt >= self.max_attempts:
                            return self._fail(
                                'flow remained below %.1f%% after %d attempts'
                                % (self.sensor.minimum_flow, self.attempt),
                                permit_failover=True)
                        self.state = STATE_RETRACTING
                        return self.reactor.NOW
                if self.attempt_extruded < self.extrude_distance:
                    return self.reactor.NOW
                self.flow_percentage = min(
                    100., 100. * self.attempt_motion
                    / self.attempt_extruded)
                if (not self.motion_seen
                        or self.flow_percentage < self.sensor.minimum_flow):
                    if self.attempt >= self.max_attempts:
                        return self._fail(
                            'flow remained below %.1f%% after %d attempts'
                            % (self.sensor.minimum_flow, self.attempt),
                            permit_failover=True)
                    self.state = STATE_RETRACTING
                    return self.reactor.NOW
                self.state = STATE_CLEANING
                return self.reactor.NOW
            if self.state == STATE_CLEANING:
                self._run_z_move(self.cleaning_z_hop - self.repair_z_hop)
                self.gcode.run_script_from_command(
                    self.cleaning_by_sensor[self.sensor_name] + '\nM400')
                modes = self.printer.lookup_object('flow_idex_modes', None)
                if modes is not None:
                    modes.end_recovery()
                if self.has_dual_carriage:
                    self.gcode.run_script_from_command(
                        'RESTORE_DUAL_CARRIAGE_STATE '
                        'NAME=FMS_RECOVERY_CARRIAGES MOVE=1 '
                        'MOVE_SPEED=%.3f\nM400' % (self.travel_speed,))
                self.gcode.run_script_from_command(
                    'RESTORE_GCODE_STATE NAME=FMS_RECOVERY_POSITION '
                    'MOVE=1 MOVE_SPEED=%.3f\nM400'
                    % (self.travel_speed,))
                self.sensor.reset_watchdog()
                recovered_sensor = self.sensor_name
                attempts = self.attempt
                self.state = STATE_IDLE
                self.sensor = self.sensor_name = None
                self.gcode.run_script_from_command('RESUME')
                self.gcode.respond_info(
                    'FMS recovery succeeded for %s after %d attempt(s)'
                    % (recovered_sensor, attempts))
                return self.reactor.NEVER
        except Exception as error:
            return self._fail(str(error))
        return self.reactor.NEVER

    def get_status(self, eventtime):
        return {
            'state': self.state,
            'active': self.state not in (STATE_IDLE, STATE_FAILED),
            'sensor': self.sensor_name,
            'attempt': self.attempt,
            'max_attempts': self.max_attempts,
            'attempt_extruded': self.attempt_extruded,
            'missing_distance': self.missing_distance,
            'attempt_motion': self.attempt_motion,
            'flow_percentage': self.flow_percentage,
            'error': self.error,
        }


def load_config(config):
    return FMSRecovery(config)
