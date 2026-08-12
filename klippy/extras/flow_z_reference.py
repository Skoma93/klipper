# Bidirectional Z reference calibration and recovery homing
#
# Copyright (C) 2026  Klipper developers
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from . import homing


class FlowZReference:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.lower_position = config.getfloat(
            'lower_position', None, minval=0.)
        self.search_start = config.getfloat('search_start', minval=0.)
        self.search_target = config.getfloat(
            'search_target', above=self.search_start)
        self.temporary_position_max = config.getfloat(
            'temporary_position_max', self.search_target,
            minval=self.search_target)
        self.calibration_return_position = config.getfloat(
            'calibration_return_position', 10.)
        self.travel_speed = config.getfloat('travel_speed', 50., above=0.)
        self.speed = config.getfloat('speed', 5., above=0.)
        self.retract_distance = config.getfloat(
            'retract_distance', 2., above=0.)
        self.recovery_release_distance = config.getfloat(
            'recovery_release_distance', 10., above=0.)
        self.second_speed = config.getfloat(
            'second_speed', self.speed, above=0.)
        self.endstop = None
        self.kinematics = None
        self.normal_z_range = None
        self.last_measured = None
        self.last_action = 'idle'
        self.printer.register_event_handler(
            'klippy:connect', self._handle_connect)
        self.gcode.register_command(
            'CALIBRATE_LOWER_Z_REFERENCE',
            self.cmd_CALIBRATE_LOWER_Z_REFERENCE,
            desc='Measure and stage the lower PC25 Z reference')
        self.gcode.register_command(
            'HOME_Z_FROM_LOWER_REFERENCE',
            self.cmd_HOME_Z_FROM_LOWER_REFERENCE,
            desc='Recovery-home Z downward using the saved PC25 reference')
        self.gcode.register_command(
            'QUERY_FLOW_Z_REFERENCE', self.cmd_QUERY_FLOW_Z_REFERENCE,
            desc='Report the lower PC25 Z reference')

    def _handle_connect(self):
        self.kinematics = (
            self.printer.lookup_object('toolhead').get_kinematics())
        rail = self.kinematics.rails[2]
        self.endstop = rail.get_endstops()[0][0]
        self.normal_z_range = rail.get_range()
        if not (self.normal_z_range[0] <= self.calibration_return_position
                <= self.normal_z_range[1]):
            raise self.printer.config_error(
                'calibration_return_position must be inside the normal Z '
                'range')

    def _set_calibration_limit(self):
        self.kinematics.limits[2] = (
            self.normal_z_range[0], self.temporary_position_max)

    def _restore_normal_limit(self):
        self.kinematics.limits[2] = self.normal_z_range

    def _return_to_normal_range(self):
        toolhead = self.printer.lookup_object('toolhead')
        if toolhead.get_position()[2] <= self.normal_z_range[1]:
            return
        pos = toolhead.get_position()
        pos[2] = self.normal_z_range[1]
        toolhead.manual_move(pos, self.speed)
        toolhead.wait_moves()

    def _return_after_calibration(self):
        self._return_to_normal_range()
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        pos[2] = self.calibration_return_position
        toolhead.manual_move(pos, self.travel_speed)
        toolhead.wait_moves()

    def _require_idle(self):
        eventtime = self.printer.get_reactor().monotonic()
        idle = self.printer.lookup_object('idle_timeout').get_status(eventtime)
        if idle['state'] == 'Printing':
            raise self.printer.command_error(
                'Z reference commands are not permitted while printing')

    def _probe_lower(self, start_position, require_homed):
        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.printer.get_reactor().monotonic()
        homed = toolhead.get_status(eventtime)['homed_axes']
        if require_homed and 'z' not in homed:
            raise self.printer.command_error(
                'Upper Z must be homed before calibrating the lower reference')
        if require_homed:
            pos = toolhead.get_position()
            pos[2] = start_position
            toolhead.manual_move(pos, self.travel_speed)
            toolhead.wait_moves()
        else:
            pos = toolhead.get_position()
            pos[2] = start_position
            toolhead.set_position(pos, homing_axes='z')
        self._set_calibration_limit()
        print_time = toolhead.get_last_move_time()
        if self.endstop.query_endstop(print_time):
            if require_homed:
                raise self.printer.command_error(
                    'PC25 is already triggered before lower Z homing')
            retract = toolhead.get_position()
            retract[2] -= self.recovery_release_distance
            toolhead.manual_move(retract, self.speed)
            toolhead.wait_moves()
            print_time = toolhead.get_last_move_time()
            if self.endstop.query_endstop(print_time):
                raise self.printer.command_error(
                    'PC25 is still triggered after lower recovery retract')
        target = toolhead.get_position()
        target[2] = self.search_target
        hmove = homing.HomingMove(
            self.printer, [(self.endstop, 'lower_z')], toolhead)
        trigger = hmove.homing_move(
            target, self.speed, probe_pos=True, triggered=True)
        first_z = trigger[2]
        retract = toolhead.get_position()
        retract[2] = first_z - self.retract_distance
        toolhead.manual_move(retract, self.speed)
        toolhead.wait_moves()
        target = toolhead.get_position()
        target[2] = self.search_target
        hmove = homing.HomingMove(
            self.printer, [(self.endstop, 'lower_z')], toolhead)
        return hmove.homing_move(
            target, self.second_speed, probe_pos=True,
            triggered=True)[2]

    def cmd_CALIBRATE_LOWER_Z_REFERENCE(self, gcmd):
        self._require_idle()
        try:
            measured = self._probe_lower(self.search_start, True)
            self.last_measured = measured
            self.lower_position = measured
            self.last_action = 'calibrated'
            configfile = self.printer.lookup_object('configfile')
            configfile.set(
                'flow_z_reference', 'lower_position', '%.6f' % measured)
            self._return_after_calibration()
            gcmd.respond_info(
                'Lower PC25 reference measured at Z=%.6f. Run SAVE_CONFIG '
                'to store it.' % measured)
        finally:
            try:
                self._return_to_normal_range()
            finally:
                self._restore_normal_limit()

    def cmd_HOME_Z_FROM_LOWER_REFERENCE(self, gcmd):
        self._require_idle()
        if self.lower_position is None:
            raise gcmd.error(
                'No lower_position is configured; calibrate it first')
        try:
            self._probe_lower(self.search_start, False)
            toolhead = self.printer.lookup_object('toolhead')
            pos = toolhead.get_position()
            pos[2] = self.lower_position
            toolhead.set_position(pos, homing_axes='z')
            self._set_calibration_limit()
            self._return_to_normal_range()
            self.last_action = 'recovery_homed'
            gcmd.respond_info(
                'Recovery Z home complete at lower PC25 reference Z=%.6f'
                % self.lower_position)
        finally:
            try:
                self._return_to_normal_range()
            finally:
                self._restore_normal_limit()

    def cmd_QUERY_FLOW_Z_REFERENCE(self, gcmd):
        if self.lower_position is None:
            gcmd.respond_info('Lower PC25 reference is not calibrated')
            return
        gcmd.respond_info(
            'Lower PC25 reference Z=%.6f; last action=%s'
            % (self.lower_position, self.last_action))

    def get_status(self, eventtime):
        return {
            'lower_position': self.lower_position,
            'last_measured': self.last_measured,
            'last_action': self.last_action,
        }


def load_config(config):
    return FlowZReference(config)
