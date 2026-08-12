# IDEX nozzle XY calibration using a shared digital bed-contact input
#
# Copyright (C) 2026  Klipper developers
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math

from . import homing


class IdexXYCalibration:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        ppins = self.printer.lookup_object('pins')
        self.endstop = ppins.setup_pin(
            'endstop', config.get('pin', 'bed_presence:virtual_endstop'))
        self.printer.register_event_handler(
            'klippy:mcu_identify', self._handle_mcu_identify)

        # User scripts own heating, cleaning, and any machine-specific parking.
        self.left_prepare = config.get('left_prepare_gcode', '')
        self.right_prepare = config.get('right_prepare_gcode', '')
        self.left_finish = config.get('left_finish_gcode', '')
        self.right_finish = config.get('right_finish_gcode', '')
        self.abort_gcode = config.get('abort_gcode', 'TURN_OFF_HEATERS')
        self.left_activate = config.get(
            'left_activate_gcode',
            'SET_DUAL_CARRIAGE CARRIAGE=0 MODE=PRIMARY\n'
            'ACTIVATE_EXTRUDER EXTRUDER=extruder')
        self.right_activate = config.get(
            'right_activate_gcode',
            'SET_DUAL_CARRIAGE CARRIAGE=1 MODE=PRIMARY\n'
            'ACTIVATE_EXTRUDER EXTRUDER=extruder1')

        self.edge_y = config.getfloat('edge_y', 10.)
        self.hole_y = config.getfloat('hole_y', 8.)
        self.left_edge_start = config.getfloat('left_edge_start', 6.)
        self.right_edge_start = config.getfloat('right_edge_start')
        self.left_edge_target = config.getfloat('left_edge_target', 0.)
        self.right_edge_target = config.getfloat('right_edge_target')
        self.temporary_x_min = config.getfloat(
            'temporary_x_min', self.left_edge_target,
            maxval=self.left_edge_target)
        self.temporary_x_max = config.getfloat(
            'temporary_x_max', self.right_edge_target,
            minval=self.right_edge_target)
        self.left_reference_edge = config.getfloat(
            'left_reference_edge', 0.)
        self.right_reference_edge = config.getfloat(
            'right_reference_edge', 446.)
        self.left_z_test_x = config.getfloat('left_z_test_x', 15.)
        self.right_z_test_x = config.getfloat('right_z_test_x', 431.)
        self.z_test_y = config.getfloat('z_test_y', self.edge_y)
        self.edge_scan_z = config.getfloat('edge_scan_z')
        self.safe_z = config.getfloat('safe_z')
        self.z_probe_target = config.getfloat('z_probe_target')
        self.surface_inset = config.getfloat('surface_inset', 15., above=0.)
        self.hole_inset = config.getfloat('hole_inset', 215., above=0.)
        self.hole_depth = config.getfloat('hole_depth', 3., above=0.)
        self.hole_search_distance = config.getfloat(
            'hole_search_distance', above=0.)
        self.plate_check_x = config.getfloat('plate_check_x', 50.)
        self.plate_check_y = config.getfloat('plate_check_y', 50.)

        self.travel_speed = config.getfloat('travel_speed', 100., above=0.)
        self.z_speed = config.getfloat('z_speed', 5., above=0.)
        self.edge_speed = config.getfloat('edge_speed', 2., above=0.)
        self.hole_speed = config.getfloat('hole_speed', 1., above=0.)
        self.z_retract = config.getfloat('z_retract', 1., above=0.)
        self.z_release_step = config.getfloat(
            'z_release_step', .05, above=0., maxval=self.z_retract)
        self.nozzle_wipe_distance = config.getfloat(
            'nozzle_wipe_distance', 1., minval=0.)
        self.contact_dwell = config.getfloat(
            'contact_dwell', 0., minval=0.)
        self.cleaning_half_range = config.getfloat(
            'cleaning_half_range', .5, minval=0.)
        self.cleaning_cycles = config.getint(
            'cleaning_cycles', 3, minval=0)
        self.cleaning_speed = config.getfloat(
            'cleaning_speed', 5., above=0.)
        self.left_park_x = config.getfloat('left_park_x', 0.)
        self.right_park_x = config.getfloat('right_park_x', 446.)
        self.xy_retract = config.getfloat('xy_retract', 1., above=0.)
        self.z_tolerance = config.getfloat(
            'z_tolerance', .02, minval=0.)
        self.stable_samples = config.getint('stable_samples', 3, minval=2)
        self.max_z_samples = config.getint(
            'max_z_samples', 12, minval=self.stable_samples)
        self.hole_samples = config.getint('hole_samples', 1, minval=1)
        self.hole_tolerance = config.getfloat(
            'hole_tolerance', .03, minval=0.)

        self.running = False
        self.state = 'idle'
        self.last_error = None
        self.left_result = None
        self.right_result = None
        self.offset = None
        self.bed_edges = None
        self.z_offset = None
        self.kinematics = None
        self.normal_x_range = None
        self.gcode.register_command(
            'IDEX_XY_CALIBRATE', self.cmd_IDEX_XY_CALIBRATE,
            desc=self.cmd_IDEX_XY_CALIBRATE_help)
        self.gcode.register_command(
            'QUERY_IDEX_XY_CALIBRATION',
            self.cmd_QUERY_IDEX_XY_CALIBRATION,
            desc=self.cmd_QUERY_IDEX_XY_CALIBRATION_help)
        self.gcode.register_command(
            'CALIBRATE_BED_EDGES', self.cmd_CALIBRATE_BED_EDGES,
            desc='Measure the left and right build-plate edges')
        self.gcode.register_command(
            'CALIBRATE_IDEX_Z_OFFSET', self.cmd_CALIBRATE_IDEX_Z_OFFSET,
            desc='Measure the T1-to-T0 nozzle Z offset')
        self.gcode.register_command(
            'CHECK_BUILD_PLATE_ORIENTATION',
            self.cmd_CHECK_BUILD_PLATE_ORIENTATION,
            desc='Verify nozzle contact with the installed build plate')
        self.gcode.register_command(
            'QUERY_BED_CONTACT', self.cmd_QUERY_BED_CONTACT,
            desc='Report the native digital PC27 contact state')

    def _handle_mcu_identify(self):
        self.kinematics = (
            self.printer.lookup_object('toolhead').get_kinematics())
        self.normal_x_range = self.kinematics.rails[0].get_range()
        for stepper in self.kinematics.get_steppers():
            self.endstop.add_stepper(stepper)

    def _set_calibration_x_limit(self):
        self.kinematics.limits[0] = (
            self.temporary_x_min, self.temporary_x_max)

    def _restore_normal_x_limit(self):
        self.kinematics.limits[0] = self.normal_x_range

    def _return_to_normal_motion_range(self):
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        if pos[2] < self.safe_z:
            self._move(z=self.safe_z, speed=self.z_speed)
            pos = toolhead.get_position()
        target_x = min(max(pos[0], self.normal_x_range[0]),
                       self.normal_x_range[1])
        if target_x != pos[0]:
            self._move(x=target_x, speed=self.travel_speed)

    def _run_script(self, script):
        if script.strip():
            self.gcode.run_script_from_command(script)

    def _move(self, x=None, y=None, z=None, speed=None):
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        if x is not None:
            pos[0] = x
        if y is not None:
            pos[1] = y
        if z is not None:
            pos[2] = z
        toolhead.manual_move(pos, speed or self.travel_speed)
        toolhead.wait_moves()

    def _digital_state(self):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.wait_moves()
        print_time = toolhead.get_last_move_time()
        return bool(self.endstop.query_endstop(print_time))

    def _probe_move(self, x=None, y=None, z=None, speed=None,
                    triggered=False, label='contact'):
        toolhead = self.printer.lookup_object('toolhead')
        current = toolhead.get_position()
        target = list(current)
        for axis, value in enumerate((x, y, z)):
            if value is not None:
                target[axis] = value
        if self._digital_state() == triggered:
            raise self.printer.command_error(
                '%s sensor is already in its trigger state' % (label,))
        hmove = homing.HomingMove(
            self.printer, [(self.endstop, label)], toolhead)
        return hmove.homing_move(
            target, speed or self.hole_speed, probe_pos=True,
            triggered=triggered, check_triggered=True)

    def _release_z_contact(self, direction):
        toolhead = self.printer.lookup_object('toolhead')
        contact_z = toolhead.get_position()[2]
        release_direction = -direction
        travelled = 0.
        while not self._digital_state():
            travelled = min(travelled + self.z_release_step,
                            self.z_retract)
            self._move(
                z=contact_z + release_direction * travelled,
                speed=self.z_speed)
            if travelled >= self.z_retract and not self._digital_state():
                raise self.printer.command_error(
                    'Bed contact did not release within %.4fmm'
                    % (self.z_retract,))
        return toolhead.get_position()[2]

    def _wipe_nozzle(self, tool):
        if not self.nozzle_wipe_distance:
            return
        x = self.printer.lookup_object('toolhead').get_position()[0]
        direction = 1. if tool == 0 else -1.
        self._move(
            x=x + direction * self.nozzle_wipe_distance,
            speed=self.travel_speed)

    def _oscillate_nozzle(self):
        if not self.cleaning_half_range or not self.cleaning_cycles:
            return
        center = self.printer.lookup_object('toolhead').get_position()[0]
        for unused in range(self.cleaning_cycles):
            self._move(
                x=center - self.cleaning_half_range,
                speed=self.cleaning_speed)
            self._move(
                x=center + self.cleaning_half_range,
                speed=self.cleaning_speed)
        self._move(x=center, speed=self.cleaning_speed)

    def _probe_stable_z(self, tool):
        direction = math.copysign(1., self.z_probe_target - self.safe_z)
        consecutive = []
        for unused in range(self.max_z_samples):
            self._probe_move(
                z=self.z_probe_target, speed=self.z_speed,
                triggered=False, label='bed contact')
            self._oscillate_nozzle()
            if self.contact_dwell:
                self._run_script('G4 P%d' % (self.contact_dwell * 1000.,))
            # Cleaning may remove the filament contact.  If it does, touch the
            # now-clean nozzle again so the Z release coordinate remains valid.
            if self._digital_state():
                self._probe_move(
                    z=self.z_probe_target, speed=self.z_speed,
                    triggered=False, label='clean nozzle contact')
            value = self._release_z_contact(direction)
            consecutive.append(value)
            if len(consecutive) > self.stable_samples:
                consecutive.pop(0)
            if (len(consecutive) == self.stable_samples
                    and max(consecutive) - min(consecutive)
                    <= self.z_tolerance):
                self._wipe_nozzle(tool)
                return sum(consecutive) / self.stable_samples
        raise self.printer.command_error(
            'Unable to obtain %d consecutive Z samples within %.4fmm'
            % (self.stable_samples, self.z_tolerance))

    def _probe_axis_once(self, axis, center, direction):
        target = center + direction * self.hole_search_distance
        kwargs = {'x': None, 'y': None}
        kwargs[axis] = target
        pos = self._probe_move(
            speed=self.hole_speed, triggered=False,
            label='hole %s%s' % (axis, '+' if direction > 0 else '-'),
            **kwargs)
        value = pos[0 if axis == 'x' else 1]
        retract = value - direction * self.xy_retract
        self._move(speed=self.travel_speed, **{axis: retract})
        self._move(speed=self.travel_speed, **{axis: center})
        return value

    def _probe_axis(self, axis, center, direction):
        values = [self._probe_axis_once(axis, center, direction)
                  for unused in range(self.hole_samples)]
        if max(values) - min(values) > self.hole_tolerance:
            raise self.printer.command_error(
                'Hole %s measurements exceed tolerance: %s'
                % (axis, ', '.join('%.4f' % v for v in values)))
        return sum(values) / len(values)

    def _measure_hole(self, center_x, center_y):
        y_plus = self._probe_axis('y', center_y, 1.)
        y_minus = self._probe_axis('y', center_y, -1.)
        measured_y = .5 * (y_plus + y_minus)
        self._move(y=measured_y)
        x_plus = self._probe_axis('x', center_x, 1.)
        x_minus = self._probe_axis('x', center_x, -1.)
        measured_x = .5 * (x_plus + x_minus)
        return {
            'center_x': measured_x, 'center_y': measured_y,
            'x_minus': x_minus, 'x_plus': x_plus,
            'y_minus': y_minus, 'y_plus': y_plus,
        }

    def _measure_tool(self, tool):
        left = tool == 0
        self.state = 'left' if left else 'right'
        self._run_script(self.left_activate if left else self.right_activate)
        self._set_calibration_x_limit()
        self._run_script(self.left_prepare if left else self.right_prepare)
        edge_start = self.left_edge_start if left else self.right_edge_start
        edge_target = self.left_edge_target if left else self.right_edge_target
        edge_sign = 1. if left else -1.

        self._move(z=self.safe_z, speed=self.z_speed)
        self._move(x=edge_start, y=self.edge_y)
        self._move(z=self.edge_scan_z, speed=self.z_speed)
        # The nozzle starts touching the plate; HIGH means it left the edge.
        edge = self._probe_move(
            x=edge_target, speed=self.edge_speed, triggered=True,
            label='%s bed edge' % ('left' if left else 'right'))[0]
        self._move(z=self.safe_z, speed=self.z_speed)

        surface_x = edge + edge_sign * self.surface_inset
        self._move(x=surface_x, y=self.edge_y)
        reference_z = self._probe_stable_z(tool)
        self._move(z=self.safe_z, speed=self.z_speed)

        hole_x = edge + edge_sign * self.hole_inset
        self._move(x=hole_x, y=self.hole_y)
        z_direction = math.copysign(1., self.z_probe_target - self.safe_z)
        self._move(
            z=reference_z + z_direction * self.hole_depth,
            speed=self.z_speed)
        result = self._measure_hole(hole_x, self.hole_y)
        result.update({'edge_x': edge, 'reference_z': reference_z})
        self._move(z=self.safe_z, speed=self.z_speed)
        self._move(x=self.left_park_x if left else self.right_park_x)
        self._run_script(self.left_finish if left else self.right_finish)
        return result

    def _measure_edge(self, tool):
        left = tool == 0
        self._run_script(self.left_activate if left else self.right_activate)
        self._set_calibration_x_limit()
        edge_start = self.left_edge_start if left else self.right_edge_start
        edge_target = self.left_edge_target if left else self.right_edge_target
        self._move(z=self.safe_z, speed=self.z_speed)
        self._move(x=edge_start, y=self.edge_y)
        self._move(z=self.edge_scan_z, speed=self.z_speed)
        edge = self._probe_move(
            x=edge_target, speed=self.edge_speed, triggered=True,
            label='%s bed edge' % ('left' if left else 'right'))[0]
        self._move(z=self.safe_z, speed=self.z_speed)
        return edge

    def _measure_tool_z(self, tool, edge):
        left = tool == 0
        self._run_script(self.left_activate if left else self.right_activate)
        self._set_calibration_x_limit()
        self._run_script(self.left_prepare if left else self.right_prepare)
        self._move(z=self.safe_z, speed=self.z_speed)
        self._move(
            x=self.left_z_test_x if left else self.right_z_test_x,
            y=self.z_test_y)
        value = self._probe_stable_z(tool)
        self._move(z=self.safe_z, speed=self.z_speed)
        self._move(x=self.left_park_x if left else self.right_park_x)
        self._run_script(self.left_finish if left else self.right_finish)
        return value

    def _preflight(self):
        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.printer.get_reactor().monotonic()
        homed = toolhead.get_status(eventtime)['homed_axes']
        if not all(axis in homed for axis in 'xyz'):
            raise self.printer.command_error(
                'IDEX XY calibration requires homed X, Y, and Z')
        idle = self.printer.lookup_object('idle_timeout').get_status(eventtime)
        if idle['state'] == 'Printing':
            raise self.printer.command_error(
                'IDEX XY calibration is not permitted while printing')
        self.gcode.run_script_from_command('REQUIRE_BED_PRESENT')

    cmd_IDEX_XY_CALIBRATE_help = \
        'Measure the IDEX nozzle XY offset using the bed reference hole'

    def cmd_IDEX_XY_CALIBRATE(self, gcmd):
        if self.running:
            raise gcmd.error('IDEX XY calibration is already running')
        self._preflight()
        self.running = True
        self.last_error = None
        self.left_result = self.right_result = self.offset = None
        try:
            self.left_result = self._measure_tool(0)
            self.right_result = self._measure_tool(1)
            self.offset = {
                'x': (self.right_result['center_x']
                      - self.left_result['center_x']),
                'y': (self.right_result['center_y']
                      - self.left_result['center_y']),
            }
            self.state = 'complete'
            gcmd.respond_info(
                'IDEX XY calibration complete: left=(%.4f, %.4f), '
                'right=(%.4f, %.4f), right-left offset=(%.4f, %.4f)'
                % (self.left_result['center_x'], self.left_result['center_y'],
                   self.right_result['center_x'], self.right_result['center_y'],
                   self.offset['x'], self.offset['y']))
        except self.printer.command_error as e:
            self.state = 'error'
            self.last_error = str(e)
            try:
                self._run_script(self.abort_gcode)
            except self.printer.command_error:
                pass
            raise
        finally:
            try:
                self._return_to_normal_motion_range()
            finally:
                self._restore_normal_x_limit()
            self.running = False

    def cmd_CALIBRATE_BED_EDGES(self, gcmd):
        self._preflight()
        if self.running:
            raise gcmd.error('A calibration is already running')
        self.running = True
        try:
            self.bed_edges = {
                'left': self._measure_edge(0),
                'right': self._measure_edge(1),
            }
            gcmd.respond_info(
                'Bed edges: left=%.4f right=%.4f'
                % (self.bed_edges['left'], self.bed_edges['right']))
        except self.printer.command_error as e:
            self.last_error = str(e)
            self._run_script(self.abort_gcode)
            raise
        finally:
            try:
                self._return_to_normal_motion_range()
            finally:
                self._restore_normal_x_limit()
            self.running = False

    def cmd_CALIBRATE_IDEX_Z_OFFSET(self, gcmd):
        self._preflight()
        if self.running:
            raise gcmd.error('A calibration is already running')
        self.running = True
        try:
            if self.bed_edges is None:
                self.bed_edges = {
                    'left': self.left_reference_edge,
                    'right': self.right_reference_edge,
                }
            left_z = self._measure_tool_z(0, self.bed_edges['left'])
            right_z = self._measure_tool_z(1, self.bed_edges['right'])
            self.z_offset = right_z - left_z
            gcmd.respond_info(
                'IDEX Z calibration: left=%.4f right=%.4f '
                'right-left=%.4f'
                % (left_z, right_z, self.z_offset))
        except self.printer.command_error as e:
            self.last_error = str(e)
            self._run_script(self.abort_gcode)
            raise
        finally:
            try:
                self._return_to_normal_motion_range()
            finally:
                self._restore_normal_x_limit()
            self.running = False

    def cmd_CHECK_BUILD_PLATE_ORIENTATION(self, gcmd):
        self._preflight()
        self._run_script(self.left_activate)
        self._move(z=self.safe_z, speed=self.z_speed)
        self._move(x=self.plate_check_x, y=self.plate_check_y)
        contact = self._probe_move(
            z=self.z_probe_target, speed=self.z_speed, triggered=False,
            label='build plate orientation')[2]
        self._move(z=self.safe_z, speed=self.z_speed)
        gcmd.respond_info(
            'Build plate contact verified at Z=%.4f' % contact)

    def cmd_QUERY_BED_CONTACT(self, gcmd):
        state = self._digital_state()
        gcmd.respond_info(
            'Bed contact digital state: %s (%s)'
            % ('HIGH' if state else 'LOW',
               'released' if state else 'contact'))

    cmd_QUERY_IDEX_XY_CALIBRATION_help = \
        'Report the last IDEX XY calibration result'

    def cmd_QUERY_IDEX_XY_CALIBRATION(self, gcmd):
        if self.offset is None:
            gcmd.respond_info(
                'IDEX XY calibration state: %s%s'
                % (self.state, '' if self.last_error is None
                   else ', error: ' + self.last_error))
            return
        gcmd.respond_info(
            'IDEX XY offset right-left: X=%.4f Y=%.4f'
            % (self.offset['x'], self.offset['y']))

    def get_status(self, eventtime):
        return {
            'state': self.state, 'running': self.running,
            'last_error': self.last_error,
            'left': self.left_result, 'right': self.right_result,
            'offset': self.offset,
            'bed_edges': self.bed_edges,
            'z_offset': self.z_offset,
        }


def load_config(config):
    return IdexXYCalibration(config)
