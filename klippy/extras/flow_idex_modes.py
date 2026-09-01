# FLOW IDEX print-mode controller
#
# Copyright (C) 2026  Oliver
#
# This file may be distributed under the terms of the GNU GPLv3 license.

MODE_NORMAL = 'NORMAL'
MODE_PARALLEL = 'PARALLEL'
MODE_MIRROR = 'MIRROR'
MODE_BACKUP = 'BACKUP'
SYNC_MODES = (MODE_PARALLEL, MODE_MIRROR)


class FlowIdexModes:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.left_park_x = config.getfloat('left_park_x', 0.)
        self.right_park_x = config.getfloat('right_park_x', 446.)
        self.parallel_separation = config.getfloat(
            'parallel_separation', 223., above=0.)
        self.synchronized_separation = self.parallel_separation
        self.travel_speed = config.getfloat('travel_speed', 100., above=0.)
        self.minimum_gap_ratio = config.getfloat(
            'minimum_gap_ratio', .5, minval=0., maxval=1.)
        self.layer_z_tolerance = config.getfloat(
            'layer_z_tolerance', .05, above=0.)
        self.adaptive_bed_stabilization_time = config.getfloat(
            'adaptive_bed_stabilization_time', 60., minval=0.)
        self.adaptive_park_y = config.getfloat('adaptive_park_y', 155.)
        self.offset_prefix = config.get('offset_prefix', 'idex_t1')
        self.require_calibrated_offsets = config.getboolean(
            'require_calibrated_offsets', False)
        self.sensor_names = config.getlist(
            'sensors', ('h0_filament', 'h1_filament'))
        if len(self.sensor_names) != 2:
            raise config.error('flow_idex_modes requires exactly two sensors')
        self.cleaning_gcodes = config.getlist(
            'cleaning_gcodes', ('NOZZLE_CLEANING', 'NOZZLE_CLEANING'))
        if len(self.cleaning_gcodes) != 2:
            raise config.error(
                'flow_idex_modes requires exactly two cleaning_gcodes')
        self.prime_distance = config.getfloat('prime_distance', 5., minval=0.)
        self.prime_speed = config.getfloat('prime_speed', 1.5, above=0.)
        self.mode = MODE_NORMAL
        self.active_tool = 0
        self.backup_enabled = False
        self.failover_latched = False
        self.failed_tool = None
        self.first_layer_height = None
        self.first_layer_active = False
        self.first_layer_z = None
        self.flow_factors = [1., 1.]
        self.gaps = [0., 0.]
        self.base_z_correction = 0.
        self.recovery_tool = None
        self.pending_mode = None
        self.pending_activation_scheduled = False
        self.previous_mesh = None
        self.adaptive_mesh_active = False
        self.adaptive_nozzle_hold_active = False
        self.deferred_nozzle_targets = [0., 0.]
        self.deferred_nozzle_waits = [False, False]
        self.offsets = [0., 0., 0.]
        self.offsets_valid = False
        self.toolhead = self.dual_carriage = self.bed_mesh = None
        self.extruders = []
        self.rails = []
        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)
        self.printer.register_event_handler(
            'print_stats:layer_changed', self._handle_layer_changed)
        self.printer.register_event_handler(
            'print_stats:state_changed', self._handle_print_state_changed)
        self.printer.register_event_handler(
            'homing:home_rails_end', self._handle_home_rails_end)
        self.gcode.register_command(
            'SET_FLOW_MODE', self.cmd_SET_FLOW_MODE,
            desc='Set the guarded FLOW IDEX print mode')
        self.gcode.register_command(
            'SET_FLOW_LAYER', self.cmd_SET_FLOW_LAYER,
            desc='Update FLOW IDEX layer state')
        self.gcode.register_command(
            'RESET_FLOW_FAILOVER', self.cmd_RESET_FLOW_FAILOVER,
            desc='Clear the latched FLOW backup failover while idle')
        self.gcode.register_command(
            'PARK_FLOW_HEADS', self.cmd_PARK_FLOW_HEADS,
            desc='Park both FLOW IDEX heads for a paused print')
        self.gcode.register_command(
            'CLEAN_FLOW_HEADS', self.cmd_CLEAN_FLOW_HEADS,
            desc='Clean the active FLOW IDEX head or synchronized pair')

    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self.gcode_move = self.printer.lookup_object('gcode_move')
        self.dual_carriage = self.printer.lookup_object('dual_carriage')
        self.bed_mesh = self.printer.lookup_object('bed_mesh')
        self.bed_mesh.register_mesh_lookup(self._mesh_lookup)
        self.rails = list(self.dual_carriage.dc_rails.values())
        if len(self.rails) != 2:
            raise self.printer.config_error(
                'flow_idex_modes requires exactly two X carriages')
        variables = self.printer.lookup_object('save_variables').allVariables
        prefix = self.offset_prefix
        names = [prefix + '_x_offset', prefix + '_y_offset',
                 prefix + '_z_offset']
        self.offsets_valid = bool(variables.get('idex_offsets_valid', 0)) \
            and all(name in variables for name in names)
        if self.offsets_valid:
            self.offsets = [float(variables[name]) for name in names]
        self.extruders = [self.printer.lookup_object('extruder'),
                          self.printer.lookup_object('extruder1')]
        self.extruders[0].set_extrusion_scale_provider(
            lambda move, axis: self._extrusion_scale(0, move, axis))
        self.extruders[1].set_extrusion_scale_provider(
            lambda move, axis: self._extrusion_scale(1, move, axis))
        self.extruders[0].register_motion_follower(self._follow_t0_motion)

    def _require_homed(self, gcmd):
        if not self._is_homed(self.printer.get_reactor().monotonic()):
            raise gcmd.error('FLOW IDEX mode changes require homed XYZ axes')

    def _is_homed(self, eventtime):
        homed = self.toolhead.get_status(eventtime).get('homed_axes', '')
        return all(axis in homed for axis in 'xyz')

    def _require_offsets(self, gcmd):
        if self.require_calibrated_offsets and not self.offsets_valid:
            raise gcmd.error('FLOW IDEX offsets are not calibrated')

    def should_heat_both(self):
        pending = self.pending_mode
        return self.mode in SYNC_MODES or (pending is not None
                                           and pending['mode'] in SYNC_MODES)

    def _begin_adaptive_nozzle_hold(self):
        eventtime = self.printer.get_reactor().monotonic()
        self.adaptive_nozzle_hold_active = True
        self.deferred_nozzle_waits = [False, False]
        self.deferred_nozzle_targets = []
        for extruder in self.extruders:
            heater = extruder.get_heater()
            self.deferred_nozzle_targets.append(heater.get_temp(eventtime)[1])
            heater.set_temp(0.)

    def defer_nozzle_temperature(self, heater, temp, wait):
        if not self.adaptive_nozzle_hold_active:
            return False
        for index, extruder in enumerate(self.extruders):
            if heater is not extruder.get_heater():
                continue
            self.deferred_nozzle_targets[index] = temp
            self.deferred_nozzle_waits[index] = bool(wait and temp)
            return True
        return False

    def _finish_adaptive_nozzle_hold(self, restore):
        if not self.adaptive_nozzle_hold_active:
            return
        targets = self.deferred_nozzle_targets
        waits = self.deferred_nozzle_waits
        self.adaptive_nozzle_hold_active = False
        self.deferred_nozzle_targets = [0., 0.]
        self.deferred_nozzle_waits = [False, False]
        if not restore:
            return
        pheaters = self.printer.lookup_object('heaters')
        for index, extruder in enumerate(self.extruders):
            pheaters.set_temperature(
                extruder.get_heater(), targets[index], False)
        for index, extruder in enumerate(self.extruders):
            if waits[index]:
                pheaters.set_temperature(
                    extruder.get_heater(), targets[index], True)

    def _physical_positions(self, x):
        if self.mode == MODE_PARALLEL:
            return [x, x + self.synchronized_separation]
        if self.mode == MODE_MIRROR:
            return [x, self.left_park_x + self.right_park_x - x]
        position = [x, 0., 0., 0.]
        return [rail.get_axis_position(position) for rail in self.rails]

    def _calculate_correction(self, required):
        correction = .5 * (required[0] + required[1])
        if self.first_layer_active:
            minimum_gap = self.first_layer_height * self.minimum_gap_ratio
            self.gaps = [self.first_layer_height + correction - value
                         for value in required]
            if min(self.gaps) < minimum_gap:
                correction += minimum_gap - min(self.gaps)
                self.gaps = [self.first_layer_height + correction - value
                             for value in required]
            if min(self.gaps) < -1.e-6:
                raise self.printer.command_error(
                    'FLOW IDEX calculated a negative nozzle gap')
            self.flow_factors = [max(0., gap / self.first_layer_height)
                                 for gap in self.gaps]
        return correction

    def _mesh_lookup(self, raw_lookup, x, y):
        if self.mode not in SYNC_MODES:
            return raw_lookup(x, y)
        x0, x1 = self._physical_positions(x)
        required = [raw_lookup(x0, y),
                    raw_lookup(x1, y + self.offsets[1]) - self.offsets[2]]
        correction = self._calculate_correction(required)
        return correction - self.base_z_correction

    def _extrusion_scale(self, tool, move, ea_index):
        if tool == 0:
            self._track_deposition_layer(move, ea_index)
        if (self.mode not in SYNC_MODES or not self.first_layer_active
                or move.axes_r[ea_index] <= 0.
                or not (move.axes_d[0] or move.axes_d[1])):
            return 1.
        scale = self.flow_factors[tool]
        if move.axes_r[ea_index] * scale \
                > self.extruders[tool].max_extrude_ratio:
            raise self.printer.command_error(
                'FLOW IDEX first-layer flow exceeds T%d maximum' % (tool,))
        return scale

    def _track_deposition_layer(self, move, ea_index):
        if (self.mode not in SYNC_MODES or not self.first_layer_active
                or move.axes_r[ea_index] <= 0.
                or not (move.axes_d[0] or move.axes_d[1])):
            return
        z = self.gcode_move.last_position[2]
        if self.first_layer_z is None or z < self.first_layer_z:
            self.first_layer_z = z
            return
        if z > self.first_layer_z + self.layer_z_tolerance:
            self._handle_layer_changed(2)

    def _follow_t0_motion(self, print_time, move, ea_index):
        if self.mode not in SYNC_MODES or self.recovery_tool is not None:
            return
        if move.axes_r[ea_index] > 0. \
                and not self.extruders[1].heater.can_extrude:
            raise self.printer.command_error(
                'FLOW IDEX T1 is below minimum extrusion temperature')
        scale = self._extrusion_scale(1, move, ea_index)
        self.extruders[1].append_follow_move(
            print_time, move, ea_index, scale)

    def expand_adaptive_points(self, points):
        if self.mode not in SYNC_MODES or not points:
            return points
        expanded = []
        for x, y in points:
            x0, x1 = self._physical_positions(x)
            expanded.append((x0, y))
            expanded.append((x1, y + self.offsets[1]))
        return expanded

    def is_synchronized(self):
        return self.mode in SYNC_MODES

    def park_for_recovery(self, pause_x, pause_y, speed):
        if self.mode in SYNC_MODES:
            self._run(
                'SET_DUAL_CARRIAGE CARRIAGE=1 MODE=PRIMARY\n'
                'G90\nG1 X%.6f Y%.6f F%.3f\n'
                'SET_DUAL_CARRIAGE CARRIAGE=0 MODE=PRIMARY\n'
                'G1 X%.6f F%.3f\nM400'
                % (self.right_park_x, pause_y, speed * 60.,
                   pause_x, speed * 60.))
            return
        active_x = pause_x if self.active_tool == 0 else self.right_park_x
        self._run('G90\nG1 X%.6f Y%.6f F%.3f\nM400'
                  % (active_x, pause_y, speed * 60.))

    def park_heads(self, pause_x, pause_y, speed):
        synchronized = self.mode in SYNC_MODES
        self.park_for_recovery(pause_x, pause_y, speed)
        if synchronized:
            native_mode = 'COPY' if self.mode == MODE_PARALLEL else 'MIRROR'
            self._run('SET_DUAL_CARRIAGE CARRIAGE=1 MODE=%s'
                      % (native_mode,))

    def cmd_PARK_FLOW_HEADS(self, gcmd):
        self._require_homed(gcmd)
        pause_x = gcmd.get_float('X', self.left_park_x)
        pause_y = gcmd.get_float('Y', 155.)
        speed = gcmd.get_float('SPEED', self.travel_speed, above=0.)
        self.park_heads(pause_x, pause_y, speed)

    def clean_heads_for_resume(self):
        synchronized = self.mode in SYNC_MODES
        tools = (0, 1) if synchronized else (self.active_tool,)
        original_tool = self.active_tool
        try:
            for tool in tools:
                self._run(
                    'SET_DUAL_CARRIAGE CARRIAGE=%d MODE=PRIMARY\n'
                    'ACTIVATE_EXTRUDER EXTRUDER=%s\n%s'
                    % (tool, self.extruders[tool].get_name(),
                       self.cleaning_gcodes[tool]))
        finally:
            restore = 'ACTIVATE_EXTRUDER EXTRUDER=%s' \
                % (self.extruders[original_tool].get_name(),)
            if synchronized:
                native_mode = ('COPY' if self.mode == MODE_PARALLEL
                               else 'MIRROR')
                restore += (
                    '\nSET_DUAL_CARRIAGE CARRIAGE=0 MODE=PRIMARY'
                    '\nSET_DUAL_CARRIAGE CARRIAGE=1 MODE=%s'
                    % (native_mode,))
            self._run(restore)

    def cmd_CLEAN_FLOW_HEADS(self, gcmd):
        self._require_homed(gcmd)
        self.clean_heads_for_resume()

    def begin_recovery(self, sensor_name):
        if self.mode not in SYNC_MODES:
            return
        if sensor_name not in self.sensor_names:
            raise self.printer.command_error(
                'Unknown FLOW recovery sensor %s' % (sensor_name,))
        self.recovery_tool = self.sensor_names.index(sensor_name)
        self._run('ACTIVATE_EXTRUDER EXTRUDER=%s'
                  % ('extruder1' if self.recovery_tool else 'extruder',))

    def end_recovery(self):
        if self.recovery_tool is None:
            return
        self._run('ACTIVATE_EXTRUDER EXTRUDER=extruder')
        self.recovery_tool = None

    def _run(self, script):
        self.gcode.run_script_from_command(script)

    def _set_normal(self, tool):
        offset = ([0., 0., 0.], self.offsets)[tool]
        park = self.right_park_x if tool == 0 else self.left_park_x
        current_tool = 1 if self.toolhead.get_extruder().get_name() \
            == 'extruder1' else 0
        park_script = ''
        if current_tool != tool:
            park_script = (
                'SET_DUAL_CARRIAGE CARRIAGE=%d MODE=PRIMARY\n'
                'G90\nG1 X%.6f F%.3f\n'
                % (current_tool, park, self.travel_speed * 60.))
        self._run(
            park_script +
            'SET_DUAL_CARRIAGE CARRIAGE=%d MODE=PRIMARY\n'
            'ACTIVATE_EXTRUDER EXTRUDER=%s\n'
            'SET_GCODE_OFFSET X=%.6f Y=%.6f Z=%.6f'
            % (tool, 'extruder1' if tool else 'extruder',
               offset[0], offset[1], offset[2]))
        self.mode = MODE_NORMAL
        self.active_tool = tool
        self.first_layer_active = False
        self.first_layer_z = None
        self.base_z_correction = 0.
        self.flow_factors = [1., 1.]

    def _set_synchronized(self, mode, separation):
        self.synchronized_separation = separation
        second_x = (self.right_park_x if mode == MODE_MIRROR
                    else self.left_park_x + separation)
        native_mode = 'MIRROR' if mode == MODE_MIRROR else 'COPY'
        self.first_layer_active = True
        self.first_layer_z = None
        self.flow_factors = [1., 1.]
        self.gaps = [self.first_layer_height, self.first_layer_height]
        self.base_z_correction = 0.
        if self.bed_mesh.get_mesh() is None:
            self.base_z_correction = self._calculate_correction(
                [0., -self.offsets[2]])
        self._run(
            'SET_GCODE_OFFSET X=0 Y=0 Z=%.6f\n'
            'SET_DUAL_CARRIAGE CARRIAGE=0 MODE=PRIMARY\n'
            'G90\nG1 X%.6f F%.3f\n'
            'ACTIVATE_EXTRUDER EXTRUDER=extruder\n'
            'SET_DUAL_CARRIAGE CARRIAGE=1 MODE=PRIMARY\n'
            'G1 X%.6f F%.3f\n'
            'SET_DUAL_CARRIAGE CARRIAGE=0 MODE=PRIMARY\n'
            'SET_DUAL_CARRIAGE CARRIAGE=1 MODE=%s'
            % (self.base_z_correction, self.left_park_x,
               self.travel_speed * 60., second_x,
               self.travel_speed * 60., native_mode))
        self.mode = mode
        self.active_tool = 0

    def _validate_backup_compatibility(self):
        variables = self.printer.lookup_object('save_variables').allVariables
        material0 = variables.get('t0_filament_material')
        material1 = variables.get('t1_filament_material')
        temp0 = float(variables.get('t0_filament_temperature', 0.))
        temp1 = float(variables.get('t1_filament_temperature', 0.))
        if not material0 or material0 != material1:
            raise self.printer.command_error(
                'FLOW backup requires matching filament materials')
        if temp0 <= 0. or abs(temp0 - temp1) > .001:
            raise self.printer.command_error(
                'FLOW backup requires matching nozzle temperatures')
        nozzle0 = self.extruders[0].nozzle_diameter
        nozzle1 = self.extruders[1].nozzle_diameter
        if abs(nozzle0 - nozzle1) > .000001:
            raise self.printer.command_error(
                'FLOW backup requires matching nozzle diameters')

    def _run_adaptive_mesh(self, margin, bed_temp, stabilization_time):
        exclude_objects = self.printer.lookup_object('exclude_object', None)
        objects = [] if exclude_objects is None \
            else exclude_objects.get_status().get('objects', [])
        if not objects:
            raise self.printer.command_error(
                'Adaptive bed mesh requires defined print objects')
        self.previous_mesh = self.bed_mesh.get_mesh()
        self.adaptive_mesh_active = True
        script = 'M140 S%.6f\nM190 S%.6f\n' % (bed_temp, bed_temp)
        if stabilization_time:
            script += 'G4 P%.3f\n' % (stabilization_time * 1000.,)
        if self.mode in SYNC_MODES:
            # Preserve the logical synchronized mode for two-area adaptive
            # point expansion, but isolate H1 physically while H0 probes.
            script += (
                'SET_DUAL_CARRIAGE CARRIAGE=1 MODE=PRIMARY\n'
                'G90\nG1 X%.6f Y%.6f F%.3f\n'
                'SET_DUAL_CARRIAGE CARRIAGE=0 MODE=PRIMARY\nM400\n'
                % (self.right_park_x, self.adaptive_park_y,
                   self.travel_speed * 60.))
        script += 'BED_MESH_CLEAR\nBED_MESH_CALIBRATE ADAPTIVE=1'
        if margin is not None:
            script += ' ADAPTIVE_MARGIN=%.6f' % (margin,)
        try:
            self._run(script)
            self.park_heads(
                self.left_park_x, self.adaptive_park_y, self.travel_speed)
        except self.printer.command_error:
            self._restore_previous_mesh()
            self._finish_adaptive_nozzle_hold(False)
            raise
        self._finish_adaptive_nozzle_hold(True)

    def _restore_previous_mesh(self):
        if not self.adaptive_mesh_active:
            return
        self.bed_mesh.set_mesh(self.previous_mesh)
        self.previous_mesh = None
        self.adaptive_mesh_active = False

    def _handle_print_state_changed(self, state):
        if state in ('complete', 'cancelled', 'error'):
            self._restore_previous_mesh()
            self._finish_adaptive_nozzle_hold(False)

    def _apply_mode(self, mode, tool, height, separation,
                    adaptive_mesh=False, adaptive_margin=None,
                    adaptive_bed_temp=None,
                    adaptive_bed_stabilization_time=None):
        self.backup_enabled = mode == MODE_BACKUP
        if mode in SYNC_MODES:
            self.first_layer_height = height
            self._set_synchronized(mode, separation)
        elif mode == MODE_NORMAL:
            self._set_normal(tool)
        else:
            self._set_normal(tool)
            self.mode = MODE_BACKUP
        if adaptive_mesh:
            self._run_adaptive_mesh(
                adaptive_margin, adaptive_bed_temp,
                adaptive_bed_stabilization_time)

    def _handle_home_rails_end(self, homing_state, rails):
        if self.pending_mode is None or self.pending_activation_scheduled:
            return
        self.pending_activation_scheduled = True
        self.printer.get_reactor().register_callback(
            self._activate_pending_mode)

    def _activate_pending_mode(self, eventtime):
        pending = self.pending_mode
        if pending is None:
            self.pending_activation_scheduled = False
            return
        if not self._is_homed(eventtime):
            self.pending_activation_scheduled = False
            return
        # Claim the activation before running any homing/probing scripts.
        # Those scripts may emit homing events and must not queue this same
        # deferred request a second time while the first activation is busy.
        self.pending_mode = None
        self.pending_activation_scheduled = False
        try:
            with self.gcode.get_mutex():
                self._apply_mode(pending['mode'], pending['tool'],
                                 pending['height'], pending['separation'],
                                 pending['adaptive_mesh'],
                                 pending['adaptive_margin'],
                                 pending['adaptive_bed_temp'],
                                 pending['adaptive_bed_stabilization_time'])
            self.gcode.respond_info(
                'FLOW IDEX deferred mode active: %s, tool: T%d'
                % (self.mode, self.active_tool))
        except self.printer.command_error as e:
            self._restore_previous_mesh()
            self._finish_adaptive_nozzle_hold(False)
            self.gcode.respond_raw(
                '!! FLOW IDEX deferred mode activation failed: %s' % (e,))
            self.printer.send_event('gcode:command_error')
            try:
                self.gcode.run_script('PAUSE')
            except self.printer.command_error:
                pass

    def attempt_failover(self, sensor_name):
        if not self.backup_enabled:
            return False
        if self.failover_latched:
            raise self.printer.command_error(
                'FLOW backup failover is already latched')
        if sensor_name not in self.sensor_names:
            raise self.printer.command_error(
                'Unknown FLOW backup sensor %s' % (sensor_name,))
        failed = self.sensor_names.index(sensor_name)
        replacement = 1 - failed
        self._validate_backup_compatibility()
        sensor = self.printer.lookup_object(
            'pat9125_filament_sensor ' + self.sensor_names[replacement])
        status = sensor.get_status(self.printer.get_reactor().monotonic())
        if not status.get('enabled') or not status.get('filament_detected'):
            raise self.printer.command_error(
                'FLOW backup replacement filament path is not healthy')
        heater_status = self.extruders[replacement].get_status(
            self.printer.get_reactor().monotonic())
        if not heater_status.get('can_extrude'):
            raise self.printer.command_error(
                'FLOW backup replacement extruder is too cold')
        self._set_normal(replacement)
        script = self.cleaning_gcodes[replacement]
        if self.prime_distance:
            script += ('\nSAVE_GCODE_STATE NAME=FLOW_BACKUP_PRIME\n'
                       'M83\nG1 E%.6f F%.3f\nM400\n'
                       'RESTORE_GCODE_STATE NAME=FLOW_BACKUP_PRIME'
                       % (self.prime_distance, self.prime_speed * 60.))
        script += ('\nRESTORE_GCODE_STATE NAME=FMS_RECOVERY_POSITION '
                   'MOVE=1 MOVE_SPEED=%.3f\nM400'
                   % (self.travel_speed,))
        self._run(script)
        self.failover_latched = True
        self.failed_tool = failed
        self.backup_enabled = False
        return True

    def cmd_SET_FLOW_MODE(self, gcmd):
        mode = gcmd.get('MODE').upper()
        if mode not in (MODE_NORMAL, MODE_PARALLEL, MODE_MIRROR, MODE_BACKUP):
            raise gcmd.error('MODE must be NORMAL, PARALLEL, MIRROR, or BACKUP')
        self._require_offsets(gcmd)
        tool = gcmd.get_int('TOOL', self.active_tool, minval=0, maxval=1)
        height = None
        separation = self.parallel_separation
        adaptive_mesh = bool(gcmd.get_int(
            'ADAPTIVE_MESH', 0, minval=0, maxval=1))
        adaptive_margin = gcmd.get_float(
            'ADAPTIVE_MARGIN', None, minval=0.)
        adaptive_bed_temp = gcmd.get_float(
            'ADAPTIVE_BED_TEMP', None, above=0.)
        adaptive_bed_stabilization_time = gcmd.get_float(
            'ADAPTIVE_BED_STABILIZATION_TIME',
            self.adaptive_bed_stabilization_time, minval=0.)
        if adaptive_mesh and adaptive_bed_temp is None:
            raise gcmd.error('Adaptive bed mesh requires a bed temperature')
        if mode in SYNC_MODES:
            height = gcmd.get_float('FIRST_LAYER_HEIGHT', None, above=0.)
            if height is None:
                raise gcmd.error('%s requires FIRST_LAYER_HEIGHT' % (mode,))
            separation = gcmd.get_float(
                'SEPARATION', self.parallel_separation, above=0.)
        if gcmd.get_int('DEFER', 0, minval=0, maxval=1):
            if adaptive_mesh:
                self._begin_adaptive_nozzle_hold()
            self.pending_mode = {
                'mode': mode, 'tool': tool, 'height': height,
                'separation': separation,
                'adaptive_mesh': adaptive_mesh,
                'adaptive_margin': adaptive_margin,
                'adaptive_bed_temp': adaptive_bed_temp,
                'adaptive_bed_stabilization_time':
                    adaptive_bed_stabilization_time,
            }
            gcmd.respond_info(
                'FLOW IDEX mode %s queued until the next complete G28'
                % (mode,))
            return
        self._require_homed(gcmd)
        self.pending_mode = None
        if adaptive_mesh:
            self._begin_adaptive_nozzle_hold()
        self._apply_mode(mode, tool, height, separation,
                         adaptive_mesh, adaptive_margin,
                         adaptive_bed_temp,
                         adaptive_bed_stabilization_time)
        gcmd.respond_info('FLOW IDEX mode: %s, tool: T%d'
                          % (mode, self.active_tool))

    def _handle_layer_changed(self, layer):
        if layer is not None and layer >= 2:
            self.first_layer_active = False
            self.flow_factors = [1., 1.]

    def cmd_SET_FLOW_LAYER(self, gcmd):
        layer = gcmd.get_int('LAYER', minval=0)
        self._handle_layer_changed(layer)

    def cmd_RESET_FLOW_FAILOVER(self, gcmd):
        print_stats = self.printer.lookup_object('print_stats', None)
        if print_stats is not None:
            state = print_stats.get_status(
                self.printer.get_reactor().monotonic()).get('state')
            if state in ('printing', 'paused'):
                raise gcmd.error('Cannot reset FLOW failover during a print')
        self.failover_latched = False
        self.failed_tool = None

    def get_status(self, eventtime):
        return {
            'mode': self.mode,
            'pending_mode': (None if self.pending_mode is None
                             else self.pending_mode['mode']),
            'active_tool': self.active_tool,
            'backup_enabled': self.backup_enabled,
            'failover_latched': self.failover_latched,
            'failed_tool': self.failed_tool,
            'offsets_valid': self.offsets_valid,
            'offsets': {'x': self.offsets[0], 'y': self.offsets[1],
                        'z': self.offsets[2]},
            'first_layer_height': self.first_layer_height,
            'first_layer_active': self.first_layer_active,
            'first_layer_z': self.first_layer_z,
            'mesh_ready': self.bed_mesh is not None
                          and self.bed_mesh.get_mesh() is not None,
            'adaptive_mesh_active': self.adaptive_mesh_active,
            'adaptive_nozzle_hold_active':
                self.adaptive_nozzle_hold_active,
            'adaptive_bed_stabilization_time':
                self.adaptive_bed_stabilization_time,
            'adaptive_park_y': self.adaptive_park_y,
            'flow_factors': list(self.flow_factors),
            'nozzle_gaps': list(self.gaps),
            'base_z_correction': self.base_z_correction,
            'recovery_tool': self.recovery_tool,
        }


def load_config(config):
    return FlowIdexModes(config)
