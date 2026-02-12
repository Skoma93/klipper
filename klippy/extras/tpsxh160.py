import logging

class TPSXH160:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]

        # Config
        self.initial_state = config.getboolean('initial_state', False)
        self.output_name = config.get('output_name', self.name)

        self.retry_on_fault = config.getboolean('retry_on_fault', False)
        self.retry_timeout = config.getfloat('retry_timeout', 5.0, minval=0.1)
        self.retry_interval = config.getfloat('retry_interval', 1.0, minval=0.1)
        self.retry_toggle_delay = config.getfloat('retry_toggle_delay', 0.100, minval=0.0)

        self.shutdown_on_fault = config.getboolean('shutdown_on_fault', True)

        # Pins
        ppins = self.printer.lookup_object('pins')
        self.enable_pin = ppins.setup_pin('digital_out', config.get('enable_pin'))
        self.enable_pin.setup_max_duration(0.)
        self.mcu = self.enable_pin.get_mcu()

        # Program MCU start/shutdown values
        self.enable_pin.setup_start_value(1.0 if self.initial_state else 0.0, 0.0)

        # Fault input via buttons (recommended for digital edge callbacks)
        self.buttons = self.printer.load_object(config, 'buttons')
        self.fault_pin = config.get('fault_pin')

        # If you set fault_pin as "!^PAx" (invert + pullup), then state==1 means fault.
        self.fault_active_high = config.getboolean('fault_active_high', True)

        self.buttons.register_buttons([self.fault_pin], self._fault_event)

        # State
        self.is_enabled = bool(self.initial_state)
        self.fault_detected = False
        self.fault_start_eventtime = None
        self.retry_timer = None
        self.retry_count = 0

        # GCode
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_mux_command(
            "SET_PMIC", "PMIC", self.name,
            self.cmd_SET_PMIC,
            desc=self.cmd_SET_PMIC_help
        )

        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)

        logging.info(
            "TPSXH160 '%s' init: initial=%s retry=%s timeout=%.1fs shutdown=%s",
            self.name, self.initial_state, self.retry_on_fault,
            self.retry_timeout, self.shutdown_on_fault
        )

    def _handle_ready(self):
        # Start value is already configured; just sync bookkeeping.
        self.is_enabled = bool(self.initial_state)
        logging.info("TPSXH160 '%s': ready (enabled=%s)", self.name, self.is_enabled)

    def _handle_shutdown(self):
        try:
            self._set_enable(False)
        except Exception:
            pass
        if self.retry_timer is not None:
            self.reactor.unregister_timer(self.retry_timer)
            self.retry_timer = None

    def _eventtime_to_printtime(self, eventtime):
        min_sched = self.mcu.min_schedule_time()
        systime = eventtime if eventtime is not None else self.reactor.monotonic()
        return self.mcu.estimated_print_time(systime + min_sched)

    def _set_enable(self, state, eventtime=None):
        pt = self._eventtime_to_printtime(eventtime)
        self.enable_pin.set_digital(pt, 1 if state else 0)
        self.is_enabled = bool(state)

    def _schedule_toggle_on(self, eventtime):
        # Force an off->on toggle to retrigger the PMIC
        min_sched = self.mcu.min_schedule_time()
        base = eventtime if eventtime is not None else self.reactor.monotonic()
        pt_off = self.mcu.estimated_print_time(base + min_sched)
        pt_on = self.mcu.estimated_print_time(base + min_sched + self.retry_toggle_delay)
        self.enable_pin.set_digital(pt_off, 0)
        self.enable_pin.set_digital(pt_on, 1)
        self.is_enabled = True

    def _fault_event(self, eventtime, state):
        fault_active = bool(state) if self.fault_active_high else (not bool(state))

        if fault_active and not self.fault_detected:
            self.fault_detected = True
            self.fault_start_eventtime = eventtime
            self._handle_fault(eventtime)
        elif not fault_active and self.fault_detected:
            self.fault_detected = False
            self.fault_start_eventtime = None
            self.retry_count = 0
            if self.retry_timer is not None:
                self.reactor.unregister_timer(self.retry_timer)
                self.retry_timer = None
            logging.info("TPSXH160 '%s': fault cleared", self.name)

    def _handle_fault(self, eventtime):
        self._set_enable(False, eventtime=eventtime)

        msg = "TPSXH160 '%s' (%s): fault detected (possible short circuit)" % (
            self.name, self.output_name
        )
        logging.error("%s retry=%s shutdown=%s",
                      msg, self.retry_on_fault, self.shutdown_on_fault)

        if self.shutdown_on_fault:
            self.printer.invoke_shutdown(msg)
            return

        if self.retry_on_fault:
            self.retry_count = 0
            if self.retry_timer is None:
                self.retry_timer = self.reactor.register_timer(
                    self._retry_timer_event, self.reactor.NOW
                )
        else:
            self.gcode.respond_info(
                "%s - output disabled. Use SET_PMIC PMIC=%s STATE=1 to re-enable."
                % (msg, self.name)
            )

    def _retry_timer_event(self, eventtime):
        if not self.fault_detected or self.fault_start_eventtime is None:
            return self.reactor.NEVER

        elapsed = eventtime - self.fault_start_eventtime
        if elapsed > self.retry_timeout:
            msg = "TPSXH160 '%s' (%s): retry timeout (%.1fs) exceeded - shutting down" % (
                self.name, self.output_name, self.retry_timeout
            )
            logging.error(msg)
            self.printer.invoke_shutdown(msg)
            return self.reactor.NEVER

        self.retry_count += 1
        logging.info("TPSXH160 '%s': retry attempt %d (elapsed %.2fs/%.2fs)",
                     self.name, self.retry_count, elapsed, self.retry_timeout)

        self._schedule_toggle_on(eventtime)
        return eventtime + self.retry_interval

    def enable(self, state):
        if state and self.fault_detected:
            raise self.printer.command_error(
                "Cannot enable TPSXH160 '%s' - fault condition active" % self.name
            )
        self._set_enable(bool(state))
        logging.info("TPSXH160 '%s': %s", self.name,
                     "enabled" if state else "disabled")

    def get_status(self, eventtime):
        return {
            'enabled': self.is_enabled,
            'fault': self.fault_detected,
            'retry_count': self.retry_count
        }

    cmd_SET_PMIC_help = "Enable/disable and query a TPSXH160 PMIC output"
    def cmd_SET_PMIC(self, gcmd):
        state = gcmd.get_int('STATE', None)
        if state is None:
            gcmd.respond_info(
                "TPSXH160 '%s': %s, fault=%s, retries=%d" %
                (self.name,
                 "enabled" if self.is_enabled else "disabled",
                 "1" if self.fault_detected else "0",
                 self.retry_count)
            )
            return
        self.enable(bool(state))
        gcmd.respond_info("TPSXH160 '%s': %s" % (
            self.name, "enabled" if state else "disabled"
        ))

def load_config_prefix(config):
    return TPSXH160(config)