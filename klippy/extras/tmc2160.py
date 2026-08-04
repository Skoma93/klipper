import math
from . import tmc, tmc2130


######################################################################
# Register map (datasheet chapter 5)
######################################################################

Registers = {
    # General configuration registers (0x00..0x0F)
    "GCONF":        0x00,
    "GSTAT":        0x01,
    "IOIN":         0x04,
    "OTP_PROG":     0x06,
    "OTP_READ":     0x07,
    "FACTORY_CONF": 0x08,
    "SHORT_CONF":   0x09,
    "DRV_CONF":     0x0A,
    "GLOBALSCALER":  0x0B,
    "OFFSET_READ":  0x0C,

    # Velocity dependent feature control (0x10..0x1F + 0x33)
    "IHOLD_IRUN":   0x10,
    "TPOWERDOWN":   0x11,
    "TSTEP":        0x12,
    "TPWMTHRS":     0x13,
    "TCOOLTHRS":    0x14,
    "THIGH":        0x15,
    "VDCMIN":       0x33,

    # Direct current mode
    "XDIRECT":      0x2D,

    # Microstep table (0x60..0x6B)
    "MSLUT0":       0x60,
    "MSLUT1":       0x61,
    "MSLUT2":       0x62,
    "MSLUT3":       0x63,
    "MSLUT4":       0x64,
    "MSLUT5":       0x65,
    "MSLUT6":       0x66,
    "MSLUT7":       0x67,
    "MSLUTSEL":     0x68,
    "MSLUTSTART":   0x69,
    "MSCNT":        0x6A,
    "MSCURACT":     0x6B,

    # Driver register set (0x6C..0x73)
    "CHOPCONF":     0x6C,
    "COOLCONF":     0x6D,
    "DCCTRL":       0x6E,
    "DRV_STATUS":   0x6F,
    "PWMCONF":      0x70,
    "PWM_SCALE":    0x71,
    "PWM_AUTO":     0x72,
    "LOST_STEPS":   0x73,
}

# Registers typically read by DUMP_TMC (readable in datasheet)
ReadRegisters = [
    "GCONF",
    "GSTAT",
    "IOIN",
    "OTP_READ",
    "FACTORY_CONF",
    "OFFSET_READ",
    "TSTEP",
    "XDIRECT",
    "MSCNT",
    "MSCURACT",
    "CHOPCONF",
    "DRV_STATUS",
    "PWM_SCALE",
    "PWM_AUTO",
    "LOST_STEPS",
]

######################################################################
# Bitfield masks (datasheet chapter 5)
######################################################################

Fields = {}

# 0x00 GCONF
Fields["GCONF"] = {
    "recalibrate":           1 << 0,
    "faststandstill":        1 << 1,
    "en_pwm_mode":           1 << 2,
    "multistep_filt":        1 << 3,
    "shaft":                 1 << 4,
    "diag0_error":           1 << 5,
    "diag0_otpw":            1 << 6,
    "diag0_stall":           1 << 7,
    "diag1_stall":           1 << 8,
    "diag1_index":           1 << 9,
    "diag1_onstate":         1 << 10,
    "diag1_steps_skipped":   1 << 11,
    "diag0_int_pushpull":    1 << 12,
    "diag1_pushpull":        1 << 13,
    "small_hysteresis":      1 << 14,
    "stop_enable":           1 << 15,
    "direct_mode":           1 << 16,
    "test_mode":             1 << 17,
}

# 0x01 GSTAT (R+WC)
Fields["GSTAT"] = {
    "reset":     1 << 0,
    "drv_err":   1 << 1,
    "uv_cp":     1 << 2,
}

# 0x04 IOIN (R)
Fields["IOIN"] = {
    "step":      1 << 0,
    "dir":       1 << 1,
    "dcen_cfg4": 1 << 2,
    "dcin_cfg5": 1 << 3,
    "drv_enn":   1 << 4,
    "dco_cfg6":  1 << 5,
    "version":   0xff << 24,
}

# 0x06 OTP_PROG (W)
Fields["OTP_PROG"] = {
    "otpbit":    0x07,         # bits 2..0
    "otpbyte":   0x03 << 4,    # bits 5..4 (must be 0)
    "otpmagic":  0xff << 8,    # bits 15..8 (0xBD)
}

# 0x07 OTP_READ (R) - bits 7..0 are OTP0
Fields["OTP_READ"] = {
    "otp0":         0xff,
    "otp_tbl":      1 << 7,
    "otp_bbm":      1 << 6,
    "otp_s2_level": 1 << 5,
    "otp_fclktrim": 0x1f,       # bits 4..0
}

# 0x08 FACTORY_CONF (RW)
Fields["FACTORY_CONF"] = {
    "fclktrim": 0x1f,           # bits 4..0
}

# 0x09 SHORT_CONF (W)
Fields["SHORT_CONF"] = {
    "s2vs_level":   0x0f,       # bits 3..0
    "s2g_level":    0x0f << 8,  # bits 11..8
    "shortfilter":  0x03 << 16, # bits 17..16
    "shortdelay":   1 << 18,    # bit 18
}

# 0x0A DRV_CONF (W)
Fields["DRV_CONF"] = {
    "bbmtime":      0x1f,       # bits 4..0
    "bbmclks":      0x0f << 8,  # bits 11..8
    "otselect":     0x03 << 16, # bits 17..16
    "drvstrength":  0x03 << 18, # bits 19..18
    "filt_isense":  0x03 << 20, # bits 21..20
}

# 0x0B GLOBAL_SCALER (W)
Fields["GLOBALSCALER"] = {
    "globalscaler": 0xff,
}

# 0x0C OFFSET_READ (R) (signed bytes)
Fields["OFFSET_READ"] = {
    "offset_b": 0xff,           # bits 7..0 (signed)
    "offset_a": 0xff << 8,      # bits 15..8 (signed)
}

# 0x10 IHOLD_IRUN (W)
Fields["IHOLD_IRUN"] = {
    "ihold":        0x1f,       # bits 4..0
    "irun":         0x1f << 8,  # bits 12..8
    "iholddelay":   0x0f << 16, # bits 19..16
}

# 0x11 TPOWERDOWN (W)
Fields["TPOWERDOWN"] = { "tpowerdown": 0xff }

# 0x12 TSTEP (R)
Fields["TSTEP"] = { "tstep": 0xfffff }

# 0x13 TPWMTHRS (W)
Fields["TPWMTHRS"] = { "tpwmthrs": 0xfffff }

# 0x14 TCOOLTHRS (W)
Fields["TCOOLTHRS"] = { "tcoolthrs": 0xfffff }

# 0x15 THIGH (W)
Fields["THIGH"] = { "thigh": 0xfffff }

# 0x2D XDIRECT (RW) - direct mode currents are signed 9-bit
# NOTE: Use unique field names to avoid FieldHelper collisions.
Fields["XDIRECT"] = {
    "direct_cur_a": 0x1ff,          # bits 8..0 (signed)
    "direct_cur_b": 0x1ff << 16,    # bits 24..16 (signed)
}

# 0x33 VDCMIN (W) - only bits 22..8 used
Fields["VDCMIN"] = { "vdcmin": 0x7fff << 8 }

# 0x60..0x67 MSLUTx (W)
Fields["MSLUT0"] = { "mslut0": 0xffffffff }
Fields["MSLUT1"] = { "mslut1": 0xffffffff }
Fields["MSLUT2"] = { "mslut2": 0xffffffff }
Fields["MSLUT3"] = { "mslut3": 0xffffffff }
Fields["MSLUT4"] = { "mslut4": 0xffffffff }
Fields["MSLUT5"] = { "mslut5": 0xffffffff }
Fields["MSLUT6"] = { "mslut6": 0xffffffff }
Fields["MSLUT7"] = { "mslut7": 0xffffffff }

# 0x68 MSLUTSEL (W)
Fields["MSLUTSEL"] = {
    "x3": 0xFF << 24,
    "x2": 0xFF << 16,
    "x1": 0xFF << 8,
    "w3": 0x03 << 6,
    "w2": 0x03 << 4,
    "w1": 0x03 << 2,
    "w0": 0x03 << 0,
}

# 0x69 MSLUTSTART (W)
Fields["MSLUTSTART"] = {
    "start_sin":    0xFF << 0,   # bits 7..0
    "start_sin90":  0xFF << 16,  # bits 23..16
}

# 0x6A MSCNT (R)
Fields["MSCNT"] = { "mscnt": 0x3ff }

# 0x6B MSCURACT (R) - signed 9-bit currents
Fields["MSCURACT"] = {
    "cur_b": 0x1ff,           # bits 8..0 (signed)
    "cur_a": 0x1ff << 16,     # bits 24..16 (signed)
}

# 0x6C CHOPCONF (RW)
Fields["CHOPCONF"] = {
    "toff":     0x0F << 0,
    "hstrt":    0x07 << 4,
    "hend":     0x0F << 7,
    "fd3":      0x01 << 11,
    "disfdcc":  0x01 << 12,
    "chm":      0x01 << 14,
    "tbl":      0x03 << 15,
    "vhighfs":  0x01 << 18,
    "vhighchm": 0x01 << 19,
    "tpfd":     0x0F << 20,
    "mres":     0x0F << 24,
    "intpol":   0x01 << 28,
    "dedge":    0x01 << 29,
    "diss2g":   0x01 << 30,
    "diss2vs":  0x01 << 31,
}

# 0x6D COOLCONF (W)
Fields["COOLCONF"] = {
    "semin":   0x0f,          # bits 3..0
    "seup":    0x03 << 5,     # bits 6..5
    "semax":   0x0f << 8,     # bits 11..8
    "sedn":    0x03 << 13,    # bits 14..13
    "seimin":  1 << 15,
    "sgt":     0x7f << 16,    # signed -64..+63
    "sfilt":   1 << 24,
}

# 0x6E DCCTRL (W)
Fields["DCCTRL"] = {
    "dc_time": 0x03ff,        # bits 9..0
    "dc_sg":   0xff << 16,    # bits 23..16
}

# 0x6F DRV_STATUS (R)
Fields["DRV_STATUS"] = {
    "sg_result":  0x3ff,          # bits 9..0
    "s2vsa":      1 << 12,
    "s2vsb":      1 << 13,
    "stealth":    1 << 14,
    "fsactive":   1 << 15,
    "cs_actual":  0x1f << 16,     # bits 20..16
    "stallguard": 1 << 24,
    "ot":         1 << 25,
    "otpw":       1 << 26,
    "s2ga":       1 << 27,
    "s2gb":       1 << 28,
    "ola":        1 << 29,
    "olb":        1 << 30,
    "stst":       1 << 31,
}

# 0x70 PWMCONF (W)
Fields["PWMCONF"] = {
    "pwm_ofs":       0xff,         # bits 7..0
    "pwm_grad":      0xff << 8,    # bits 15..8
    "pwm_freq":      0x03 << 16,   # bits 17..16
    "pwm_autoscale": 1 << 18,
    "pwm_autograd":  1 << 19,
    "freewheel":     0x03 << 20,   # bits 21..20
    "pwm_reg":       0x0f << 24,   # bits 27..24
    "pwm_lim":       0x0f << 28,   # bits 31..28
}

# 0x71 PWM_SCALE (R) - pwm_scale_auto is signed 9-bit
Fields["PWM_SCALE"] = {
    "pwm_scale_sum":  0xff,        # bits 7..0
    "pwm_scale_auto": 0x1ff << 16, # bits 24..16 (signed)
}

# 0x72 PWM_AUTO (R)
Fields["PWM_AUTO"] = {
    "pwm_ofs_auto":   0xff,        # bits 7..0
    "pwm_grad_auto":  0xff << 16,  # bits 23..16
}

# 0x73 LOST_STEPS (R)
Fields["LOST_STEPS"] = { "lost_steps": 0xfffff }


######################################################################
# Formatting helpers
######################################################################

FieldFormatters = {
    "shaft":       (lambda v: "1(Reverse)" if v else ""),
    "reset":       (lambda v: "1(Reset)" if v else ""),
    "drv_err":     (lambda v: "1(ErrorShutdown!)" if v else ""),
    "uv_cp":       (lambda v: "1(Undervoltage!)" if v else ""),
    "version":     (lambda v: "%#x" % v),
    "mres":        (lambda v: "%d(%dusteps)" % (v, 0x100 >> v)),
    "otpw":        (lambda v: "1(OvertempWarning!)" if v else ""),
    "ot":          (lambda v: "1(OvertempError!)" if v else ""),
    "s2ga":        (lambda v: "1(ShortToGND_A!)" if v else ""),
    "s2gb":        (lambda v: "1(ShortToGND_B!)" if v else ""),
    "s2vsa":       (lambda v: "1(ShortToSupply_A!)" if v else ""),
    "s2vsb":       (lambda v: "1(ShortToSupply_B!)" if v else ""),
    "ola":         (lambda v: "1(OpenLoad_A!)" if v else ""),
    "olb":         (lambda v: "1(OpenLoad_B!)" if v else ""),
    "stealth":     (lambda v: "1(StealthChop)" if v else ""),
    "fsactive":    (lambda v: "1(FullStepActive)" if v else ""),
    "cs_actual":   (lambda v: ("%d" % v) if v else "0(Reset?)"),
}

SignedFields = [
    # Microstep currents
    "cur_a", "cur_b",
    # Direct mode currents
    "direct_cur_a", "direct_cur_b",
    # COOLCONF.sgt (-64..+63)
    "sgt",
    # PWM_SCALE.pwm_scale_auto (-255..+255)
    "pwm_scale_auto",
    # OFFSET_READ signed bytes
    "offset_a", "offset_b",
]


######################################################################
# TMC stepper current config helper
######################################################################

VREF = 0.325
# The practical maximum is board and power-stage dependent. This limit is a
# configuration sanity check consistent with Klipper's TMC5160 support.
MAX_CURRENT = 10.0

class TMC2160CurrentHelper:
    def __init__(self, config, mcu_tmc):
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        run_current = config.getfloat(
            'run_current', above=0., maxval=MAX_CURRENT)
        hold_current = config.getfloat(
            'hold_current', run_current, above=0., maxval=MAX_CURRENT)
        self.req_hold_current = hold_current
        self.sense_resistor = config.getfloat(
            'sense_resistor', 0.075, above=0.)
        gscaler, irun, ihold = self._calc_current(
            run_current, hold_current)
        self.fields.set_field("globalscaler", gscaler)
        self.fields.set_field("ihold", ihold)
        self.fields.set_field("irun", irun)
        self.fields.set_field("iholddelay", 6)

    def _calc_globalscaler(self, current):
        globalscaler = int(
            current * 256. * math.sqrt(2.) * self.sense_resistor / VREF
            + .5)
        globalscaler = max(32, globalscaler)
        if globalscaler >= 256:
            globalscaler = 0
        return globalscaler

    def _calc_current_bits(self, current, globalscaler):
        if not globalscaler:
            globalscaler = 256
        cs = int(
            current * 256. * 32. * math.sqrt(2.) * self.sense_resistor
            / (globalscaler * VREF) - 1. + .5)
        return max(0, min(31, cs))

    def _calc_current(self, run_current, hold_current):
        gscaler = self._calc_globalscaler(run_current)
        irun = self._calc_current_bits(run_current, gscaler)
        ihold = self._calc_current_bits(
            min(hold_current, run_current), gscaler)
        return gscaler, irun, ihold

    def _calc_current_from_field(self, field_name):
        globalscaler = self.fields.get_field("globalscaler")
        if not globalscaler:
            globalscaler = 256
        bits = self.fields.get_field(field_name)
        return (globalscaler * (bits + 1) * VREF
                / (256. * 32. * math.sqrt(2.) * self.sense_resistor))

    def get_current(self):
        run_current = self._calc_current_from_field("irun")
        hold_current = self._calc_current_from_field("ihold")
        return run_current, hold_current, self.req_hold_current, MAX_CURRENT

    def set_current(self, run_current, hold_current, print_time):
        self.req_hold_current = hold_current
        gscaler, irun, ihold = self._calc_current(
            run_current, hold_current)
        val = self.fields.set_field("globalscaler", gscaler)
        self.mcu_tmc.set_register("GLOBALSCALER", val, print_time)
        self.fields.set_field("ihold", ihold)
        val = self.fields.set_field("irun", irun)
        self.mcu_tmc.set_register("IHOLD_IRUN", val, print_time)

######################################################################
# TMC2160 printer object
######################################################################

class TMC2160:
    def __init__(self, config):
        # Setup field tracking and MCU comms (internal clock ~12MHz)
        self.fields = tmc.FieldHelper(Fields, SignedFields, FieldFormatters)
        self.mcu_tmc = tmc2130.MCU_TMC_SPI(config, Registers, self.fields, 12000000.0)

        # Register commands / status helpers
        current_helper = TMC2160CurrentHelper(config, self.mcu_tmc)
        cmdhelper = tmc.TMCCommandHelper(config, self.mcu_tmc, current_helper)
        cmdhelper.setup_register_dump(ReadRegisters)
        self.get_phase_offset = cmdhelper.get_phase_offset
        self.get_status = cmdhelper.get_status

        # Common helpers (config-driven)
        tmc.TMCWaveTableHelper(config, self.mcu_tmc)
        tmc.TMCStealthchopHelper(config, self.mcu_tmc)
        tmc.TMCVcoolthrsHelper(config, self.mcu_tmc)
        tmc.TMCVhighHelper(config, self.mcu_tmc)
        tmc.TMCVirtualPinHelper(config, self.mcu_tmc)

        set_config_field = self.fields.set_config_field

        # GCONF defaults (safe, configurable)
        set_config_field(config, "recalibrate", 0)
        set_config_field(config, "faststandstill", 0)
        set_config_field(config, "multistep_filt", 1)
        set_config_field(config, "shaft", 0)
        set_config_field(config, "diag0_error", 0)
        set_config_field(config, "diag0_otpw", 0)
        set_config_field(config, "diag0_stall", 0)
        set_config_field(config, "diag1_stall", 0)
        set_config_field(config, "diag1_index", 0)
        set_config_field(config, "diag1_onstate", 0)
        set_config_field(config, "diag1_steps_skipped", 0)
        set_config_field(config, "diag0_int_pushpull", 0)
        set_config_field(config, "diag1_pushpull", 0)
        set_config_field(config, "small_hysteresis", 0)
        set_config_field(config, "stop_enable", 0)
        set_config_field(config, "direct_mode", 0)
        set_config_field(config, "test_mode", 0)

        # CHOPCONF baseline (do not touch mres/intpol/dedge here - handled by Klipper helpers)
        set_config_field(config, "toff", 3)
        set_config_field(config, "hstrt", 5)
        set_config_field(config, "hend", 2)
        set_config_field(config, "fd3", 0)
        set_config_field(config, "disfdcc", 0)
        set_config_field(config, "chm", 0)
        set_config_field(config, "tbl", 2)
        set_config_field(config, "vhighfs", 0)
        set_config_field(config, "vhighchm", 0)
        set_config_field(config, "tpfd", 0)
        set_config_field(config, "diss2g", 0)
        set_config_field(config, "diss2vs", 0)

        # COOLCONF defaults (all off by default; enable via driver_* fields in cfg)
        set_config_field(config, "semin", 0)
        set_config_field(config, "seup", 0)
        set_config_field(config, "semax", 0)
        set_config_field(config, "sedn", 0)
        set_config_field(config, "seimin", 0)
        set_config_field(config, "sgt", 0)
        set_config_field(config, "sfilt", 0)

        # DCCTRL / VDCMIN (DcStep) defaults: disabled
        set_config_field(config, "dc_time", 0)
        set_config_field(config, "dc_sg", 0)
        set_config_field(config, "vdcmin", 0)

        # PWMCONF defaults to datasheet reset defaults (0xC40C001E)
        set_config_field(config, "pwm_ofs", 30)
        set_config_field(config, "pwm_grad", 0)
        set_config_field(config, "pwm_freq", 0)
        set_config_field(config, "pwm_autoscale", 1)
        set_config_field(config, "pwm_autograd", 1)
        set_config_field(config, "freewheel", 0)
        set_config_field(config, "pwm_reg", 4)
        set_config_field(config, "pwm_lim", 12)

        # TPOWERDOWN default per datasheet (min 2 recommended for StealthChop auto-tune)
        set_config_field(config, "tpowerdown", 10)

        # Optional: advanced power-stage/OTP-sensitive registers
        # Only write them if user explicitly sets a driver_* config key.
        def _set_if_defined(field_name, default):
            cfgname = "driver_" + field_name.upper()
            if config.get(cfgname, None) is not None:
                set_config_field(config, field_name, default)

        # FACTORY_CONF (clock trim) - do not touch unless explicitly configured
        _set_if_defined("fclktrim", 0)

        # SHORT_CONF / DRV_CONF - OTP-based defaults; override only if explicitly configured
        _set_if_defined("s2vs_level", 0)
        _set_if_defined("s2g_level", 0)
        _set_if_defined("shortfilter", 0)
        _set_if_defined("shortdelay", 0)

        _set_if_defined("bbmtime", 0)
        _set_if_defined("bbmclks", 0)
        _set_if_defined("otselect", 0)
        _set_if_defined("drvstrength", 0)
        _set_if_defined("filt_isense", 0)



def load_config_prefix(config):
    return TMC2160(config)
