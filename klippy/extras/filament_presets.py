# Filament material temperature presets
#
# Copyright (C) 2026  Oliver
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import ast


class FilamentPresets:
    def __init__(self, config):
        raw_presets = config.get('presets')
        try:
            entries = ast.literal_eval(raw_presets)
        except (SyntaxError, ValueError):
            raise config.error("Option 'presets' in section '%s' must be a "
                               "list of [material, nozzle, bed] entries"
                               % (config.get_name(),))
        if not isinstance(entries, (list, tuple)) or not entries:
            raise config.error("Option 'presets' in section '%s' must contain "
                               "at least one entry" % (config.get_name(),))
        presets = []
        materials = set()
        for entry in entries:
            if not isinstance(entry, (list, tuple)) or len(entry) != 3:
                raise config.error("Each filament preset must be "
                                   "[material, nozzle, bed]")
            material, nozzle, bed = entry
            if not isinstance(material, str) or not material.strip():
                raise config.error("Filament preset material must be a "
                                   "non-empty string")
            material = material.strip().upper()
            if material in materials:
                raise config.error("Duplicate filament preset material '%s'"
                                   % (material,))
            try:
                nozzle = float(nozzle)
                bed = float(bed)
            except (TypeError, ValueError):
                raise config.error("Filament preset temperatures must be "
                                   "numbers")
            if nozzle < 0. or nozzle > 300.:
                raise config.error("Filament preset nozzle temperature must "
                                   "be between 0 and 300 C")
            if bed < 0. or bed > 130.:
                raise config.error("Filament preset bed temperature must be "
                                   "between 0 and 130 C")
            materials.add(material)
            presets.append([material, nozzle, bed])
        self.presets = presets

    def get_status(self, eventtime):
        return {'presets': self.presets}


def load_config(config):
    return FilamentPresets(config)
