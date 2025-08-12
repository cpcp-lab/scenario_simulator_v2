# -*- coding: utf-8 -*-

# Copyright 2020 TIER IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Modified by D. Ishii, 2025.

# Cloned from openscenario_utility/conversion.py.

import math
import xmlschema
import yaml
import random
import subprocess
from typing import List

from argparse import ArgumentParser
from copy import deepcopy
from itertools import product
from functools import reduce
from pathlib import Path
from re import sub
from sys import exit, stderr
from pkg_resources import resource_string

import lanelet2
import lanelet2.geometry
from autoware_lanelet2_extension_python.projection import MGRSProjector

from openscenario_utility.conversion import iota, load_yaml, from_yaml
#from openscenario_utility.conversion import convert, MacroExpander
from scenario_test_runner.scenario import Scenario

from scenario_modification import ScenarioModActionFactory

def get_from_dict(path, dictio):
    if not isinstance(path, list) or not path or not isinstance(dictio, dict):
        return []

    #print(f"Look for {path[0]}")
    if path[0] in dictio:
        v = dictio[path[0]]
        rest = path[1:]
        if not rest:
            if v[0] == '$':
                r = subprocess.check_output(f"echo {v}", shell=True, text=True)
                v = r.strip()
            return v
        else:
            return get_from_dict(rest, v)
    else:
        return []

def test_projection(lat=0.0, lon=0.0):
    return MGRSProjector(lanelet2.io.Origin(lat, lon))


def test_io(map_path, projection):
    return lanelet2.io.load(map_path, projection)

#

class MacroExpander:
    def __init__(self, rules, schema, lanelet_map, verbose = False):

        self.rules = rules

        self.schema = schema

        self.verbose = verbose

        self.specs = []

        #action_factory = ScenarioModActionFactory(lanelet_map, verbose)

        if rules is not None:
            # Analyze the ScenarioModifier section.
            for each in rules['ScenarioModifier']:
                self.specs.append(
                    #action_factory.create(each)
                    ScenarioModActionFactory(lanelet_map, each, verbose)
                    )

    def substitute_and_save(self, ctx, target, path):
        for nm in ctx:
            if self.verbose:
                print(f"Substitute {nm} with {ctx[nm]}")
            target = sub(str(nm), str(ctx[nm]), target)

        if self.verbose:
            print('====')
            print(target, flush=True)
            print('====')

        with path.open(mode="w") as file:
            file.write(target)

        try:
            self.schema.validate(target)

        except xmlschema.XMLSchemaValidationError as exception:
            print("File: " + str(path), file=stderr)
            print("", file=stderr)
            print("Error: " + str(exception), file=stderr)
            exit()

        return path

    def product_spec(self, specs, ctx, cont):
        if not specs:
            yield cont(ctx)
        else:
            spec = specs[0]
            rest = specs[1:]
            action = spec.create()
            while True:
                ctx_mod = deepcopy(ctx)
                if action(ctx_mod):
                    yield from self.product_spec(rest, ctx_mod, cont)
                else:
                    return

    def __call__(self, xosc: str, output: Path, basename: str, verbose: bool = True):
        target = deepcopy(xosc)
        path = output.joinpath(basename + ".xosc")

        yield from self.product_spec(self.specs, {}, 
                                     lambda ctx: self.substitute_and_save(ctx, target, path))


def convert_mod(input: Path, output: Path, verbose: bool = True):

    if output.exists():
        for each in output.iterdir():
            each.resolve().unlink()
    else:
        output.mkdir(parents=True, exist_ok=True)

    #xsd = resource_string(__name__, "resources/OpenSCENARIO-1.2.xsd").decode("utf-8")
    xsd = resource_string("openscenario_utility", "resources/OpenSCENARIO-1.2.xsd").decode("utf-8")
    schema = xmlschema.XMLSchema(xsd)

    yaml = load_yaml(input)

    map_filepath = get_from_dict(['OpenSCENARIO','RoadNetwork','LogicFile','filepath'], yaml)
    if not map_filepath:
        print("Error: map_filepath not found")
        exit()
    map_path = Path(map_filepath)
    map_osm_f = map_path / 'lanelet2_map.osm'
    if not map_osm_f.exists():
        print(f"Error: {map_osm_f} not found")
        exit()

    lanelet_map = test_io(str(map_osm_f), test_projection())

    macroexpand = MacroExpander(yaml.pop("ScenarioModifiers", None), schema, lanelet_map, verbose)

    xosc, errors = schema.encode(
        from_yaml("OpenSCENARIO", yaml),
        indent=2,
        preserve_root=True,
        unordered=True,  # Reorder elements
        validation="lax",  # The "strict" mode is too strict than we would like.
    )

    if not schema.is_valid(xosc) and len(errors) != 0:
        print(
            "Error: " + str(errors[0]), file=stderr
        )  # Other than the first is not important.
        exit()

    else:
        yield from macroexpand(
            xmlschema.XMLResource(xosc)
            .tostring()
            .replace("True", "true")
            .replace("False", "false"),
            output,
            input.stem,
        )


def convert_scenarios_to_xosc(scenarios: List[Scenario], output_directory: Path):

    for each in scenarios:
        if each.path.suffix == ".xosc":
            yield each

        else:  # == '.yaml' or == '.yml'
            for path in convert_mod(each.path, output_directory / each.path.stem, True):
                yield Scenario(path, each.frame_rate)


# eof
