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

from autoware_lanelet2_extension_python.projection import MGRSProjector
import lanelet2
import lanelet2.geometry

from openscenario_utility.conversion import iota, load_yaml, from_yaml
#from openscenario_utility.conversion import convert, MacroExpander
from scenario_test_runner.scenario import Scenario
from argparse import ArgumentParser
from copy import deepcopy
from itertools import product
from functools import reduce
from pathlib import Path
from re import sub
from sys import exit, stderr
from pkg_resources import resource_string

import math
import xmlschema
import yaml
import random
import subprocess
from typing import List

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


class MacroExpander:
    def __init__(self, rules, schema, lanelets, verbose = False):

        self.rules = rules

        self.schema = schema

        self.lanelets = lanelets

        self.verbose = verbose

        self.specs = []

        if rules is not None:
            # Analyze the ScenarioModifier section.
            for each in rules["ScenarioModifier"]:
                name = each["name"]
                if "list" in each:
                    queue = each["list"]
                    self.specs.append((name, lambda: queue.pop(0) if queue else None))

                elif "method" in each:
                    if each["method"] == "randomLaneIds":
                        print(f"Setting LaneIds from {self.lanelets[0:2]}...")
                        print(f"Such as {random.choice(self.lanelets).id}...")
                        self.specs.append(
                            #(name, lambda: [random.choice(self.lanelets).id])
                            (name, lambda: random.choice(self.lanelets).id)
                        )
                    else:
                        self.specs.append(('', lambda: [None]))

                else:
                    self.specs.append(
                        (name, lambda: iota(each["start"], each["step"], each["stop"]))
                    )

        #self.specs.append(('', lambda: [None]))

    #def substitute_and_save(self, t_p_pair, name, x):
    #    target, path = t_p_pair

    #    if x is not None:
    #        # Substitute.
    #        target = sub(name, str(x), target)

    #    else:
    #        # Invocation w/ a dummy spec element.

    #        print('====')
    #        print(target, flush=True)
    #        print('====')

    #        with path.open(mode="w") as file:
    #            file.write(target)

    #        try:
    #            self.schema.validate(target)

    #        except xmlschema.XMLSchemaValidationError as exception:
    #            print("File: " + str(path), file=stderr)
    #            print("", file=stderr)
    #            print("Error: " + str(exception), file=stderr)
    #            exit()

    #    return (target,path)

    def substitute(self, name, x, target):
        if x and target:
            return sub(str(name), str(x), target)
        else:
            return

    def __call__(self, xosc: str, output: Path, basename: str, verbose: bool = True):
        target = deepcopy(xosc)


        ## This will invoke `substitute_and_save()` in a lazy manner.
        ## Accumulator is always `(target,path)`.
        #return reduce(
        #    lambda acc, it: 
        #    #(self.substitute_and_save(pr, str(it[0]), x) for pr in acc for x in it[1]()),
        #    #[self.substitute_and_save(acc[0], str(it[0]), x) for x in it[1]()],
        #    [[self.substitute_and_save(acc[0], str(it[0]), it[1]())]],
        #    self.specs,
        #    [(target,path)]
        #)

        if self.verbose:
            print(f"Specs: {self.specs}", flush=True) 
        s = self.specs[0]
        #t_p_pair = self.substitute((target,path), str(s[0]), s[1]())
        #target = sub(str(s[0]), str(s[1]()), target)
        target = reduce(
            lambda tgt, it: 
            self.substitute(it[0], it[1](), tgt),
            self.specs,
            target
        )

        if target:
            if self.verbose:
                print('====')
                print(target, flush=True)
                print('====')

            #self.substitute_and_save(t_p_pair, '', None)

            path = output.joinpath(basename + ".xosc")
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
    lanelets = list(lanelet_map.laneletLayer)

    macroexpand = MacroExpander(yaml.pop("ScenarioModifiers", None), schema, lanelets, verbose)

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
        while True:
            #t_p_pairs = macroexpand(
            path = macroexpand(
                xmlschema.XMLResource(xosc)
                .tostring()
                .replace("True", "true")
                .replace("False", "false"),
                output,
                input.stem,
            )

            if path:
                yield path
            else:
                break

            #for each in t_p_pairs:
            #    if verbose:
            #        print(f"Expanded scenario: {each[1]}")

            #    yield each[1]

            #for each in paths:
            #    if verbose:
            #        print(f"Expanded scenario: {each}", flush=True)

            #    yield each


def convert_scenarios_to_xosc(scenarios: List[Scenario], output_directory: Path):

    for each in scenarios:
        if each.path.suffix == ".xosc":
            yield each

        else:  # == '.yaml' or == '.yml'
            for path in convert_mod(each.path, output_directory / each.path.stem, True):
                yield Scenario(path, each.frame_rate)


# eof
