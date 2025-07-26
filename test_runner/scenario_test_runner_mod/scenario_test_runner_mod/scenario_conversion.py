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
from typing import List


class MacroExpander:
    def __init__(self, rules, schema):

        self.rules = rules

        self.schema = schema

        self.specs = []

        if rules is not None:
            # Analyze the ScenarioModifier section.
            for each in rules["ScenarioModifier"]:
                name = each["name"]
                if "list" in each:
                    #self.specs.append(list(map(lambda x: (name, x), each["list"])))
                    self.specs.append((name, lambda: each["list"]))
                else:
                    self.specs.append(
                        #list(
                        #    map(
                        #        lambda x: (name, x),
                        #        iota(each["start"], each["step"], each["stop"]),
                        #    )
                        #)
                        (name, lambda: iota(each["start"], each["step"], each["stop"]))
                    )

        self.specs.append(('', lambda: [None]))

    def substitute_and_save(self, t_p_pair, name, x):
        target, path = t_p_pair

        if x is not None:
            # Substitute.
            target = sub(name, str(x), target)

        else:
            # Invocation w/ a dummy spec element.

            #print('====')
            #print(target)
            #print('====')

            with path.open(mode="w") as file:
                file.write(target)

            try:
                self.schema.validate(target)

            except xmlschema.XMLSchemaValidationError as exception:
                print("File: " + str(path), file=stderr)
                print("", file=stderr)
                print("Error: " + str(exception), file=stderr)
                exit()

        return (target,path)

    def __call__(self, xosc: str, output: Path, basename: str):
        #paths = []

        #for index, bindings in enumerate(product(*self.specs)):
        #    target = deepcopy(xosc)

        #    for binding in bindings:
        #        target = sub(str(binding[0]), str(binding[1]), target)

        #    if self.specs:
        #        paths.append(output.joinpath(basename + "_" + str(index) + ".xosc"))
        #    else:
        #        paths.append(output.joinpath(basename + ".xosc"))

        #    with paths[-1].open(mode="w") as file:
        #        file.write(target)

        #        try:
        #            self.schema.validate(target)

        #        except xmlschema.XMLSchemaValidationError as exception:
        #            print("File: " + str(paths[-1]), file=stderr)
        #            print("", file=stderr)
        #            print("Error: " + str(exception), file=stderr)
        #            exit()

        #return paths

        target = deepcopy(xosc)

        path = output.joinpath(basename + ".xosc")

        return reduce(
            lambda acc, it: 
            (self.substitute_and_save(pr, str(it[0]), x) for pr in acc for x in it[1]()),
            self.specs,
            [(target,path)]
        )


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

    macroexpand = MacroExpander(yaml.pop("ScenarioModifiers", None), schema)

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
        t_p_pairs = macroexpand(
            xmlschema.XMLResource(xosc)
            .tostring()
            .replace("True", "true")
            .replace("False", "false"),
            output,
            input.stem,
        )

        for each in t_p_pairs:
            if verbose:
                print(f"Expanded scenario: {each[1]}")

            yield each[1]


def convert_scenarios_to_xosc(scenarios: List[Scenario], output_directory: Path):

    for each in scenarios:
        if each.path.suffix == ".xosc":
            yield each

        else:  # == '.yaml' or == '.yml'
            for path in convert_mod(each.path, output_directory / each.path.stem, True):
                yield Scenario(path, each.frame_rate)


# eof
