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
from argparse import ArgumentParser
from copy import deepcopy
from itertools import product
from pathlib import Path
from re import sub
from sys import exit, stderr
from pkg_resources import resource_string

import math
import xmlschema
import yaml


class MacroExpander:
    def __init__(self, rules, schema):

        self.rules = rules

        self.schema = schema

        self.specs = []

        if rules is not None:
            for each in rules["ScenarioModifier"]:
                name = each["name"]
                if "list" in each:
                    self.specs.append(list(map(lambda x: (name, x), each["list"])))
                else:
                    self.specs.append(
                        list(
                            map(
                                lambda x: (name, x),
                                iota(each["start"], each["step"], each["stop"]),
                            )
                        )
                    )

    def __call__(self, xosc: str, output: Path, basename: str):
        paths = []

        for index, bindings in enumerate(product(*self.specs)):
            target = deepcopy(xosc)

            for binding in bindings:
                target = sub(str(binding[0]), str(binding[1]), target)

            if self.specs:
                paths.append(output.joinpath(basename + "_" + str(index) + ".xosc"))
            else:
                paths.append(output.joinpath(basename + ".xosc"))

            with paths[-1].open(mode="w") as file:
                file.write(target)

                try:
                    self.schema.validate(target)

                except xmlschema.XMLSchemaValidationError as exception:
                    print("File: " + str(paths[-1]), file=stderr)
                    print("", file=stderr)
                    print("Error: " + str(exception), file=stderr)
                    exit()

        return paths


def convert(input: Path, output: Path, verbose: bool = True):

    if output.exists():
        for each in output.iterdir():
            each.resolve().unlink()
    else:
        output.mkdir(parents=True, exist_ok=True)

    #xsd = resource_string(__name__, "resources/OpenSCENARIO-1.2.xsd").decode("utf-8")
    xsd = resource_string("openscenario_utility", "resources/OpenSCENARIO-1.2.xsd").decode("utf-8")
    #xsd_path = Path(get_package_share_directory("openscenario_utility")) / "resources/OpenSCENARIO-1.2.xsd"
    #xsd = resource_string(str(xsd_path)).decode("utf-8")

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
        paths = macroexpand(
            xmlschema.XMLResource(xosc)
            .tostring()
            .replace("True", "true")
            .replace("False", "false"),
            output,
            input.stem,
        )

        if verbose:
            for each in paths:
                print(each)

        return paths

# eof
