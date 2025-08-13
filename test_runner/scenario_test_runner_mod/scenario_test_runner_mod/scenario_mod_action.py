# -*- coding: utf-8 -*-

# Copyright 2025 D. Ishii. All rights reserved.
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

import copy
import random
from abc import ABC, abstractmethod

import lanelet2
import lanelet2.geometry

from openscenario_utility.conversion import iota

class ScenarioModAction(ABC):
    def __init__(self, name):
        self.name = name

    @abstractmethod
    def get_value(self, ctx):
        pass

    def __call__(self, ctx):
        v = self.get_value(ctx)
        ctx[self.name] = v
        return v

class ListAction(ScenarioModAction):
    def __init__(self, name, lst):
        self.name = name
        self.queue = lst.copy()
        #print(f"LA init {lst}")

    def get_value(self, ctx):
        return self.queue.pop(0) if self.queue else None

class IotaAction(ScenarioModAction):
    def __init__(self, name, start, step, stop):
        self.name = name
        self.start = start
        self.step = step
        self.stop = stop

    def get_value(self, ctx):
        yield from iota(self.start, self.step, self.stop)

class AllLaneIds(ScenarioModAction):
    def __init__(self, name, llts):
        self.name = name
        self.queue = list(map(lambda x: x.id, llts))

    def get_value(self, ctx):
        return self.queue.pop(0) if self.queue else None

class RandomLaneId(ScenarioModAction):
    def __init__(self, name, llts):
        self.name = name
        self.llts = llts

    def get_value(self, ctx):
        return random.choice(this.llts).id if init else None

class ReachableLaneIds(ScenarioModAction):
    def __init__(self, name, ref_nm, param, llt_map, llts):
        self.name = name
        self.ref_nm = ref_nm
        self.llt_map = llt_map
        self.llts = llts

        # Prepare a routing graph.
        rules_map = {"vehicle": lanelet2.traffic_rules.Participants.Vehicle,
                     "bicycle": lanelet2.traffic_rules.Participants.Bicycle,
                     "pedestrian": lanelet2.traffic_rules.Participants.Pedestrian,
                     "train": lanelet2.traffic_rules.Participants.Train}
        traffic_rules = lanelet2.traffic_rules.create(
                lanelet2.traffic_rules.Locations.Germany,
                rules_map["vehicle"])
        # Cost for lane changes
        #routing_cost = lanelet2.routing.RoutingCostDistance(0.)
        routing_cost = lanelet2.routing.RoutingCostDistance(10.)
        self.graph = lanelet2.routing.RoutingGraph(
                self.llt_map, traffic_rules, [routing_cost])

        if isinstance(param, list):
            if len(param) == 1:
                self.inf_cost = 0
                self.max_cost = int(param[0])
            elif len(param) > 1:
                self.inf_cost = int(param[0])
                self.max_cost = int(param[1])
            else:
                raise ValueError("empty param list")
        else:
            self.inf_cost = 0
            self.max_cost = int(param)

    def get_value(self, ctx):
        if not self.ref_nm in ctx:
            raise Exception(f"Ref name {self.ref_nm} not found")

        if not self.name in ctx:
            ref_llt_id = ctx[self.ref_nm]
            ref_llt = next((llt for llt in self.llts if llt.id == ref_llt_id), None)
            if not ref_llt:
                raise Exception(f"Lanelet not found: {ref_llt_id}")
            rs = self.graph.reachableSet(ref_llt, self.max_cost)
            if self.inf_cost > 0:
                rs_lb = self.graph.reachableSet(ref_llt, self.inf_cost)
                rs = set(rs) - set(rs_lb)
            self.queue = list(map(lambda x: x.id, rs))

        return self.queue.pop(0) if self.queue else None

# 

class ScenarioModActionFactory:
    def __init__(self, lanelet_map, mod_entry, verbose = False):
        self.lanelet_map = lanelet_map
        self.lanelets = list(lanelet_map.laneletLayer)
        self.mod_entry = mod_entry
        self.verbose = verbose

    def create(self):
        entry = self.mod_entry
        name = entry['name']
    
        if 'list' in entry:
            return ListAction(name, entry['list'])
    
        elif 'method' in entry:
            if entry['method'] == 'allLaneIds':
                if self.verbose:
                    print(f"Setting LaneIds from {self.lanelets[0:2]}...")
                return AllLaneIds(name, self.lanelets)

            elif entry['method'] == 'randomLaneId':
                return RandomLaneId(name, self.lanelets)

            elif entry['method'] == 'reachableLaneIds':
                if not 'ref' in entry or not 'param' in entry:
                    raise Exception('Parameters are missing')
    
                return ReachableLaneIds(name, entry['ref'], entry['param'], 
                                        self.lanelet_map, self.lanelets)
                 
            else:
                raise Exception(f"Unknown method: {entry['method']}")
    
        else:
            return IotaAction(name, entry["start"], entry["step"], entry["stop"])

# eof
