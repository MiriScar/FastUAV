"""
FAST - Copyright (c) 2016 ONERA ISAE.
"""

"""
Structural Analysis Computation Module - ISAE SUPAERO Msc student Miriam Scarano, summer intern at Concordia University
"""

#  This file is part of FAST : A framework for rapid Overall Aircraft Design
#  Copyright (C) 2020  ONERA & ISAE-SUPAERO
#  FAST is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.

import openmdao.api as om
import numpy as np
import matlab.engine
import math
from fastoad.module_management.service_registry import RegisterOpenMDAOSystem


# Load calculation Analysis component
@RegisterOpenMDAOSystem("fastuav.StructuralAnalysis.multirotor")
class LoadCalculation(om.ExplicitComponent):

    def setup(self):
        # Define inputs (drop height, speed, angle wrt vertical axis, total mass of the multicopter and safety factor)
        self.add_input("data:structures:landing_gear:drop_test:drop_height", val=np.nan, units="m")
        self.add_input("data:structures:landing_gear:drop_test:impact_speed", val=np.nan, units="m/s")
        self.add_input("data:geometry:landing_gear:legs:alpha", val=np.nan, units = "rad")
        self.add_input("data:weight:mtow", units="kg")
        self.add_input("uncertainity:airworthiness:safety_factor", val=np.nan, units=None)

        # Define outputs (computed impact load and its decomposition into axial components)
        self.add_output("data:structures:landing_gear:drop_test:impact_load", units="N")
        self.add_output("data:structures:landing_gear:drop_test:impact_load:compression", units="N")
        self.add_output("data:structures:landing_gear:drop_test:impact_load:transverse_load", units="N")

    def setup_partials(self):
        # Finite difference all partials.
        self.declare_partials(of="*", wrt="*", method="fd")

    def compute(self, inputs, outputs):
        # Extract inputs
        drop_height = inputs["data:structures:landing_gear:drop_test:drop_height"]
        V_impact    = inputs["data:structures:landing_gear:drop_test:impact_speed"]
        alpha       = inputs["data:geometry:landing_gear:legs:alpha"]
        uav_mass    = inputs["data:weight:mtow"]
        SF          = inputs["uncertainity:airworthiness:safety_factor"]

        # Compute the impact load with an energetic approach. Energy conservation law applies:
        # at impact, the uav kinetic energy is entirely converted into potential energy.
        # A safety factor of 1.5 has been assumed.
        Impact_load = (1/2*drop_height)*uav_mass*V_impact**2*(SF)
        # Force decomposition into longitudinal load (compression) and transverse load (upwards).
        Compression = Impact_load*np.cos(alpha)
        Transverse_load  = Impact_load*np.sin(alpha)

        # Assign outputs (computed load and its components)
        outputs["data:structures:landing_gear:drop_test:impact_load"] = Impact_load
        outputs["data:structures:landing_gear:drop_test:impact_load:compression"] = Compression
        outputs["data:structures:landing_gear:drop_test:impact_load:transverse_load"] = Transverse_load



# Structural Analysis component
@RegisterOpenMDAOSystem("fastuav.StructuralAnalysis.multirotor")
class StructuralAnalysis(om.ImplicitComponent):
    
    # Define inputs (impact load just computed, geometry)
    def setup(self):
        # Define inputs (geometry, load longitudinal component)
        self.add_input("data:geometry:landing_gear:cross_section_area", val=np.nan, units="m**2")
        self.add_input("data:structures:landing_gear:drop_test:impact_load:compression", units="N")

        # Define outputs (stress distribution)
        self.add_output("data:structures:landing_gear:drop_test:impact_stress", units="N/m**2")


    def setup_partials(self):
        # Finite difference all partials.
        self.declare_partials(of="*", wrt="*", method="fd")

    # Considering the strut as a slender body pinned at both extremities (one at the airframe, one at the foot),
    # and assuming the worst loading scenario (hard one-point landing), the critical stress would be the one 
    # caused by the longitudinal component (compression) of the impact load, since it may provoke buckling.
    def compute(self, inputs, outputs):
        # Extract inputs (compression, landing gear cross section area)
        Compression = inputs["data:structures:landing_gear:drop_test:impact_load:compression"]
        S = inputs["data:geometry:landing_gear:cross_section_area"]


        # Compute the stress generated by the impact force in the landing gear strut (to be refined).
        Impact_stress = Compression/S

        # Assign outputs
        outputs["data:structures:landing_gear:drop_test:impact_stress"] = Impact_stress



    """
    Module to perform structural analysis. The scope is to compute the stress in the structure
    based on given design variables (e.g., dimensions, material properties, loads..). Since the 
    load depends on the mass and the mass is the objective function to minimize, the optimization
    problem is coupled, meaning that the module needs to be implemented as an implicit component.
    """