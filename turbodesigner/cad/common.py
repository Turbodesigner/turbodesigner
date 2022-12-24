from typing import Callable, List, Optional
import cadquery as cq
import cq_warehouse.extensions as cq_warehouse_extensions
from cq_warehouse.fastener import SocketHeadCapScrew, HeatSetNut
import numpy as np
import re


class ExtendedWorkplane(cq.Workplane):
    clearanceHole = cq_warehouse_extensions._clearanceHole
    tapHole = cq_warehouse_extensions._tapHole
    threadedHole = cq_warehouse_extensions._threadedHole
    insertHole = cq_warehouse_extensions._insertHole
    pressFitHole = cq_warehouse_extensions._pressFitHole

    def ring(self, radius: float, thickness: float, depth: float) -> "ExtendedWorkplane":
        return (
            self
            .circle(radius)
            .circle(radius - thickness)
            .extrude(depth)
        )

    def truncated_cone(self, start_radius: float, end_radius: float, height: float):
        plane_world_coords = self.plane.toWorldCoords((0, 0, 0))
        path = cq.Workplane("XZ").moveTo(0, plane_world_coords.z).lineTo(0, height + plane_world_coords.z)

        return (
            self
            .circle(start_radius)
            .transformed(offset=cq.Vector(0, 0, height))
            .circle(end_radius)
            .sweep(path, multisection=True, makeSolid=True)
            .clean()
        )

    def hollow_truncated_cone(self, inner_start_radius: float, inner_end_radius: float, height: float, start_thickness: float, end_thickness: float):
        outer_radius = inner_start_radius + start_thickness
        return (
            self
            .truncated_cone(outer_radius, outer_radius, height)
            .cut(
                ExtendedWorkplane("XY")
                .truncated_cone(inner_start_radius, inner_end_radius, height)
            )
        )

    def mutatePoints(self, callback: Callable[[cq.Location], cq.Location]):
        for (i, loc) in enumerate(self.objects):
            if isinstance(loc, cq.Location):
                mutated_loc = callback(loc)
                self.objects[i] = mutated_loc
        return self
 

class FastenerPredicter:
    @staticmethod
    def get_nominal_size(
        target_diameter: float,
        nominal_size_range: List[str],
    ):
        for nominal_size in nominal_size_range:
            nominal_diameter = float(nominal_size.split("-")[0].replace("M", ""))
            if target_diameter <= nominal_diameter:
                return nominal_size

        raise ValueError(f"nominal size for target diameter {target_diameter} could not be found")

    @staticmethod
    def get_nominal_length(
        target_length: float,
        nominal_length_range: List[float],
    ):
        for nominal_length in nominal_length_range:
            if target_length <= nominal_length:
                return nominal_length
        raise ValueError(f"nominal length for target length {target_length} could not be found")


    @staticmethod
    def predict_heatset(target_diameter: float, max_nut_thickness: float, type: str = "Hilitchi"):
        nominal_size_range = HeatSetNut.sizes(type)
        predicted_size = FastenerPredicter.get_nominal_size(target_diameter, nominal_size_range)

        heatset = HeatSetNut(
            size=predicted_size,
            fastener_type=type,
            simple=True,
        )

        # assert heatset.nut_thickness <= max_nut_thickness, f"heatset nut thickness {heatset.nut_thickness} is greater than max nut thickness {max_nut_thickness}"
        return heatset

    @staticmethod
    def predict_screw(target_diameter: float, target_length: float, type: str = "iso4762"):
        nominal_length_range = SocketHeadCapScrew.nominal_length_range[type]
        predicted_length = FastenerPredicter.get_nominal_length(target_length, nominal_length_range)
        
        nominal_size_range = SocketHeadCapScrew.sizes(type)
        predicted_size = FastenerPredicter.get_nominal_size(target_diameter, nominal_size_range)

        return SocketHeadCapScrew(predicted_size, predicted_length, type)
