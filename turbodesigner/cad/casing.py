from dataclasses import dataclass
from turbodesigner.cad.common import ExtendedWorkplane
from turbodesigner.stage import StageExport
import cadquery as cq


@dataclass
class CasingCadModel:
    thickness_to_inlet_radius: float
    "casing thickness to tip radius of first stage (dimensionless)"

    transition_to_total_height: float
    "casing transition height to total stage height (dimensionless)"

    def casing_profile(self, stage: StageExport, first_stage: StageExport):
        casing_thickness = self.thickness_to_inlet_radius * first_stage.rotor.tip_radius
        transition_height = stage.stage_height * self.transition_to_total_height
        casing_height = stage.stage_height + transition_height
        path = (
            cq.Workplane("XZ")
            .moveTo(0, 0)
            .lineTo(0, stage.stage_height*(1+self.transition_to_total_height))
        )
        return (
            cq.Workplane("XY")
            .circle(first_stage.rotor.tip_radius + casing_thickness)
            .extrude(casing_height)
            .cut(
                cq.Workplane("XY")
                .circle(stage.stator.tip_radius)
                .transformed(offset=cq.Vector(0, 0, stage.stator.disk_height))
                .circle(stage.stator.tip_radius)
                .transformed(offset=cq.Vector(0, 0, transition_height))
                .circle(stage.rotor.tip_radius)
                .transformed(offset=cq.Vector(0, 0, stage.rotor.disk_height))
                .circle(stage.rotor.tip_radius)
                .sweep(path, multisection=True, makeSolid=True)
            )
        )