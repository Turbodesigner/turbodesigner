from dataclasses import dataclass
from turbodesigner.cad.blade import BladeCadModel
from turbodesigner.cad.common import ExtendedWorkplane
from turbodesigner.stage import StageExport
import cadquery as cq

from turbodesigner.turbomachinery import TurbomachineryExport


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
        stator_blade = BladeCadModel.blade_assembly(stage.stator)
        blade_height_offset = stage.stator.disk_height/2

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
            .add(
                cq.Workplane("XY")
                .polarArray(stage.stator.hub_radius, 0, 360, stage.stator.number_of_blades)
                .eachpoint(
                    lambda loc: (
                        stator_blade
                        .translate((-blade_height_offset, 0, 0))
                        .rotate((0, 0, 0), (0, 1, 0), 90)
                    ).val().located(loc), True)  # type: ignore
            )

        )

    def casing_assembly(self, turbomachinery: TurbomachineryExport):
        assembly = cq.Assembly()
        stage_height_offset = 0

        first_stage = turbomachinery.stages[0]
        for i in range(0, len(turbomachinery.stages)):
            current_stage = turbomachinery.stages[i]
            stage_height_offset -= current_stage.stage_height * (1+self.transition_to_total_height)
            stage_shaft_profile = self.casing_profile(current_stage, first_stage)
            assembly.add(stage_shaft_profile, loc=cq.Location(cq.Vector(0, 0, stage_height_offset)), name=f"Stage {current_stage.stage_number}")

        return assembly
