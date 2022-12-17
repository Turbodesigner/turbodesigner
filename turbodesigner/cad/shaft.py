from dataclasses import dataclass
import cadquery as cq
import numpy as np
from turbodesigner.cad.common import ExtendedWorkplane
from turbodesigner.cad.blade import BladeCadModel
from turbodesigner.stage import StageExport, BladeRow
from turbodesigner.turbomachinery import TurbomachineryExport

@dataclass
class ShaftCadModel:
    transition_to_total_height: float
    "casing transition height to total stage height (dimensionless)"

    screw_indent_length_to_hub_radius: float
    "rotor screw indent to hub radius (dimensionless)"


    def shaft_profile(self, stage: StageExport):
        transition_height = stage.stage_height * self.transition_to_total_height
        rotor_blade = BladeCadModel.blade_profile(stage.rotor)
        blade_height_offset = stage.stator.disk_height+transition_height+stage.rotor.disk_height/2
        screw_indent_length = stage.rotor.hub_radius * self.screw_indent_length_to_hub_radius

        return (
            ExtendedWorkplane("XY")
            .circle(stage.stator.hub_radius)
            .extrude(stage.stator.disk_height)
            .faces("<Z")
            .workplane()
            .text("S", 5, -5)

            .faces(">Z")
            .workplane()
            .truncated_cone(stage.stator.hub_radius, stage.rotor.hub_radius, transition_height)

            .faces(">Z")
            .workplane()
            .circle(stage.rotor.hub_radius)
            .extrude(stage.rotor.disk_height)
            .faces(">Z")
            .workplane()
            .text("R", 5, -5)

            .faces(">Z")
            .workplane()
            .polarArray(stage.rotor.hub_radius*1.0001, 0, 360, stage.rotor.number_of_blades)
            .eachpoint(
                lambda loc: (
                    cq.Workplane("XY")
                    .polyline(stage.rotor.attachment)  # type: ignore 
                    .close()
                    .rotate((0,0,0), (0,0,1), 270)
                ).val().located(loc), True)  # type: ignore
            .cutBlind(-stage.rotor.disk_height)

            .faces(">Z")
            .workplane()
            .circle(stage.rotor.hub_radius-stage.rotor.attachment_height-screw_indent_length)
            .cutBlind(-5)

            .add(
                cq.Workplane("XY")
                .transformed(offset=(0, 0, blade_height_offset))
                .polarArray(stage.rotor.hub_radius, 0, 360, stage.rotor.number_of_blades)
                .eachpoint(
                    lambda loc: (
                        rotor_blade
                        .rotate((0,0,0), (0,1,0), 90)
                    ).val().located(loc), True)  # type: ignore
            )
        )

    def shaft_assembly(self, turbomachinery: TurbomachineryExport):
        assembly = cq.Assembly()
        stage_height_offset = 0

        for i in range(0, len(turbomachinery.stages)):
            current_stage = turbomachinery.stages[i]
            stage_height_offset -= current_stage.stage_height * (1+self.transition_to_total_height)
            stage_shaft_profile = self.shaft_profile(current_stage)
            assembly.add(stage_shaft_profile, loc=cq.Location(cq.Vector(0, 0, stage_height_offset)), name=f"Stage {current_stage.stage_number}")

        return assembly