from dataclasses import dataclass
import cadquery as cq
from turbodesigner.cad.common import ExtendedWorkplane
from turbodesigner.cad.blade import BladeCadModel
from turbodesigner.stage import StageExport
from turbodesigner.turbomachinery import TurbomachineryExport

@dataclass
class ShaftCadModel:
    transition_to_total_height: float
    "casing transition height to total stage height (dimensionless)"

    def stage_shaft_profile(self, stage: StageExport):
        transition_height = stage.stage_height * self.transition_to_total_height
        rotor_blade = BladeCadModel.blade_profile(stage.rotor)
        return (
            ExtendedWorkplane("XY")
            .circle(stage.stator.hub_radius)
            .extrude(stage.stator.disk_height)

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

            .faces("<Z")
            .workplane()
            .text("S", 5, -5)
        )

    def shaft_assembly(self, turbomachinery: TurbomachineryExport):
        assembly = cq.Assembly()
        stage_height_offset = 0

        for i in range(0, len(turbomachinery.stages)):
            current_stage = turbomachinery.stages[i]
            stage_height_offset -= current_stage.stage_height * (1+self.transition_to_total_height)
            stage_shaft_profile = self.stage_shaft_profile(current_stage)
            assembly.add(stage_shaft_profile, loc=cq.Location(cq.Vector(0, 0, stage_height_offset)), name=f"Stage {current_stage.stage_number}")

        return assembly