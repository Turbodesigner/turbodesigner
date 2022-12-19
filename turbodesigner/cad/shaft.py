from dataclasses import dataclass
from typing import List, Optional
import cadquery as cq
import numpy as np
from turbodesigner.cad.common import ExtendedWorkplane
from turbodesigner.cad.blade import BladeCadModel
from turbodesigner.stage import StageExport, BladeRow
from turbodesigner.turbomachinery import TurbomachineryExport
from cq_warehouse.fastener import SocketHeadCapScrew


@dataclass
class ShaftCadModel:
    transition_to_total_height: float
    "casing transition height to total stage height (dimensionless)"

    shaft_connect_length_to_hub_radius: float
    "shaft connect to hub radius (dimensionless)"

    shaft_connect_height_to_disk_height: float
    "shaft connect to disk height (dimensionless)"

    attachment_tolerance_factor: float = 1.05
    "rotor blade attachment tolerance scale factor (dimensionless)"

    def shaft_stage_assembly(self, stage: StageExport, include_attachment: bool = True):
        base_assembly = cq.Assembly()
        transition_height = stage.stage_height * self.transition_to_total_height

        blade_cad_model = BladeCadModel(self.shaft_connect_height_to_disk_height)
        screw_diameter = blade_cad_model.fastener_diameter(stage.rotor)
        heatset_height = blade_cad_model.fastener_diameter(stage.rotor)
        rotor_blade = blade_cad_model.blade_assembly(stage.rotor, include_attachment)

        shaft_connect_length_length = stage.rotor.hub_radius * self.shaft_connect_length_to_hub_radius
        shaft_connect_length_height = stage.rotor.disk_height * self.shaft_connect_height_to_disk_height
        attachment_indent_radius = stage.rotor.hub_radius-stage.rotor.attachment_height-shaft_connect_length_length

        rotor_blade_screw = SocketHeadCapScrew(f"M{screw_diameter}-0.4", np.floor(shaft_connect_length_length+heatset_height), "iso4762")

        shaft_profile = (
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
        )

        if include_attachment:
            shaft_profile = (
                shaft_profile
                .faces(">Z")
                .workplane()
                .polarArray(stage.rotor.hub_radius*1.0001, 0, 360, stage.rotor.number_of_blades)
                .eachpoint(
                    lambda loc: (
                        cq.Workplane("XY")
                        .polyline(stage.rotor.attachment * self.attachment_tolerance_factor)  # type: ignore
                        .close()
                        .rotate((0, 0, 0), (0, 0, 1), 270)
                    ).val().located(loc), True)  # type: ignore
                .cutBlind(-stage.rotor.disk_height)

                .faces(">Z")
                .workplane(offset=-shaft_connect_length_height/2)
                .polarArray(attachment_indent_radius*0.99, 0, 360, stage.rotor.number_of_blades)
                .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), -90))
                .clearanceHole(rotor_blade_screw, baseAssembly=base_assembly)

                .faces(">Z")
                .workplane()
                .circle(attachment_indent_radius)
                .cutThruAll()
                # .cutBlind(-shaft_connect_length_height)
            )
        base_assembly.add(shaft_profile, name=f"Stage {stage.stage_number} Shaft")

        blade_assembly_locs = (
            ExtendedWorkplane("XY")
            .polarArray(stage.rotor.hub_radius, 0, 360, stage.rotor.number_of_blades)
            .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, stage.stator.disk_height+transition_height+stage.rotor.disk_height/2), cq.Vector(0, 1, 0), 90))
            .vals()
        )

        for (i, blade_assembly_loc) in enumerate(blade_assembly_locs):
            assert isinstance(blade_assembly_loc, cq.Location)
            base_assembly.add(rotor_blade, loc=blade_assembly_loc, name=f"Blade {i+1}")

        return base_assembly

    def shaft_assembly(self, turbomachinery: TurbomachineryExport, include_attachment: bool = True):
        assembly = cq.Assembly()
        stage_height_offset = 0

        for i in range(0, len(turbomachinery.stages)):
            current_stage = turbomachinery.stages[i]
            stage_height_offset -= current_stage.stage_height * (1+self.transition_to_total_height)
            stage_shaft_profile = self.shaft_stage_assembly(current_stage, include_attachment)
            assembly.add(stage_shaft_profile, loc=cq.Location(cq.Vector(0, 0, stage_height_offset)), name=f"Stage {current_stage.stage_number}")

        return assembly
