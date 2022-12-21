from dataclasses import dataclass
from typing import List, Optional
import cadquery as cq
import numpy as np
from turbodesigner.cad.common import ExtendedWorkplane
from turbodesigner.cad.blade import BladeCadModel
from turbodesigner.stage import StageExport
from turbodesigner.turbomachinery import TurbomachineryExport
from cq_warehouse.fastener import SocketHeadCapScrew, HeatSetNut


@dataclass
class ShaftCadModel:
    transition_to_total_height: float
    "casing transition height to total stage height (dimensionless)"

    shaft_connect_length_to_hub_radius: float
    "shaft connect to hub radius (dimensionless)"

    shaft_connect_height_to_disk_height: float
    "shaft connect to disk height (dimensionless)"

    shaft_connect_padding_to_attachment_height: float = 1.2
    "shaft connect padding to attachment height (dimensionless)"

    shaft_connect_screw_quantity: int = 4
    "shaft connect screw quantity (dimensionless)"

    attachment_insert_scale_factor: float = 1.05
    "rotor blade attachment tolerance scale factor (dimensionless)"



    def shaft_stage_assembly(self, stage: StageExport, next_stage: StageExport, include_attachment: bool = True):
        base_assembly = cq.Assembly()
        transition_height = stage.stage_height * self.transition_to_total_height

        blade_cad_model = BladeCadModel(self.shaft_connect_height_to_disk_height)
        blade_assembly = blade_cad_model.blade_assembly(stage.rotor, include_attachment)
        lock_screw_diameter = blade_cad_model.fastener_diameter(stage.rotor)
        blade_heatset_height = blade_cad_model.fastener_diameter(stage.rotor)

        shaft_connect_length = stage.rotor.hub_radius * self.shaft_connect_length_to_hub_radius
        shaft_connect_height = stage.rotor.disk_height * self.shaft_connect_height_to_disk_height
        shaft_connect_outer_radius =  stage.rotor.hub_radius-stage.rotor.attachment_height * self.shaft_connect_padding_to_attachment_height
        shaft_connect_inner_radius = shaft_connect_outer_radius-shaft_connect_length

        next_shaft_connect_height = next_stage.rotor.disk_height * self.shaft_connect_height_to_disk_height
        next_shaft_connect_outer_radius = next_stage.rotor.hub_radius-next_stage.rotor.attachment_height * self.shaft_connect_padding_to_attachment_height

        lock_screw = SocketHeadCapScrew(f"M{lock_screw_diameter}-0.4", np.floor(shaft_connect_length+blade_heatset_height), "iso4762")
        shaft_connector_heatset = HeatSetNut(
            size=f"M{lock_screw_diameter}-0.4-3",
            fastener_type="Hilitchi",
            simple=True,
        )
        
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
                .polarArray(1.01*stage.rotor.hub_radius, 0, 360, stage.rotor.number_of_blades)
                .eachpoint(
                    lambda loc: (
                        cq.Workplane("XY")
                        .polyline(stage.rotor.attachment * self.attachment_insert_scale_factor)  # type: ignore
                        .close()
                        .rotate((0, 0, 0), (0, 0, 1), 270)
                    ).val().located(loc), True)  # type: ignore
                .cutBlind(-stage.rotor.disk_height)

                .faces(">Z")
                .workplane(offset=-shaft_connect_height/2)
                .polarArray(0.99*shaft_connect_inner_radius, 0, 360, stage.rotor.number_of_blades)
                .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), -90))
                .clearanceHole(lock_screw, fit="Loose", baseAssembly=base_assembly)

                .faces(">Z")
                .workplane()
                .circle(shaft_connect_outer_radius)
                .extrude(shaft_connect_height)

                .faces(">Z")
                .workplane()
                .circle(shaft_connect_inner_radius)
                .cutThruAll()
                # .cutBlind(-shaft_connect_height)

                .faces(">Z")
                .workplane(offset=-shaft_connect_height/2)
                .polarArray(shaft_connect_outer_radius, 0, 360, self.shaft_connect_screw_quantity)
                .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90))
                .insertHole(shaft_connector_heatset, fit="Loose", baseAssembly=base_assembly, depth=shaft_connector_heatset.nut_thickness)

                .faces("<Z")
                .workplane()
                .circle(next_shaft_connect_outer_radius)
                .cutBlind(-next_shaft_connect_height)

                .faces("<Z")
                .workplane(offset=-next_shaft_connect_height/2)
                .polarArray(next_stage.rotor.hub_radius, 0, 360, self.shaft_connect_screw_quantity)
                .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90))
                .clearanceHole(lock_screw, fit="Loose", baseAssembly=base_assembly)
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
            base_assembly.add(blade_assembly, loc=blade_assembly_loc, name=f"Blade {i+1}")

        return base_assembly

    def shaft_assembly(self, turbomachinery: TurbomachineryExport, include_attachment: bool = True):
        assembly = cq.Assembly()
        stage_height_offset = 0

        for i in range(0, len(turbomachinery.stages)):
            current_stage = turbomachinery.stages[i]
            next_stage = turbomachinery.stages[i+1] if i+1 < len(turbomachinery.stages) else current_stage

            stage_height_offset -= current_stage.stage_height * (1+self.transition_to_total_height)
            stage_shaft_profile = self.shaft_stage_assembly(current_stage, next_stage, include_attachment)
            assembly.add(stage_shaft_profile, loc=cq.Location(cq.Vector(0, 0, stage_height_offset)), name=f"Stage {current_stage.stage_number}")

        return assembly
