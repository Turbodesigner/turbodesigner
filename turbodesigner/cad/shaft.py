from dataclasses import dataclass
from functools import cached_property
import cadquery as cq
from turbodesigner.cad.common import ExtendedWorkplane, FastenerPredicter
from turbodesigner.cad.blade import BladeCadModel, BladeCadModelSpecification
from turbodesigner.stage import StageExport
from turbodesigner.turbomachinery import TurbomachineryExport

@dataclass
class ShaftCadModelSpecification:
    include_attachment: bool = True
    "whether to include attachment or fuse into shaft (bool)"

    transition_to_total_height: float = 0.25
    "casing transition height to total stage height (dimensionless)"

    shaft_connect_length_to_hub_radius: float = 0.15
    "shaft connect to hub radius (dimensionless)"

    shaft_connect_height_to_disk_height: float = 0.25
    "shaft connect to disk height (dimensionless)"

    shaft_connect_padding_to_attachment_height: float = 1.2
    "shaft connect padding to attachment height (dimensionless)"

    shaft_fastener_diameter_to_connect_height: float = 0.25
    "shaft fastener diameter to connect height (dimensionless)"

    shaft_heaset_height_to_connect_length: float = 0.75
    "shaft heatset height to connect length (dimensionless)"

    shaft_connect_screw_quantity: int = 4
    "shaft connect screw quantity (dimensionless)"

    attachment_insert_scale_factor: float = 1.05
    "rotor blade attachment tolerance scale factor (dimensionless)"


@dataclass
class ShaftCadModel:
    stage: StageExport
    "turbomachinery stage"

    next_stage: StageExport
    "next turbomachinery stage"

    spec: ShaftCadModelSpecification = ShaftCadModelSpecification()
    "shaft cad model specification"

    def __post_init__(self):
        self.transition_height = self.stage.stage_height * self.spec.transition_to_total_height

        self.shaft_connect_length = self.stage.rotor.hub_radius * self.spec.shaft_connect_length_to_hub_radius
        self.shaft_connect_height = self.stage.rotor.disk_height * self.spec.shaft_connect_height_to_disk_height
        shaft_connect_padding = self.stage.rotor.attachment_height * self.spec.shaft_connect_padding_to_attachment_height
        self.shaft_connect_outer_radius = self.stage.rotor.hub_radius-shaft_connect_padding
        self.shaft_connect_inner_radius = self.shaft_connect_outer_radius-self.shaft_connect_length

        self.next_shaft_connect_length = self.next_stage.rotor.hub_radius * self.spec.shaft_connect_length_to_hub_radius
        self.next_shaft_connect_height = self.next_stage.rotor.disk_height * self.spec.shaft_connect_height_to_disk_height
        next_shaft_connect_padding = self.next_stage.rotor.attachment_height * self.spec.shaft_connect_padding_to_attachment_height
        self.next_shaft_connect_outer_radius = self.next_stage.rotor.hub_radius-next_shaft_connect_padding

        self.blade_cad_model = BladeCadModel(
            self.stage.rotor,
            spec=BladeCadModelSpecification(
                self.spec.include_attachment,
                shaft_connect_length_to_hub_radius=self.spec.shaft_connect_length_to_hub_radius
            )
        )
        # self.blade_cad_model.lock_screw
        self.shaft_connector_heatset = FastenerPredicter.predict_heatset(
            target_diameter=self.shaft_connect_height*self.spec.shaft_fastener_diameter_to_connect_height,
            max_nut_thickness=self.shaft_connect_length*self.spec.shaft_heaset_height_to_connect_length
        )

        self.next_stage_shaft_connector_screw = FastenerPredicter.predict_screw(
            target_diameter=self.next_shaft_connect_height*self.spec.shaft_fastener_diameter_to_connect_height,
            target_length=next_shaft_connect_padding+self.next_shaft_connect_length*self.spec.shaft_heaset_height_to_connect_length
        )

    @cached_property
    def shaft_stage_assembly(self):
        base_assembly = cq.Assembly()

        shaft_profile = (
            ExtendedWorkplane("XY")
            .circle(self.stage.stator.hub_radius)
            .extrude(self.stage.stator.disk_height)

            .faces(">Z")
            .workplane()
            .truncated_cone(
                start_radius=self.stage.stator.hub_radius,
                end_radius=self.stage.rotor.hub_radius,
                height=self.transition_height
            )

            .faces(">Z")
            .workplane()
            .circle(self.stage.rotor.hub_radius)
            .extrude(self.stage.rotor.disk_height)
        )

        if self.spec.include_attachment:
            shaft_profile = (
                shaft_profile
                # .faces(">Z")
                # .workplane()
                # .polarArray(1.01*self.stage.rotor.hub_radius, 0, 360, self.stage.rotor.number_of_blades)
                # .eachpoint(
                #     lambda loc: (
                #         cq.Workplane("XY")
                #         .polyline(self.stage.rotor.attachment * self.spec.attachment_insert_scale_factor)  # type: ignore
                #         .close()
                #         .rotate((0, 0, 0), (0, 0, 1), 270)
                #     ).val().located(loc), True)  # type: ignore
                # .cutBlind(-self.stage.rotor.disk_height)

                # TODO: make the points here have countersink clean cut
                .faces(">Z")
                .workplane(offset=-self.shaft_connect_height/2)
                .polarArray(self.shaft_connect_inner_radius, 0, 360, self.stage.rotor.number_of_blades)
                .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), -90))
                .clearanceHole(self.blade_cad_model.lock_screw, fit="Loose", baseAssembly=base_assembly)

                .faces(">Z")
                .workplane()
                .circle(self.shaft_connect_outer_radius)
                .extrude(self.shaft_connect_height)

                .faces(">Z")
                .workplane()
                .circle(self.shaft_connect_inner_radius)
                .cutThruAll()
                # .cutBlind(-shaft_connect_height)

                .faces(">Z")
                .workplane(offset=-self.shaft_connect_height/2)
                .polarArray(self.shaft_connect_outer_radius, 0, 360, self.spec.shaft_connect_screw_quantity)
                .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90))
                .insertHole(self.shaft_connector_heatset, fit="Loose", baseAssembly=base_assembly, depth=self.shaft_connector_heatset.nut_thickness)

                .faces("<Z")
                .workplane()
                .circle(self.next_shaft_connect_outer_radius)
                .cutBlind(-self.next_shaft_connect_height)

                # .faces("<Z")
                # .workplane(offset=-self.next_shaft_connect_height/2)
                # .polarArray(self.next_stage.rotor.hub_radius, 0, 360, self.spec.shaft_connect_screw_quantity)
                # .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90))
                # .clearanceHole(self.next_stage_shaft_connector_screw, fit="Loose", baseAssembly=base_assembly)
            )

        base_assembly.add(shaft_profile, name=f"Stage {self.stage.stage_number} Shaft")
        # rotor_blade_vertical_offset = self.stage.stator.disk_height+self.transition_height+self.stage.rotor.disk_height/2
        # blade_assembly_locs = (
        #     ExtendedWorkplane("XY")
        #     .polarArray(self.stage.rotor.hub_radius, 0, 360, self.stage.rotor.number_of_blades)
        #     .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, rotor_blade_vertical_offset), cq.Vector(0, 1, 0), 90))
        #     .vals()
        # )

        # for (i, blade_assembly_loc) in enumerate(blade_assembly_locs):
        #     assert isinstance(blade_assembly_loc, cq.Location)
        #     base_assembly.add(self.blade_cad_model.blade_assembly, loc=blade_assembly_loc, name=f"Blade {i+1}")

        return base_assembly

    @staticmethod
    def shaft_assembly(turbomachinery: TurbomachineryExport, spec: ShaftCadModelSpecification):
        assembly = cq.Assembly()
        stage_height_offset = 0

        for i in range(0, len(turbomachinery.stages)):
            current_stage = turbomachinery.stages[i]
            next_stage = turbomachinery.stages[i+1] if i+1 < len(turbomachinery.stages) else current_stage

            stage_height_offset -= current_stage.stage_height * (1+spec.transition_to_total_height)
            shaft_cad_model = ShaftCadModel(current_stage, next_stage, spec)
            assembly.add(shaft_cad_model.shaft_stage_assembly, loc=cq.Location(cq.Vector(0, 0, stage_height_offset)), name=f"Stage {current_stage.stage_number}")

        return assembly
