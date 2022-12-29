from dataclasses import dataclass
from functools import cached_property
from typing import Optional
from turbodesigner.cad.blade import BladeCadModel, BladeCadModelSpecification
from turbodesigner.cad.common import ExtendedWorkplane, FastenerPredicter
from turbodesigner.stage import StageExport
import cadquery as cq
from turbodesigner.turbomachinery import TurbomachineryExport

@dataclass
class CasingCadModelSpecifciation:
    include_attachment: bool = True
    "whether to include attachment or fuse into shaft (bool)"

    casing_thickness_to_inlet_radius: float = 0.25
    "casing thickness to tip radius of first stage (dimensionless)"

    casing_transition_to_total_height: float = 0.25
    "casing transition height to total stage height (dimensionless)"

    casing_connect_height_to_screw_head_diameter: float = 1.75
    "casing connect to disk height (dimensionless)"

    casing_connect_padding_to_attachment_height: float = 1.25
    "casing connect padding to attachment height (dimensionless)"

    casing_connect_heatset_diameter_to_disk_height: float = 0.25
    "casing connect heatset diameter to disk height (dimensionless)"

    casing_connect_screw_quantity: int = 4
    "casing connect screw quantity (dimensionless)"


@dataclass
class CasingCadModel:
    stage: StageExport
    "turbomachinery stage"

    first_stage: StageExport
    "turbomachinery first stage"

    previous_stage: Optional[StageExport] = None
    "turbomachinery next stage"

    spec: CasingCadModelSpecifciation = CasingCadModelSpecifciation()
    "casing cad model specification"

    def __post_init__(self):
        self.casing_thickness = self.spec.casing_thickness_to_inlet_radius * self.first_stage.rotor.tip_radius
        self.transition_height = self.stage.stage_height * self.spec.casing_transition_to_total_height
        self.casing_height = self.stage.stage_height + self.transition_height


        self.blade_cad_model = BladeCadModel(
            self.stage.stator,
            spec=BladeCadModelSpecification(
                self.spec.include_attachment,
                # shaft_connect_length=self.casing_connect_length
            )
        )

        self.casing_connect_padding = self.spec.casing_connect_padding_to_attachment_height * self.stage.stator.attachment_height
        self.casing_connect_heatset = FastenerPredicter.predict_heatset(
            target_diameter=self.stage.rotor.disk_height*self.spec.casing_connect_heatset_diameter_to_disk_height,
        )

        self.casing_connect_length = self.casing_thickness - self.casing_connect_padding - self.stage.stator.attachment_height

        self.casing_connect_screw = FastenerPredicter.predict_screw(
            target_diameter=self.casing_connect_heatset.thread_diameter,
            target_length=self.casing_connect_length+self.casing_connect_padding+self.stage.stator.attachment_height*0.5
        )

        self.casing_connect_height = self.casing_connect_screw.head_diameter * self.spec.casing_connect_height_to_screw_head_diameter
        self.casing_connect_outer_radius = self.first_stage.rotor.tip_radius+self.casing_thickness
        self.casing_connect_inner_radius =  self.casing_connect_outer_radius-self.casing_connect_length

        if self.previous_stage:
            self.previous_stage_casing_cad_model = CasingCadModel(self.previous_stage, self.first_stage, spec=self.spec)
            # self.next_stage_casing_connector_screw = FastenerPredicter.predict_screw(
            #     target_diameter=self.next_stage_shaft_cad_model.shaft_connector_heatset.thread_diameter,
            #     target_length=self.next_stage_shaft_cad_model.shaft_connector_heatset.nut_thickness + (self.stage.stator.hub_radius - self.next_stage_shaft_cad_model.shaft_connect_outer_radius)
            # )


    @cached_property
    def casing_stage_assembly(self):
        base_assembly = cq.Assembly()
        blade_assembly = cq.Assembly()
        fastener_assembly = cq.Assembly()

        path = (
            cq.Workplane("XZ")
            .moveTo(0, 0)
            .lineTo(0, self.stage.stage_height*(1+self.spec.casing_transition_to_total_height))
        )

        casing_cut_profile = (
            # Stator Disk
            ExtendedWorkplane("XY")
            .circle(self.stage.stator.tip_radius*1.001)
            .extrude(self.stage.stator.disk_height)

            # Transition Disk
            .faces(">Z")
            .workplane()
            .truncated_cone(
                start_radius=self.stage.stator.tip_radius,
                end_radius=self.stage.rotor.tip_radius,
                height=self.transition_height
            )

            # Rotor Disk
            .faces(">Z")
            .workplane()
            .circle(self.stage.rotor.tip_radius)
            .extrude(self.stage.rotor.disk_height)
        )
        casing_profile = (
            ExtendedWorkplane("XY")
            .circle(self.first_stage.rotor.tip_radius + self.casing_thickness)
            .extrude(self.casing_height)
            .cut(casing_cut_profile)
        )

        if self.spec.include_attachment:
            casing_profile = (
                casing_profile

                # Cut Attachments - TODO: make this operation faster
                # .faces("<Z")
                # .workplane()
                # .polarArray(self.stage.stator.tip_radius, 0, 360, self.stage.stator.number_of_blades)
                # .eachpoint(
                #     lambda loc: (
                #         cq.Workplane("XY")
                #         .polyline(self.stage.stator.attachment_with_tolerance)  # type: ignore
                #         .close()
                #         .rotate((0, 0, 0), (0, 0, 1), 90)

                #     ).val().located(loc), True)  # type: ignore
                # .cutBlind(-self.stage.stator.disk_height)

                # Blade Lock Screws
                # .faces("<Z")
                # .workplane(offset=-self.blade_cad_model.lock_screw.head_diameter*1.5)
                # .polarArray(self.casing_connect_outer_radius*0.99, 0, 360, self.stage.stator.number_of_blades)
                # .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90))
                # .clearanceHole(self.blade_cad_model.lock_screw, fit="Loose", baseAssembly=fastener_assembly)

                # Previous Stage Shaft Connect
                .faces(">Z")
                .workplane()
                .circle(self.previous_stage_casing_cad_model.casing_connect_outer_radius)
                .circle(self.previous_stage_casing_cad_model.casing_connect_inner_radius)
                .cutBlind(-self.previous_stage_casing_cad_model.casing_connect_height)

                # Previous Stage Shaft Connect Heatsets
                .faces(">Z")
                .workplane(offset=-self.previous_stage_casing_cad_model.casing_connect_height/2)
                .polarArray(self.previous_stage_casing_cad_model.casing_connect_inner_radius, 0, 360, self.spec.casing_connect_screw_quantity)
                .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90))
                .insertHole(self.previous_stage_casing_cad_model.casing_connect_heatset, fit="Loose", baseAssembly=fastener_assembly, depth=self.previous_stage_casing_cad_model.casing_connect_heatset.nut_thickness)

                # Stage Shaft Connect
                .faces("<Z")
                .workplane()
                .circle(self.casing_connect_outer_radius)
                .circle(self.casing_connect_inner_radius)
                .extrude(self.casing_connect_height)

                # Stage Shaft Connect Screws
                .faces("<Z")
                .workplane(offset=-self.casing_connect_height/2)
                .polarArray(self.casing_connect_outer_radius, 0, 360, self.spec.casing_connect_screw_quantity)
                .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90))
                .clearanceHole(self.casing_connect_screw, fit="Loose", baseAssembly=fastener_assembly)

            )


        
        # .transformed(rotate=(90, 0, 0))
        # .split(keepBottom=True)

        # blade_vertical_offset = self.stage.stator.disk_height/2
        # blade_assembly_locs = (
        #     ExtendedWorkplane("XY")
        #     .polarArray(self.stage.stator.hub_radius, 0, 360, self.stage.stator.number_of_blades)
        #     .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, blade_vertical_offset), cq.Vector(0, 1, 0), 90))
        #     .vals()
        # )

        # for (i, blade_assembly_loc) in enumerate(blade_assembly_locs):
        #     assert isinstance(blade_assembly_loc, cq.Location)
        #     blade_assembly.add(self.blade_cad_model.blade_assembly, loc=blade_assembly_loc, name=f"Blade {i+1}")
        
        base_assembly.add(casing_profile, name=f"Stage Casing")
        # base_assembly.add(blade_assembly, name="Blades")
        base_assembly.add(fastener_assembly, name="Fasteners")

        return base_assembly


    # @staticmethod
    # def casing_assembly(
    #     turbomachinery: TurbomachineryExport, 
    #     spec: CasingCadModelSpecifciation = CasingCadModelSpecifciation()
    # ):
    #     assembly = cq.Assembly()
    #     stage_height_offset = 0
    #     first_stage = turbomachinery.stages[0]

    #     for i in range(0, len(turbomachinery.stages)):
    #         current_stage = turbomachinery.stages[i]
    #         next_stage = turbomachinery.stages[i+1] if i+1 < len(turbomachinery.stages) else current_stage

    #         stage_height_offset -= current_stage.stage_height * (1+spec.casing_transition_to_total_height)
    #         casing_cad_model = CasingCadModel(current_stage, first_stage, next_stage, spec)
    #         assembly.add(casing_cad_model.casing_stage_assembly, loc=cq.Location(cq.Vector(0, 0, stage_height_offset)), name=f"Stage {current_stage.stage_number}")

    #     return assembly

        
        
