from dataclasses import dataclass
from functools import cached_property
from typing import Optional
from turbodesigner.cad.blade import BladeCadModel, BladeCadModelSpecification
from turbodesigner.cad.common import ExtendedWorkplane, FastenerPredicter
from turbodesigner.turbomachinery import TurbomachineryExport
from turbodesigner.stage import StageExport
import cadquery as cq
import numpy as np

@dataclass
class CasingCadModelSpecifciation:
    is_simple: bool = False
    "create simple assembly (bool)"

    casing_thickness_to_inlet_radius: float = 0.25
    "casing thickness to tip radius of first stage (dimensionless)"

    casing_transition_to_total_height: float = 0.25
    "casing transition height to total stage height (dimensionless)"

    stage_connect_height_to_screw_head_diameter: float = 1.75
    "casing stage connect to disk height (dimensionless)"

    stage_connect_padding_to_attachment_height: float = 1.25
    "casing stage connect padding to attachment height (dimensionless)"

    stage_connect_heatset_diameter_to_disk_height: float = 0.25
    "casing stage connect heatset diameter to disk height (dimensionless)"

    stage_connect_screw_quantity: int = 4
    "casing stage connect screw quantity (dimensionless)"

    half_connect_width_to_casing_thickness: float = 0.5
    "casing half connect width to casing thickness"

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
        self.transition_height = self.stage.stage_height * self.spec.casing_transition_to_total_height
        self.stage_connect_heatset = FastenerPredicter.predict_heatset(
            target_diameter=self.stage.rotor.disk_height*self.spec.stage_connect_heatset_diameter_to_disk_height,
        )
        self.casing_thickness = self.spec.casing_thickness_to_inlet_radius * self.first_stage.rotor.tip_radius
        self.casing_radius = self.first_stage.rotor.tip_radius+self.casing_thickness
        stage_connect_padding = self.spec.stage_connect_padding_to_attachment_height * self.stage.stator.attachment_height
        self.stage_connect_length = self.casing_thickness - stage_connect_padding - self.stage.stator.attachment_height

        self.blade_cad_model = BladeCadModel(
            self.stage.stator,
            spec=BladeCadModelSpecification(
                not self.spec.is_simple,
                screw_length_padding=self.casing_thickness-self.stage.stator.attachment_height
            )
        )

        self.stage_connect_screw = FastenerPredicter.predict_screw(
            target_diameter=self.stage_connect_heatset.thread_diameter,
            target_length=self.stage_connect_length+self.stage_connect_heatset.nut_thickness
        )

        self.stage_connect_height = self.stage_connect_screw.head_diameter * self.spec.stage_connect_height_to_screw_head_diameter
        self.stage_connect_outer_radius = self.casing_radius
        self.stage_connect_inner_radius =  self.stage_connect_outer_radius-self.stage_connect_length

        self.previous_stage_casing_cad_model: Optional[CasingCadModel] = None
        if self.previous_stage:
            self.previous_stage_casing_cad_model = CasingCadModel(self.previous_stage, self.first_stage, spec=self.spec)

        # Half Connector
        self.sector_angle = 360 / self.stage.stator.number_of_blades
        self.half_connect_thickness = self.casing_radius*np.sin(np.radians(self.sector_angle) / 2)
        self.half_connect_width = self.casing_thickness*self.spec.half_connect_width_to_casing_thickness
        self.half_connect_height = self.stage.stage_height + self.transition_height
        self.half_connect_heatset = FastenerPredicter.predict_heatset(
            target_diameter=self.half_connect_width*0.5,
        )
        self.half_connect_screw = FastenerPredicter.predict_screw(
            target_diameter=self.half_connect_heatset.thread_diameter,
            target_length=self.half_connect_thickness*0.75+self.half_connect_heatset.nut_thickness
        )



    @cached_property
    def casing_stage_assembly(self):
        base_assembly = cq.Assembly()
        blade_assembly = cq.Assembly()
        fastener_assembly = cq.Assembly()

        casing_cut_profile = (
            # Stator Disk
            ExtendedWorkplane("XY")
            .transformed(offset=(0,0,-self.stage_connect_height))
            .circle(self.stage.stator.tip_radius*1.001)
            .extrude(self.stage.stator.disk_height+self.stage_connect_height)

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
            .extrude(self.half_connect_height)
        )

        if not self.spec.is_simple:
            casing_profile = (
                casing_profile
                
                # Stage Shaft Connect
                .faces("<Z")
                .workplane()
                .circle(self.stage_connect_outer_radius)
                .circle(self.stage_connect_inner_radius)
                .extrude(self.stage_connect_height)

                # Add Half Connect
                .add(
                    cq.Workplane("XY")
                    .transformed(rotate=(0,0,90), offset=(0,0,-self.stage_connect_height))
                    .box(self.half_connect_thickness*2,(self.casing_radius+self.half_connect_width)*2, self.half_connect_height, centered=(True, True, False))
                )
            )

        casing_profile = (
            casing_profile
            .cut(casing_cut_profile)
        )

        if not self.spec.is_simple:
            casing_profile = (
                casing_profile
                # Cut Attachments - TODO: make this operation faster
                .faces("<Z")
                .workplane(offset=-self.stage_connect_height)
                .transformed(rotate=(0,0,-self.sector_angle/2))
                .polarArray(self.stage.stator.tip_radius, 0, 360, self.stage.stator.number_of_blades)
                .eachpoint(
                    lambda loc: (
                        cq.Workplane("XY")
                        .polyline(self.stage.stator.attachment_with_tolerance)  # type: ignore
                        .close()
                        .rotate((0, 0, 0), (0, 0, 1), 90)

                    ).val().located(loc), True)  # type: ignore
                .cutBlind(-self.stage.stator.disk_height)
                
                # Blade Lock Screws
                .faces("<Z")
                .workplane(offset=-self.stage_connect_height-self.stage.stator.attachment_height)
                .transformed(rotate=(0,0,-self.sector_angle/2))
                .polarArray(self.stage_connect_outer_radius, 0, 360, self.stage.stator.number_of_blades)
                .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90))
                .clearanceHole(self.blade_cad_model.lock_screw, depth=self.blade_cad_model.lock_screw.length, fit="Loose", baseAssembly=fastener_assembly)

                # Stage Shaft Connect
                .faces("<Z")
                .workplane()
                .circle(self.stage_connect_inner_radius)
                .cutBlind(-self.stage_connect_height)

                # Stage Shaft Connect Screws
                .faces("<Z")
                .workplane(offset=-self.stage_connect_height/2)
                .transformed(rotate=(0,0,45))
                .polarArray(self.stage_connect_outer_radius, 0, 360, self.spec.stage_connect_screw_quantity)
                .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90))
                .clearanceHole(self.stage_connect_screw, fit="Loose", baseAssembly=fastener_assembly)
            )
            if self.previous_stage_casing_cad_model:
                casing_profile = (
                    casing_profile
                    # Previous Stage Shaft Connect
                    .faces(">Z")
                    .workplane()
                    .circle(self.previous_stage_casing_cad_model.stage_connect_outer_radius)
                    .circle(self.previous_stage_casing_cad_model.stage_connect_inner_radius)
                    .cutBlind(-self.previous_stage_casing_cad_model.stage_connect_height)

                    # Previous Stage Shaft Connect Heatsets
                    .faces(">Z")
                    .workplane(offset=-self.previous_stage_casing_cad_model.stage_connect_height/2)
                    .transformed(rotate=(0,0,45))
                    .polarArray(self.previous_stage_casing_cad_model.stage_connect_inner_radius, 0, 360, self.spec.stage_connect_screw_quantity)
                    .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, 0), cq.Vector(0, 1, 0), 90))
                    .insertHole(self.previous_stage_casing_cad_model.stage_connect_heatset, fit="Loose", baseAssembly=fastener_assembly, depth=self.previous_stage_casing_cad_model.stage_connect_heatset.nut_thickness)
                )
            

        if not self.spec.is_simple:
            half_casing_bottom_offset = -(self.stage.stage_height+self.transition_height+self.stage_connect_height)/2
            left_casing_profile = (
                casing_profile
                .transformed(rotate=(90, -45, 0))
                .split(keepBottom=True)
                .faces("<Z")
                .workplane()
                .transformed(rotate=(-90,0,0), offset=(0,0,-self.half_connect_height/2))
                .rect(self.casing_radius*2+self.half_connect_width, self.half_connect_height*0.75, forConstruction=True)
                .vertices()
                .insertHole(self.half_connect_heatset, fit="Loose", baseAssembly=fastener_assembly, depth=self.half_connect_heatset.nut_thickness)
            )

            right_casing_profile = (
                casing_profile
                .transformed(rotate=(90, 135, 0))
                .split(keepBottom=True)
                .faces("<Z")
                .workplane()
                .transformed(rotate=(-90,0,180), offset=(0,self.half_connect_thickness,-self.half_connect_height/2))
                .rect(self.casing_radius*2+self.half_connect_width, self.half_connect_height*0.75, forConstruction=True)
                .vertices()
                .clearanceHole(self.half_connect_screw, depth=self.half_connect_screw.length, fit="Loose", baseAssembly=fastener_assembly)
            )
            
            base_assembly.add(left_casing_profile, name=f"Left Casing")
            base_assembly.add(right_casing_profile, name=f"Right Casing")
        else:
            base_assembly.add(casing_profile, name=f"Casing")

        blade_vertical_offset = self.stage.stator.disk_height/2
        blade_assembly_locs = (
            ExtendedWorkplane("XY")
            .polarArray(self.stage.stator.hub_radius, 0, 360, self.stage.stator.number_of_blades)
            .mutatePoints(lambda loc: loc * cq.Location(cq.Vector(0, 0, blade_vertical_offset), cq.Vector(0, 1, 0), 90))
            .vals()
        )

        for (i, blade_assembly_loc) in enumerate(blade_assembly_locs):
            assert isinstance(blade_assembly_loc, cq.Location)
            blade_assembly.add(self.blade_cad_model.blade_assembly, loc=blade_assembly_loc, name=f"Blade {i+1}")
        blade_assembly.rotate((0, 0, 1), -self.sector_angle/2)  # type: ignore


        base_assembly.add(blade_assembly, name="Blades")
        base_assembly.add(fastener_assembly, name="Fasteners")

        return base_assembly


    @staticmethod
    def casing_assembly(
        turbomachinery: TurbomachineryExport, 
        spec: CasingCadModelSpecifciation = CasingCadModelSpecifciation()
    ):
        assembly = cq.Assembly()
        stage_height_offset = 0
        first_stage = turbomachinery.stages[0]
        previous_stage: Optional[StageExport] = None
        for i in range(0, len(turbomachinery.stages)):
            current_stage = turbomachinery.stages[i]

            stage_height_offset -= current_stage.stage_height * (1+spec.casing_transition_to_total_height)
            casing_cad_model = CasingCadModel(current_stage, first_stage, previous_stage, spec)
            assembly.add(casing_cad_model.casing_stage_assembly, loc=cq.Location(cq.Vector(0, 0, stage_height_offset)), name=f"Stage {current_stage.stage_number}")
            previous_stage = current_stage
        return assembly

        
        
