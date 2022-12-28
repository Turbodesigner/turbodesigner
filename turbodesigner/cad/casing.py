from dataclasses import dataclass
from functools import cached_property
from turbodesigner.cad.blade import BladeCadModel, BladeCadModelSpecification
from turbodesigner.cad.common import ExtendedWorkplane
from turbodesigner.stage import StageExport
import cadquery as cq
from turbodesigner.turbomachinery import TurbomachineryExport

@dataclass
class CasingCadModelSpecifciation:
    casing_thickness_to_inlet_radius: float = 0.25
    "casing thickness to tip radius of first stage (dimensionless)"

    casing_transition_to_total_height: float = 0.25
    "casing transition height to total stage height (dimensionless)"

    blade: BladeCadModelSpecification = BladeCadModelSpecification()
    "stator blade cad model specification"


@dataclass
class CasingCadModel:
    stage: StageExport
    "turbomachinery stage"
    
    first_stage: StageExport
    "turbomachinery first stage"

    spec: CasingCadModelSpecifciation = CasingCadModelSpecifciation()
    "casing cad model specification"

    def __post_init__(self):
        self.casing_thickness = self.spec.casing_thickness_to_inlet_radius * self.first_stage.rotor.tip_radius
        self.transition_height = self.stage.stage_height * self.spec.casing_transition_to_total_height
        self.casing_height = self.stage.stage_height + self.transition_height
        self.blade_cad_model = BladeCadModel(self.stage.stator, self.spec.blade)
        
    @cached_property
    def casing_stage_assembly(self):
        base_assembly = cq.Assembly()
        blade_assembly = cq.Assembly()

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
            cq.Workplane("XY")
            .circle(self.first_stage.rotor.tip_radius + self.casing_thickness)
            .extrude(self.casing_height)
            .cut(casing_cut_profile)

            .faces("<Z")
            .workplane()
            .polarArray(self.stage.stator.tip_radius, 0, 360, self.stage.stator.number_of_blades)
            .eachpoint(
                lambda loc: (
                    cq.Workplane("XY")
                    .polyline(self.stage.stator.attachment_with_tolerance)  # type: ignore
                    .close()
                    .rotate((0, 0, 0), (0, 0, 1), 90)

                ).val().located(loc), True)  # type: ignore
            .cutBlind(-self.stage.stator.disk_height)
            
        )

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
        
        base_assembly.add(casing_profile, name=f"Stage Casing")
        base_assembly.add(blade_assembly, name="Blades")
        return base_assembly


    @staticmethod
    def casing_assembly(
        turbomachinery: TurbomachineryExport, 
        spec: CasingCadModelSpecifciation = CasingCadModelSpecifciation()
    ):
        assembly = cq.Assembly()
        stage_height_offset = 0
        first_stage = turbomachinery.stages[0]

        for i in range(0, len(turbomachinery.stages)):
            current_stage = turbomachinery.stages[i]

            stage_height_offset -= current_stage.stage_height * (1+spec.casing_transition_to_total_height)
            casing_cad_model = CasingCadModel(current_stage, first_stage, spec)
            assembly.add(casing_cad_model.casing_stage_assembly, loc=cq.Location(cq.Vector(0, 0, stage_height_offset)), name=f"Stage {current_stage.stage_number}")

        return assembly

        
        
