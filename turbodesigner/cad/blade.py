from dataclasses import dataclass
from functools import cached_property
import cadquery as cq
import numpy as np
from turbodesigner.blade.row import BladeRowExport
from turbodesigner.cad.common import ExtendedWorkplane, FastenerPredicter

@dataclass
class BladeCadModelSpecification:
    include_attachment: bool = True
    "whether to include attachment (bool)"

    shaft_connect_length: float = 0.00
    "shaft connect length (dimensionless)"

    fastener_diameter_to_attachment_bottom_width: float = 0.25
    "blade attachment fastener to disk height (dimensionless)"

@dataclass
class BladeCadModel:
    blade_row: BladeRowExport
    "blade row"

    spec: BladeCadModelSpecification = BladeCadModelSpecification()
    "blade cad model specification"

    @cached_property
    def lock_screw(self):
        return FastenerPredicter.predict_screw(
            target_diameter=self.heatset.thread_diameter,
            target_length=self.spec.shaft_connect_length+self.heatset.nut_thickness
        )

    @cached_property
    def heatset(self):
        return FastenerPredicter.predict_heatset(
            target_diameter=self.spec.fastener_diameter_to_attachment_bottom_width*self.blade_row.attachment_bottom_width,
            max_height=self.blade_row.attachment_height*0.75
        )

    @cached_property
    def blade_assembly(self):
        base_assembly = cq.Assembly()
        fastener_assembly = cq.Assembly()

        start_airfoil = self.blade_row.airfoils[0]
        airfoil_vertical_offset = np.array([
            (np.max(start_airfoil[:, 0]) + np.min(start_airfoil[:, 0]))/2,
            (np.max(start_airfoil[:, 1]) + np.min(start_airfoil[:, 1]))/2
        ])
        
        # Hub Airfoil
        blade_profile = (
            cq.Workplane("XY")
            .polyline(start_airfoil - airfoil_vertical_offset)
            .close()
        )

        # Add all airfoil stations
        for i in range(0, len(self.blade_row.radii) - 1):
            blade_profile = (
                blade_profile
                .transformed(offset=cq.Vector(0, 0, self.blade_row.radii[i+1]-self.blade_row.radii[i]))
                .polyline(self.blade_row.airfoils[i+1] - airfoil_vertical_offset)
                .close()
            )

        if self.blade_row.is_rotating:
            hub_height_offset = self.blade_row.hub_radius*np.cos((2*np.pi / self.blade_row.number_of_blades) / 2)-self.blade_row.hub_radius
        else:
            hub_height_offset = 0

        blade_height = self.blade_row.tip_radius-self.blade_row.hub_radius
        path = (
            cq.Workplane("XZ")
            .lineTo(hub_height_offset, blade_height)
        )

        attachment_workplane = ExtendedWorkplane("YZ")
        if not self.blade_row.is_rotating:
            attachment_workplane = (
                attachment_workplane
                .transformed(rotate=(0,0,180), offset=(0,blade_height,0))
            )
        # Attachment Profile
        attachment_profile = (
            attachment_workplane
            .polyline(self.blade_row.attachment)  # type: ignore
            .close()
            .extrude(self.blade_row.disk_height*0.5, both=True)

            .faces("<Z" if self.blade_row.is_rotating else ">Z")
            .workplane()
            .insertHole(self.heatset, depth=self.heatset.nut_thickness*0.9, baseAssembly=fastener_assembly)
        )

        blade_profile = (
            blade_profile
            .sweep(path, multisection=True, makeSolid=True)
            .add(attachment_profile)
        )


        base_assembly.add(blade_profile, name="Blade")
        base_assembly.add(fastener_assembly, name="Fasteners")

        return base_assembly
