from dataclasses import dataclass
from typing import Optional
import cadquery as cq
import numpy as np
from turbodesigner.blade.row import BladeRowExport
from turbodesigner.cad.common import ExtendedWorkplane
from cq_warehouse.fastener import HeatSetNut

@dataclass
class BladeCadModel:
    attachment_screw_indent_height_to_disk_height: Optional[float] = None
    "blade attachment screw indent to disk height (dimensionless)"

    def fastener_diameter(self, blade_row: BladeRowExport):
        return int(0.1*blade_row.disk_height)

    def heatset_height(self, blade_row: BladeRowExport):
        return np.round(0.5*blade_row.attachment_height)

    def blade_assembly(
        self,
        blade_row: BladeRowExport,
        include_attachment: bool = True
    ):
        base_assembly = cq.Assembly()
        hub_airfoil = blade_row.airfoils[0]
        airfoil_offset = np.array([
            (np.max(hub_airfoil[:, 0]) + np.min(hub_airfoil[:, 0]))/2,
            (np.max(hub_airfoil[:, 1]) + np.min(hub_airfoil[:, 1]))/2
        ])

        blade_profile = cq.Workplane("XY")
        if include_attachment:
            assert self.attachment_screw_indent_height_to_disk_height is not None, "attachment_screw_indent_height_to_disk_height needs to be assigned for include_attachment"
            attachment_screw_indent_height = blade_row.disk_height * self.attachment_screw_indent_height_to_disk_height
            heatset_diameter = self.fastener_diameter(blade_row)
            heatset_height =  self.heatset_height(blade_row)
            # heatset = HeatSetNut(
            #     size=f"M{heatset_diameter}-0.4-{heatset_height}",
            #     fastener_type="Hilitchi",
            #     simple=True,
            # )

            blade_profile = (
                ExtendedWorkplane("YZ")
                .polyline(blade_row.attachment)  # type: ignore
                .close()
                .extrude(blade_row.disk_height*0.5, both=True)
                
                # .faces(">Z")
                # .translate()
                # .clearanceHole(heatset, baseAssembly=base_assembly)

                .faces(">Z")
                .workplane()
            )

        blade_profile = (
            blade_profile
            .polyline(hub_airfoil - airfoil_offset)
            .close()
        )

        for i in range(0, len(blade_row.radii) - 1):
            blade_profile = (
                blade_profile
                .transformed(offset=cq.Vector(0, 0, blade_row.radii[i+1]-blade_row.radii[i]))
                .polyline(blade_row.airfoils[i+1] - airfoil_offset)
                .close()
            )

        hub_height_offset = blade_row.hub_radius*np.cos((2*np.pi / blade_row.number_of_blades) / 2)-blade_row.hub_radius
        path = (
            cq.Workplane("XZ")
            .lineTo(hub_height_offset, blade_row.tip_radius-blade_row.hub_radius)
        )

        blade_profile = (
            blade_profile
            .sweep(path, multisection=True, makeSolid=True)
        )

        base_assembly.add(blade_profile, name="Blade")
        return base_assembly
