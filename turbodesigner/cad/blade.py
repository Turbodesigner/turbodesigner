import cadquery as cq
import numpy as np
from turbodesigner.blade.row import BladeRowExport

class BladeCadModel:
    @staticmethod
    def blade_profile(
        blade_row: BladeRowExport,
        include_attachment: bool = True
    ):
        hub_airfoil = blade_row.airfoils[0]
        airfoil_offset = np.array([
            (np.max(hub_airfoil[:, 0]) + np.min(hub_airfoil[:, 0]))/2,
            (np.max(hub_airfoil[:, 1]) + np.min(hub_airfoil[:, 1]))/2
        ])

        blade_profile = cq.Workplane("YZ")
        if include_attachment:
            blade_profile = (
                blade_profile
                .polyline(blade_row.attachment)  # type: ignore
                .close()
                .extrude(blade_row.disk_height*0.5, both=True)
                .faces(">Z")
                .workplane()
            )

        blade_profile = (
            blade_profile
            .polyline(blade_row.airfoils[0] - airfoil_offset)
            .close()
        )

        for i in range(0, len(blade_row.radii) - 1):
            blade_profile = (
                blade_profile
                .transformed(offset=cq.Vector(0, 0, blade_row.radii[i+1]-blade_row.radii[i]))
                .polyline(blade_row.airfoils[i+1] - airfoil_offset)
                .close()
            )

        path = (
            cq.Workplane("XZ")
            .lineTo(0, blade_row.tip_radius-blade_row.hub_radius)
        )

        blade_profile = (
            blade_profile
            .sweep(path, multisection=True, makeSolid=True)
        )
        
        return blade_profile