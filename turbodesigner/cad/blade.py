import cadquery as cq
import numpy as np
from turbodesigner.blade.row import BladeRowExport

class BladeCadModel:
    @staticmethod
    def blade_profile(
        blade_row: BladeRowExport,
    ):
        blade_profile = (
            cq.Workplane("XY")
            .polyline(blade_row.airfoils[0])
            .close()
        )

        for i in range(0, len(blade_row.radii) - 1):
            blade_profile = (
                blade_profile
                .transformed(offset=cq.Vector(0, 0, blade_row.radii[i+1]-blade_row.radii[i]))
                .polyline(blade_row.airfoils[i+1])
                .close()
            )

        path = (
            cq.Workplane("XZ")
            .lineTo(0, blade_row.tip_radius-blade_row.hub_radius)
        )

        return (
            blade_profile
            .sweep(path, multisection=True, makeSolid=True)
        )