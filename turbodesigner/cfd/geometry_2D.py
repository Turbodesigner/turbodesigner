from dataclasses import dataclass
from functools import cached_property
import numpy as np
from turbodesigner.turbomachinery import Turbomachinery
from turbodesigner.stage import Stage
from turbodesigner.blade.row import BladeRow
from plotly import graph_objects as go


@dataclass
class CFDGeometry:
    "A row of blades for CFD simulation"
    top_outline: np.ndarray
    "top outline of the row"
    bottom_outline: np.ndarray
    "bottom outline of the row"
    airfoils: list[np.ndarray]
    "airfoil coordinates for row"

    @cached_property
    def coords(self):
        return np.concatenate((self.top_outline, self.bottom_outline))

    def visualize(self):
        fig = go.Figure(
            layout=go.Layout(title=go.layout.Title(text="2D CFD Geometry"))
        )
        fig.add_trace(go.Scatter(
            x=self.coords[:, 0],
            y=self.coords[:, 1],
            
            fill="toself"
        ))

        for airfoil in self.airfoils:
            fig.add_trace(go.Scatter(
                x=airfoil[:, 0],
                y=airfoil[:, 1],
                fill="toself",
            ))
        
        

        fig.layout.yaxis.scaleanchor = "x"  # type: ignore
        fig.show()


class TurbomachineryGeometry2D:
    @staticmethod
    def row_geometry(
        row: BladeRow,
        station_number: int,
        leading_edge_gap: float,
        trailing_edge_gap: float,
        num_blades: int = 2
    ):
        blade_spacing = row.s
        height = blade_spacing * num_blades
        airfoil = row.airfoils[station_number].get_coords()
        airfoil_width: float = np.max(airfoil[:, 0]) - np.min(airfoil[:, 0])
        airfoil_leading_pnt = airfoil[np.argmin(airfoil[:, 0])]
        airfoil_trailing_pnt = airfoil[np.argmax(airfoil[:, 0])]

        airfoils = []
        airfoil_offset = np.array([
            leading_edge_gap-np.min(airfoil[:, 0]), 
            -airfoil_leading_pnt[1]+(height/2) -(blade_spacing/2)
        ])
        for i in range(num_blades):
            airfoils.append(airfoil+airfoil_offset-np.array([0, i*blade_spacing]))

        airfoil_trailing_height_offset = airfoil_trailing_pnt[1] - airfoil_leading_pnt[1]
        return CFDGeometry(
            top_outline=np.array([
                [0, height/2],
                [leading_edge_gap, height/2],
                [leading_edge_gap+airfoil_width, height/2 + airfoil_trailing_height_offset],
                [leading_edge_gap+airfoil_width+trailing_edge_gap, height/2 + airfoil_trailing_height_offset],
            ]),
            bottom_outline=np.array([
                [leading_edge_gap+airfoil_width+trailing_edge_gap, -height/2 + airfoil_trailing_height_offset],
                [leading_edge_gap+airfoil_width, -height/2 + airfoil_trailing_height_offset],
                [leading_edge_gap, -height/2],
                [0, -height/2],
            ]),
            airfoils=airfoils
        )

    @staticmethod
    def stage_geometry(
        stage: Stage,
        next_stage: Stage,
        station_number: int,
        num_blades: int = 2
    ):
        rotor_geometry = TurbomachineryGeometry2D.row_geometry(stage.rotor, station_number, stage.stage_gap/2, stage.row_gap/2, num_blades)
        stator_geometry = TurbomachineryGeometry2D.row_geometry(stage.stator, station_number, stage.row_gap/2, next_stage.stage_gap/2, num_blades)

        rotor_airfoil = rotor_geometry.airfoils[0]
        stator_airfoil = rotor_geometry.airfoils[0]

        rotor_airfoil_trailing_pnt = rotor_airfoil[np.argmax(rotor_airfoil[:, 0])]
        stator_airfoil_leading_pnt = stator_airfoil[np.argmin(stator_airfoil[:, 0])]

        stator_offset = np.array([
            np.max(rotor_geometry.top_outline[:, 0]),
            rotor_airfoil_trailing_pnt[1] - stator_airfoil_leading_pnt[1]
        ])

        stator_airfoils = [airfoil + stator_offset for airfoil in stator_geometry.airfoils]
        return CFDGeometry(
            top_outline=np.concatenate((
                rotor_geometry.top_outline,
                stator_geometry.top_outline + stator_offset,
            )),
            bottom_outline=np.concatenate((
                stator_geometry.bottom_outline + stator_offset,
                rotor_geometry.bottom_outline,
            )),
            airfoils=rotor_geometry.airfoils + stator_airfoils
        )

    @staticmethod
    def get_turbomachinery_outline(
        turbomachinery: Turbomachinery,
        station_number: int,
        num_blades: int = 2
    ):
        turbomachinery_top_outline = np.array([])
        turbomachinery_bottom_outline = np.array([])

        for i in range(turbomachinery.N_stg):
            stage = turbomachinery.stages[i]
            next_stage = turbomachinery.stages[i+1] if i+1 < turbomachinery.N_stg else stage
            stage_geometry = TurbomachineryGeometry2D.stage_geometry(
                stage,
                next_stage,
                station_number,
                num_blades
            )

            if i == 0:
                turbomachinery_top_outline = stage_geometry.top_outline
                turbomachinery_bottom_outline = stage_geometry.bottom_outline
            else:
                stage_outline_offset = np.array(
                    [
                        np.max(turbomachinery_top_outline[:, 0]),
                        0
                        # turbomachinery_top_outline[-1][1] - turbomachinery_bottom_outline[0][1]
                    ]
                )
                turbomachinery_top_outline = np.concatenate((
                    turbomachinery_top_outline,
                    stage_geometry.top_outline + stage_outline_offset,
                ))

                turbomachinery_bottom_outline = np.concatenate((
                    stage_geometry.bottom_outline + stage_outline_offset,
                    turbomachinery_bottom_outline,
                ))

        return np.concatenate((turbomachinery_top_outline, turbomachinery_bottom_outline))
