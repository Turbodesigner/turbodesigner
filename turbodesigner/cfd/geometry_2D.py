import numpy as np
from turbodesigner.turbomachinery import Turbomachinery
from turbodesigner.stage import Stage
from turbodesigner.blade.row import BladeRow


class TurbomachineryGeometry2D:
    @staticmethod
    def get_row_outline_components(
        row: BladeRow,
        station_number: int,
        leading_edge_gap: float,
        trailing_edge_gap: float,
        num_blades: int = 2
    ):
        blade_spacing = row.s
        height = blade_spacing * num_blades
        airfoil = row.airfoils[station_number].get_coords()
        airfoil_height: float = np.max(airfoil[:, 1]) - np.min(airfoil[:, 1])
        airfoil_width: float = np.max(airfoil[:, 0]) - np.min(airfoil[:, 0])

        top_outline = np.array([
            [0, height/2],
            [leading_edge_gap, height/2],
            [leading_edge_gap+airfoil_width, height/2 - airfoil_height],
            [leading_edge_gap+airfoil_width+trailing_edge_gap, height/2 - airfoil_height],
        ])

        bottom_outline = np.array([
            [leading_edge_gap+airfoil_width+trailing_edge_gap, -height/2 - airfoil_height],
            [leading_edge_gap+airfoil_width, -height/2 - airfoil_height],
            [leading_edge_gap, -height/2],
            [0, -height/2],
        ])

        return (top_outline, bottom_outline)

    @staticmethod
    def get_row_outline(
        row: BladeRow,
        station_number: int,
        leading_edge_gap: float,
        trailing_edge_gap: float,
        num_blades: int = 2
    ):
        top_outline, bottom_outline = TurbomachineryGeometry2D.get_row_outline_components(
            row,
            station_number,
            leading_edge_gap,
            trailing_edge_gap,
            num_blades
        )

        return np.concatenate((top_outline, bottom_outline))

    @staticmethod
    def get_stage_outline_components(
        stage: Stage,
        next_stage: Stage,
        station_number: int,
        num_blades: int = 2
    ):
        rotor_top_outline, rotor_bottom_outline = TurbomachineryGeometry2D.get_row_outline_components(
            stage.rotor,
            station_number,
            stage.stage_gap/2,
            stage.row_gap/2,
            num_blades,
        )
        stator_top_outline, stator_bottom_outline = TurbomachineryGeometry2D.get_row_outline_components(
            stage.stator,
            station_number,
            stage.row_gap/2,
            next_stage.stage_gap/2
        )

        stator_outline_offset = np.array([
                np.max(rotor_top_outline[:, 0]),
                -(rotor_top_outline[-1][1] - rotor_bottom_outline[0][1])/2
        ])

        stage_top_outline = np.concatenate((
            rotor_top_outline,
            stator_top_outline + stator_outline_offset,
        ))

        stage_bottom_outline = np.concatenate((
            stator_bottom_outline + stator_outline_offset,
            rotor_bottom_outline,
        ))

        return (stage_top_outline, stage_bottom_outline)

    @staticmethod
    def get_stage_outline(
        stage: Stage,
        next_stage: Stage,
        station_number: int,
        num_blades: int = 2
    ):
        top_outline, bottom_outline = TurbomachineryGeometry2D.get_stage_outline_components(
            stage,
            next_stage,
            station_number,
            num_blades,
        )
        return np.concatenate((top_outline, bottom_outline))


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
            stage_top_outline, stage_bottom_outline = TurbomachineryGeometry2D.get_stage_outline_components(
                stage,
                next_stage,
                station_number,
                num_blades
            )

            if i == 0:
                turbomachinery_top_outline = stage_top_outline
                turbomachinery_bottom_outline = stage_bottom_outline
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
                    stage_top_outline + stage_outline_offset,
                ))

                turbomachinery_bottom_outline = np.concatenate((
                    stage_bottom_outline + stage_outline_offset,
                    turbomachinery_bottom_outline,
                ))




        return np.concatenate((turbomachinery_top_outline, turbomachinery_bottom_outline))
