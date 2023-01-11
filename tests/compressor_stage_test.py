import unittest
import numpy as np
from tests.designs import base_design


class CompressorStageTest(unittest.TestCase):
    def test_first_stage_base_design(self):
        next_stage = base_design.stages[1]
        np.testing.assert_almost_equal(next_stage.inlet_flow_station.Tt, 308)
        np.testing.assert_almost_equal(next_stage.mid_flow_station.Tt, 333)
        np.testing.assert_almost_equal(next_stage.inlet_flow_station.T, 296.265, 3)
        np.testing.assert_almost_equal(next_stage.mid_flow_station.T,  313.76513, 5)

        np.testing.assert_almost_equal(next_stage.inlet_flow_station.Pt, 124787.12942, 5)
        np.testing.assert_almost_equal(next_stage.mid_flow_station.Pt, 159563.80095, 5)
        np.testing.assert_almost_equal(next_stage.inlet_flow_station.P, 108924.147, 3)
        np.testing.assert_almost_equal(next_stage.mid_flow_station.P, 129567.45266, 5)
        np.testing.assert_almost_equal(next_stage.inlet_flow_station.rho, 1.281, 3)
        np.testing.assert_almost_equal(next_stage.mid_flow_station.rho,  1.43883, 5)

        np.testing.assert_almost_equal(next_stage.inlet_flow_station.inner_radius, 0.121, 3)
        np.testing.assert_almost_equal(next_stage.inlet_flow_station.outer_radius, 0.218, 3)
        np.testing.assert_almost_equal(next_stage.inlet_flow_station.radius, 0.1696031)
        np.testing.assert_almost_equal(next_stage.mid_flow_station.inner_radius, 0.12612, 5)
        np.testing.assert_almost_equal(next_stage.mid_flow_station.outer_radius, 0.21308, 5)
        np.testing.assert_almost_equal(next_stage.mid_flow_station.radius, 0.1696031)


if __name__ == '__main__':
    unittest.main()
