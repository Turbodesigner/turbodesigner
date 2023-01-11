import unittest
import numpy as np
from tests.designs import base_design
from turbodesigner.flow_station import FlowStation
from turbodesigner.stage import Stage


class CompressorDesignTest(unittest.TestCase):
    def test_flow_station_base_design(self):
        np.testing.assert_almost_equal(base_design.outlet_flow_station.Pt, 419150)
        np.testing.assert_almost_equal(base_design.inlet_flow_station.P, 87908.56, 2)
        np.testing.assert_almost_equal(base_design.outlet_flow_station.P, 383948.29, 2)
        np.testing.assert_almost_equal(base_design.inlet_flow_station.T, 276.80039821)
        np.testing.assert_almost_equal(base_design.outlet_flow_station.T, 441.27919465)
        np.testing.assert_almost_equal(base_design.inlet_flow_station.rho, 1.10657931)
        np.testing.assert_almost_equal(base_design.outlet_flow_station.rho, 3.03163833)
        np.testing.assert_almost_equal(base_design.inlet_flow_station.A_flow, 0.12049144)
        np.testing.assert_almost_equal(base_design.inlet_flow_station.A_phys, 0.12049144)
        np.testing.assert_almost_equal(base_design.outlet_flow_station.A_flow, 0.04398062)
        np.testing.assert_almost_equal(base_design.outlet_flow_station.A_phys, 0.044, 3)
        np.testing.assert_almost_equal(base_design.outlet_flow_station.Tt, 452.47879644)
        np.testing.assert_almost_equal(base_design.Delta_T0, 164.47879644)
        np.testing.assert_almost_equal(base_design.inlet_flow_station.inner_radius, 0.11307, 5)
        np.testing.assert_almost_equal(base_design.outlet_flow_station.inner_radius, 0.14897, 5)
        np.testing.assert_almost_equal(base_design.inlet_flow_station.outer_radius, 0.22614, 5)
        np.testing.assert_almost_equal(base_design.outlet_flow_station.outer_radius, 0.19024, 5)
        np.testing.assert_almost_equal(base_design.inlet_flow_station.radius, 0.16960, 5)
        np.testing.assert_almost_equal(base_design.outlet_flow_station.radius, 0.16960, 5)
        np.testing.assert_almost_equal(base_design.inlet_flow_station.N, 15000)


if __name__ == '__main__':
    unittest.main()
