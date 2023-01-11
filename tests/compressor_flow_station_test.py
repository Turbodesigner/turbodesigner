import unittest
import numpy as np
from turbodesigner.flow_station import FlowStation
from turbodesigner.stage import Stage


class CompressorFlowStationTest(unittest.TestCase):
    def test_stage_flow_station(self):
        inlet_flow_station = FlowStation(
            gamma=1.4,          # dimensionless
            Rs=287,             # J/kg/K
            Tt=450,             # K
            Pt=2.80E5,          # Pa
            mdot=2.2,           # kg/s
            Vm=263.95,          # m/s
            alpha=0,            # rad
            N=36000,            # rpm
            radius=0.08,        # m
        )

        np.testing.assert_almost_equal(inlet_flow_station.beta, np.radians(-48.808), 3)
        np.testing.assert_almost_equal(inlet_flow_station.U, 301.593, 3)
        np.testing.assert_almost_equal(inlet_flow_station.V, 263.95, 3)
        np.testing.assert_almost_equal(inlet_flow_station.Vcr, 388.169, 3)
        np.testing.assert_almost_equal(inlet_flow_station.Ttr, 495.275, 3)
        np.testing.assert_almost_equal(inlet_flow_station.Ptr, 391631.736, 3)
        np.testing.assert_almost_equal(inlet_flow_station.Vtheta, 0.00, 3)

        mid_flow_station = FlowStation(
            gamma=1.4,          # dimensionless
            Rs=287,             # J/kg/K
            Tt=504.6,           # K
            Pt=3.60E5,          # Pa
            mdot=2.2,           # kg/s
            Vm=263.95,          # m/s
            alpha=0.593,        # rad
            N=36000,            # rpm
            radius=0.08,        # m
        )

        np.testing.assert_almost_equal(mid_flow_station.Wtheta, -123.715, 3)
        np.testing.assert_almost_equal(mid_flow_station.Ttr, 496.469, 3)
        np.testing.assert_almost_equal(mid_flow_station.Ptr,340102.039, 3)


    def test_velocity_triangle(self):
        inlet_flow_station = FlowStation(
            Vm=150,           # m/s
            N=15000,          # RPM
            radius=0.1696031,  # m
            alpha=0,          # rad
        )

        outlet_flow_station = FlowStation(
            Vm=150,                         # m/s
            N=15000,                        # RPM
            radius=0.1696031,               # m
            alpha=np.radians(26.69005971),  # rad
        )

        # Inlet Flow Station
        np.testing.assert_almost_equal(np.degrees(inlet_flow_station.alpha), 0.0)
        np.testing.assert_almost_equal(np.degrees(inlet_flow_station.beta), -60.61884197)
        np.testing.assert_almost_equal(inlet_flow_station.Vm, 150.0)
        np.testing.assert_almost_equal(inlet_flow_station.Vtheta, 0.0)
        np.testing.assert_almost_equal(inlet_flow_station.V, 150.0)
        np.testing.assert_almost_equal(inlet_flow_station.U, 266.41192649)
        np.testing.assert_almost_equal(inlet_flow_station.Wtheta, -266.41192649)
        np.testing.assert_almost_equal(inlet_flow_station.W, 305.73732938)

        # Outlet Flow Station
        np.testing.assert_almost_equal(np.degrees(outlet_flow_station.alpha), 26.69005971)
        np.testing.assert_almost_equal(np.degrees(outlet_flow_station.beta), -51.85637225)
        np.testing.assert_almost_equal(outlet_flow_station.Vm, 150.0)
        np.testing.assert_almost_equal(outlet_flow_station.Vtheta, 75.40953689)
        np.testing.assert_almost_equal(outlet_flow_station.V, 167.88864838)
        np.testing.assert_almost_equal(outlet_flow_station.Wtheta, -191.0023896)
        np.testing.assert_almost_equal(outlet_flow_station.W, 242.86192133)


if __name__ == '__main__':
    unittest.main()
