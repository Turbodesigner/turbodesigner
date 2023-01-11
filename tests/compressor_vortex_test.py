import unittest
import numpy as np
from turbodesigner.flow_station import FlowStation
from turbodesigner.blade.vortex.free_vortex import FreeVortex


class VelocityTriangleTest(unittest.TestCase):
    def test_free_vortex_angles(self):
        N = 6000                 # RPM
        rm = 0.475               # m
        Um = (1/30)*np.pi*N*rm   # m/s
        cx = 136                 # m/s
        Rm = 1-0.5*(0.5/0.475)**2
        ctheta1_m = 78.6 * (0.5/.475)
        ctheta2_m = 235.6 * (0.5/.475)
        psi_m = (ctheta2_m - ctheta1_m) / Um
        vortex = FreeVortex(
            Um=Um,               # m/s
            Vm=cx,               # m/s
            Rm=Rm,               # dimensionless
            psi_m=psi_m,         # dimensionless
            rm=rm                # m
        )

        ctheta1_hub = vortex.ctheta(r=0.45, is_rotating=True)
        ctheta2_hub = vortex.ctheta(r=0.45, is_rotating=False)
        alpha1_hub = vortex.alpha(r=0.45, is_rotating=True)
        alpha2_hub = vortex.alpha(r=0.45, is_rotating=False)

        ctheta1_tip = vortex.ctheta(r=0.5, is_rotating=True)
        ctheta2_tip = vortex.ctheta(r=0.5, is_rotating=False)
        alpha1_tip = vortex.alpha(r=0.5, is_rotating=True)
        alpha2_tip = vortex.alpha(r=0.5, is_rotating=False)

        # Hub
        np.testing.assert_almost_equal(ctheta1_hub, 87.31, 2)
        np.testing.assert_almost_equal(ctheta2_hub, 261.755, 3)
        np.testing.assert_almost_equal(np.degrees(alpha1_hub), 32.70, 2)
        np.testing.assert_almost_equal(np.degrees(alpha2_hub), 62.54, 2)

        # Tip
        np.testing.assert_almost_equal(ctheta1_tip, 78.57, 2)
        np.testing.assert_almost_equal(ctheta2_tip, 235.58, 2)
        np.testing.assert_almost_equal(np.degrees(alpha1_tip), 30.02, 2)
        np.testing.assert_almost_equal(np.degrees(alpha2_tip), 60.00, 2)


if __name__ == '__main__':
    unittest.main()
