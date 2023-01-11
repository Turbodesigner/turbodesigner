from functools import cached_property
from dataclasses import dataclass
from typing import Optional, Union
import numpy as np

PROP_NON_STREAM_ERROR = "Property not allowed with streams"


class FluidConstants:
    MU_REF = 1.73E-5
    "reference dynamic viscocity at sea level ((N*s)/m**2)"

    P0_REF = 101325
    "reference pressure at sea level (Pa)"

    T0_REF = 288.15
    "reference temperature at sea level (K)"

    C = 110.4
    "sutherland constant (K)"


@dataclass
class FlowStation:
    "calculates flow station for an ideal gas"

    gamma: float = np.nan
    "ratio of specific heats (dimensionless)"

    Rs: float = np.nan
    "specific gas constant (J/(kg*K))"

    Tt: float = np.nan
    "total temperature (K)"

    Pt: float = np.nan
    "total pressure (Pa)"

    Vm: float = np.nan
    "meridional flow velocity (m/s)"

    mdot: float = np.nan
    "mass flow rate (kg/s)"

    B: float = 0.0
    "blockage factor (dimensionless)"

    alpha: Union[float, np.ndarray] = np.nan
    "absolute flow angle (rad)"

    N: float = np.nan
    "rotational speed (rpm)"

    radius: Union[float, np.ndarray] = np.nan
    "flow radius (m)"

    mixture: str = "Air"
    "mixture"

    is_stream: bool = False
    "whether station is 1D stream"

    def copyFlow(
        self,
        T0: Optional[float] = None,
        P0: Optional[float] = None,
        Vm: Optional[float] = None,
        mdot: Optional[float] = None,
        alpha: Optional[Union[float, np.ndarray]] = None,
        radius: Optional[Union[float, np.ndarray]] = None,
    ):
        "copies all elements of FlowStation (FlowStation)"
        return FlowStation(
            gamma=self.gamma,
            Rs=self.Rs,
            Tt=self.Tt if T0 is None else T0,
            Pt=self.Pt if P0 is None else P0,
            Vm=self.Vm if Vm is None else Vm,
            mdot=self.mdot if mdot is None else mdot,
            B=self.B,
            alpha=self.alpha if alpha is None else alpha,
            N=self.N,
            radius=self.radius if radius is None else radius,
            mixture=self.mixture
        )

    def copyStream(
        self,
        alpha: Optional[Union[float, np.ndarray]] = None,
        radius: Optional[Union[float, np.ndarray]] = None,
    ):
        """copies stream elements of FlowStation (FlowStation)

           excludes:
                * mdot - mass flow rate

        """
        return FlowStation(
            gamma=self.gamma,
            Rs=self.Rs,
            Tt=self.Tt,
            Pt=self.Pt,
            Vm=self.Vm,
            alpha=self.alpha if alpha is None else alpha,
            N=self.N,
            radius=self.radius if radius is None else radius,
            is_stream=True,
            mixture=self.mixture
        )

    @cached_property
    def h(self):
        "static enthalpy (J/kg*K)"
        return self.T*self.Cp

    @cached_property
    def ht(self):
        "total enthalpy (J/kg*K)"
        return self.h + (self.V**2)/2

    @cached_property
    def Cp(self):
        "specific heat at constant pressure (J/(kg*K))"
        return self.Rs*self.gamma/(self.gamma - 1)

    @cached_property
    def T(self):
        "static temperature (K)"
        return self.Tt - (self.V**2)/(2*self.Cp)

    @cached_property
    def Ttr(self):
        "total realtive temperature (K)"
        return self.Tt + (self.W**2 - self.V**2)/(2*self.Cp)

    @cached_property
    def P(self):
        "static pressure (Pa)"
        return self.Pt*(self.T/self.Tt)**(self.gamma/(self.gamma - 1))

    @cached_property
    def Ptr(self):
        "total relative pressure (Pa)"
        return self.Pt*(self.Ttr/self.Tt)**(self.gamma/(self.gamma - 1))

    @cached_property
    def rho(self):
        "density (kg/m**3)"
        return self.P/(self.T*self.Rs)

    @cached_property
    def q(self):
        "dynamic pressure (Pa)"
        return 0.5*self.rho*self.Vm**2

    @cached_property
    def a(self):
        "speed of sound in medium (m/s)"
        return np.sqrt(self.T*self.Rs*self.gamma)

    @cached_property
    def mu(self):
        "dynamic velocity using Sutherland's formula ((N*s)/m**2)"
        return FluidConstants.MU_REF * ((self.T / FluidConstants.T0_REF)**1.5) * ((FluidConstants.T0_REF + FluidConstants.C) / (self.T + FluidConstants.C))

    @cached_property
    def MN(self):
        "mach number (dimensionless)"
        return self.Vm/self.a

    @cached_property
    def Vcr(self):
        "critical velocity (m/s)"
        return np.sqrt(((2*self.gamma)/(self.gamma+1)) * self.Rs*self.Tt)

    @cached_property
    def U(self):
        "blade velocity (m/s)"
        return FlowStation.calc_U(self.N, self.radius)

    @cached_property
    def omega(self):
        "blade angular velocity (rad/s)"
        return self.U/self.radius

    @cached_property
    def Vtheta(self):
        "absolute tangential velocity (m/s)"
        return self.Vm*np.tan(self.alpha)

    @cached_property
    def V(self):
        "absolute flow velocity (m/s)"
        if np.isnan(self.alpha).all():
            return self.Vm
        return self.Vm/np.cos(self.alpha)

    @cached_property
    def Wtheta(self):
        "relative tangential flow velocity (m/s)"
        return self.Vtheta - self.U

    @cached_property
    def beta(self):
        "relative flow angle (rad)"
        return np.arctan(self.Wtheta/self.Vm)

    @cached_property
    def W(self):
        "relative flow velocity (m/s)"
        return self.Vm/np.cos(self.beta)

    # %% Annular Properties
    @cached_property
    def A_flow(self):
        "cross-sectional flow area (m**2)"
        assert not self.is_stream, PROP_NON_STREAM_ERROR
        return self.mdot/(self.rho*self.Vm)

    @cached_property
    def A_phys(self):
        "physical cross sectional area (m**2)"
        return self.A_flow*(self.B + 1)

    @cached_property
    def outer_radius(self):
        "flow outer radius (m)"
        assert not self.is_stream, PROP_NON_STREAM_ERROR
        return self.A_phys/(4*np.pi*self.radius) + self.radius

    @cached_property
    def inner_radius(self):
        "flow inner radius (m)"
        assert not self.is_stream, PROP_NON_STREAM_ERROR
        return 2*self.radius - self.outer_radius

    @staticmethod
    def calc_radius_from_ht(ht: float, A_phys: Union[float, np.ndarray]):
        """calculates radius from hub to tip ratio

        Parameters
        ==========

        ht: float
            hub to tip ratio (dimensionless)

        A_phys: float
            physical cross sectional area (m**2)

        """

        outer_radius = np.sqrt(A_phys / (np.pi*(1-ht**2)))
        inner_radius = ht * outer_radius
        return (outer_radius + inner_radius) / 2

    @staticmethod
    def calc_U(N: float, radius: Union[float, np.ndarray]):
        """calculates blade velocity

        Parameters
        ==========

        N: float
            rotational speed (rpm)

        radius: float
            blade radius (m)

        """

        return 2*np.pi*N*radius/60

    def set_radius(self, ht: float):
        """sets radius from hub to tip ratio

        Parameters
        ==========

        ht: float
            hub to tip ratio (dimensionless)
        """
        self.radius = FlowStation.calc_radius_from_ht(ht, self.A_phys)
