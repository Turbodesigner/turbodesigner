from functools import cached_property
from dataclasses import dataclass
from typing import Optional
import numpy as np
from turbodesigner.blade.row import BladeRow, BladeRowExport
from turbodesigner.blade.vortex.free_vortex import FreeVortex
from turbodesigner.flow_station import FlowStation


@dataclass
class StageBladeProperty:
    rotor: float
    stator: float

@dataclass
class StageExport:
    rotor: BladeRowExport
    "rotor blade row"

    stator: BladeRowExport
    "stator blade row"

    stage_height: float
    "stage height"

    stage_number: int
    "stage number"

@dataclass
class Stage:
    "calculates turbomachinery stage"

    stage_number: int
    "stage number"

    Delta_T0: float
    "stage stagnation temperature change between inlet and outlet (K)"

    R: float
    "stage reaction (dimensionless)"

    inlet_flow_station: FlowStation
    "inlet flow station (FlowStation)"

    eta_poly: float
    "polytropic efficiency (dimensionless)"

    N_stream: int
    "number of streams per blade (dimensionless)"

    AR: StageBladeProperty
    "aspect ratio (dimensionless)"

    sc: StageBladeProperty
    "spacing to chord ratio (dimensionless)"

    tbc: StageBladeProperty
    "max thickness to chord (dimensionless)"

    next_stage: Optional["Stage"] = None
    "next turbomachinery stage"

    def __post_init__(self):
        assert isinstance(self.inlet_flow_station.radius, float)
        self.rm = self.inlet_flow_station.radius
        self.N = self.inlet_flow_station.N

    @cached_property
    def Delta_h(self) -> float:
        "enthalpy change between inlet and outlet (J/kg)"
        return self.Delta_T0*self.inlet_flow_station.Cp

    @cached_property
    def U(self):
        "mean blade velocity (m/s)"
        return (1/30)*np.pi*self.N*self.rm

    @cached_property
    def phi(self):
        "flow coefficient (dimensionless)"
        return self.inlet_flow_station.Vm/self.U

    @cached_property
    def psi(self):
        "loading coefficient (dimensionless)"
        return self.Delta_h/self.U**2

    @cached_property
    def alpha1(self):
        "absolute inlet flow angle (rad)"
        return np.arctan((1 - self.R + -(1/2)*self.psi)/self.phi)

    @cached_property
    def alpha2(self):
        "absolute outlet flow angle (rad)"
        return np.arctan((1 - self.R + (1/2)*self.psi)/self.phi)

    @cached_property
    def T02(self):
        "outlet stagnation temperature (K)"
        return self.inlet_flow_station.T0 + self.Delta_T0

    @cached_property
    def outlet_flow_station(self):
        "outlet flow station (FlowStation)"
        P02 = self.inlet_flow_station.P0*self.PR
        return self.inlet_flow_station.copyFlow(
            T0=self.T02,
            P0=P02,
            alpha=self.alpha2,
        )

    @cached_property
    def PR(self):
        "pressure ratio (dimensionless)"
        gamma = self.inlet_flow_station.gamma
        return self.TR**(self.eta_poly*gamma/(gamma - 1))

    @cached_property
    def TR(self):
        "stagnation temperature ratio between stage outlet and inlet (dimensionless)"
        return self.T02/self.inlet_flow_station.T0

    @cached_property
    def vortex(self):
        return FreeVortex(
            Um=self.U, 
            Vm=self.inlet_flow_station.Vm, 
            Rm=self.R, 
            phi_m=self.phi, 
            psi_m=self.psi, 
            rm=self.rm
        )

    @cached_property
    def rotor(self):
        return BladeRow(
            stage_number=self.stage_number,
            stage_flow_station=self.inlet_flow_station,
            vortex=self.vortex,
            AR=self.AR.rotor,
            sc=self.sc.rotor,
            tbc=self.tbc.rotor,
            is_rotating=True,
            N_stream=self.N_stream,
            next_flow_station=self.stator.flow_station
        )

    @cached_property
    def stator(self):
        return BladeRow(
            stage_number=self.stage_number,
            stage_flow_station=self.outlet_flow_station,
            vortex=self.vortex,
            AR=self.AR.stator,
            sc=self.sc.stator,
            tbc=self.tbc.stator,
            is_rotating=False,
            N_stream=self.N_stream,
            next_flow_station=None if self.next_stage is None else self.next_stage.rotor.flow_station
        )

    def to_export(self):
        rotor=self.rotor.to_export()
        stator=self.stator.to_export()
        stage_height = rotor.disk_height+stator.disk_height
        return StageExport(
            stage_number=self.stage_number,
            rotor=rotor,
            stator=stator,
            stage_height=stage_height
        )