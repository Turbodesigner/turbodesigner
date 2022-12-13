from functools import cached_property
from dataclasses import dataclass
import json
import numpy as np
from turbodesigner.flow_station import FlowStation
from turbodesigner.stage import Stage, StageBladeProperty, StageExport
from dacite.core import from_dict

@dataclass
class TurbomachineryExport:
    stages: list[StageExport]
    "turbomachinery stages"

@dataclass
class Turbomachinery:
    gamma: float
    "ratio of specific heats (dimensionless)"

    cx: float
    "inlet flow velocity in the radial and axial plane (m/s)"

    N: float
    "rotational speed (rpm)"

    Rs: float
    "specific gas constant (J/(kg*K))"

    mdot: float
    "mass flow rate (kg/s)"

    PR: float
    "pressure ratio (dimensionless)"

    P01: float
    "inlet stagnation pressure (Pa)"

    T01: float
    "inlet stagnation temperature (K)"

    eta_isen: float
    "isentropic efficiency (dimensionless)"

    N_stg: int
    "number of stages (dimensionless)"

    Delta_T0_stg: list[float]
    "array of stage stagnation temperature change between inlet and outlet (K)"

    R_stg: list[float]
    "array of stage reaction rates (dimensionless)"

    B_in: float
    "inlet blockage factor (dimensionless)"

    B_out: float
    "outlet blockage factor (dimensionless)"

    ht: float
    "hub to tip ratio(dimensionless)"

    N_stream: int
    "number of streams per blade (dimensionless)"

    AR: list[StageBladeProperty]
    "aspect ratio (dimensionless)"

    sc: list[StageBladeProperty]
    "spacing to chord ratio (dimensionless)"

    tbc: list[StageBladeProperty]
    "max thickness to chord (dimensionless)"

    @cached_property
    def T02(self):
        "outlet stagnation temperature (K)"
        return self.T01*self.PR**((self.gamma - 1)/(self.eta_poly*self.gamma))

    @cached_property
    def P02(self):
        "stagnation outlet pressure (Pa)"
        return self.P01*self.PR

    @cached_property
    def eta_poly(self):
        "polytropic efficiency (dimensionless)"
        return (self.gamma - 1)*np.log(self.PR)/(self.gamma*np.log((self.eta_isen + self.PR**((self.gamma - 1)/self.gamma) - 1)/self.eta_isen))

    @cached_property
    def inlet_flow_station(self):
        "inlet flow station (FlowStation)"
        flow_station = FlowStation(
            gamma=self.gamma,
            Rs=self.Rs,
            T0=self.T01,
            P0=self.P01,
            Vm=self.cx,
            mdot=self.mdot,
            B=self.B_in,
            N=self.N,
        )
        flow_station.set_radius(self.ht)
        return flow_station

    @cached_property
    def outlet_flow_station(self):
        "outlet flow station (FlowStation)"
        return FlowStation(
            gamma=self.gamma,
            Rs=self.Rs,
            T0=self.T02,
            P0=self.P02,
            Vm=self.cx,
            mdot=self.mdot,
            B=self.B_out,
            N=self.N,
            radius=self.inlet_flow_station.radius
        )

    @cached_property
    def Delta_T0(self):
        "stagnation temperature change between outlet and inlet (dimensionless)"
        return self.outlet_flow_station.T0 - self.inlet_flow_station.T0


    @cached_property
    def TR(self):
        "stagnation temperature ratio between outlet and inlet (dimensionless)"
        return self.outlet_flow_station.T0/self.inlet_flow_station.T0

    @cached_property
    def stages(self):
        "turbomachinery stages (list[Stage])"
        assert len(self.Delta_T0_stg) == self.N_stg, "Delta_T0 quantity does not equal N_stg"
        assert len(self.R_stg) == self.N_stg, "R quantity does not equal N_stg"
        assert self.R_stg[self.N_stg-1] == 0.5, "Last stage reaction only supports R=0.5"
        assert len(self.AR) == self.N_stg, "AR quantity does not equal N_stg"
        assert len(self.sc) == self.N_stg, "sc quantity does not equal N_stg"
        assert len(self.tbc) == self.N_stg, "tbc quantity does not equal N_stg"

        inlet_flow_station = self.inlet_flow_station
        stages: list[Stage] = []
        # TODO: make this more efficient with Numba
        for i in range(self.N_stg):
            stage = Stage(
               stage_number=i+1,
               Delta_T0=self.Delta_T0_stg[i],
               R=self.R_stg[i],
               inlet_flow_station=inlet_flow_station,
               eta_poly=self.eta_poly,
               N_stream=self.N_stream,
               AR=self.AR[i],
               sc=self.sc[i],
               tbc=self.tbc[i]
            )
            inlet_flow_station = stage.outlet_flow_station
            if i > 0 and i < self.N_stg:
                stages[i-1].next_stage = stage 
            stages.append(stage)
        
        return stages

    def to_export(self):
        return TurbomachineryExport(
            stages = [
                stage.to_export() for stage in self.stages
            ]
        )
    
    @staticmethod
    def from_dict(obj) -> "Turbomachinery":
        return from_dict(data_class=Turbomachinery, data=obj)


    @staticmethod
    def from_file(file_name: str) -> "Turbomachinery":
        with open(file_name, "r") as fp:
            obj = json.load(fp)
        return from_dict(data_class=Turbomachinery, data=obj)