from functools import cached_property
from dataclasses import dataclass
import json
from typing import Literal, Union
import numpy as np
from turbodesigner.flow_station import FlowStation
from turbodesigner.stage import Stage, StageBladeProperty, StageCadExport
from dacite.core import from_dict
Number = Union[int, float]


@dataclass
class TurbomachineryCadExport:
    stages: list[StageCadExport]
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

    Pt: float
    "ambient total pressure (Pa)"

    Tt: float
    "ambient total temperature (K)"

    eta_isen: float
    "isentropic efficiency (dimensionless)"

    N_stg: int
    "number of stages (dimensionless)"

    B_in: Union[Number, list[Number]]
    "inlet blockage factor (dimensionless)"

    B_out: Union[Number, list[Number]]
    "outlet blockage factor (dimensionless)"

    ht: float
    "hub to tip ratio (dimensionless)"

    N_stream: int
    "number of streams per blade (dimensionless)"

    Delta_T0_stg: Union[list[float], Literal["equal"]]
    "array of stage stagnation temperature rises between inlet and outlet (K)"

    R_stg: Union[Number, list[Number]]
    "array of stage reaction rates (dimensionless)"

    rgc: Union[Number, list[Number]]
    "row gap to chord (dimensionless)"

    sgc: Union[Number, list[Number]]
    "stage gap to chord (dimensionless)"

    AR: Union[StageBladeProperty, list[StageBladeProperty]]
    "aspect ratios (dimensionless)"

    sc: Union[StageBladeProperty, list[StageBladeProperty]]
    "spacing to chord ratios (dimensionless)"

    tbc: Union[StageBladeProperty, list[StageBladeProperty]]
    "max thickness to chords (dimensionless)"

    @cached_property
    def T02(self):
        "outlet stagnation temperature (K)"
        return self.Tt*self.PR**((self.gamma - 1)/(self.eta_poly*self.gamma))

    @cached_property
    def P02(self):
        "stagnation outlet pressure (Pa)"
        return self.Pt*self.PR

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
            Tt=self.Tt,
            Pt=self.Pt,
            Vm=self.cx,
            mdot=self.mdot,
            B=self.B_in[0] if isinstance(self.B_in, list) else self.B_in,
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
            Tt=self.T02,
            Pt=self.P02,
            Vm=self.cx,
            mdot=self.mdot,
            B=self.B_out[-1] if isinstance(self.B_out, list) else self.B_out,
            N=self.N,
            radius=self.inlet_flow_station.radius
        )

    @cached_property
    def Delta_T0(self):
        "stagnation temperature change between outlet and inlet (dimensionless)"
        return self.outlet_flow_station.Tt - self.inlet_flow_station.Tt

    @cached_property
    def TR(self):
        "stagnation temperature ratio between outlet and inlet (dimensionless)"
        return self.outlet_flow_station.Tt/self.inlet_flow_station.Tt

    @cached_property
    def stages(self):
        "turbomachinery stages (list[Stage])"
        if isinstance(self.Delta_T0_stg, list):
            assert len(self.Delta_T0_stg) == self.N_stg, "Delta_T0 quantity does not equal N_stg"
        else:
            assert self.Delta_T0_stg == "equal", f"'{self.Delta_T0_stg}' for  Delta_T0_stg is invalid"

        if isinstance(self.R_stg, list):
            assert len(self.R_stg) == self.N_stg, "R quantity does not equal N_stg"
            assert self.R_stg[self.N_stg-1] == 0.5, "Last stage reaction only supports R=0.5"
        else:
            assert self.R_stg == 0.5, "Last stage reaction only supports R=0.5"

        if isinstance(self.AR, list):
            assert len(self.AR) == self.N_stg, "AR quantity does not equal N_stg"
        if isinstance(self.sc, list):
            assert len(self.sc) == self.N_stg, "sc quantity does not equal N_stg"
        if isinstance(self.tbc, list):
            assert len(self.tbc) == self.N_stg, "tbc quantity does not equal N_stg"

        previous_flow_station = self.inlet_flow_station
        stages: list[Stage] = []
        # TODO: make this more efficient with Numba
        for i in range(self.N_stg):
            stage = Stage(
                stage_number=i+1,
                Delta_Tt=self.Delta_T0_stg[i] if isinstance(self.Delta_T0_stg, list) else self.Delta_T0/self.N_stg,
                R=self.R_stg[i] if isinstance(self.R_stg, list) else self.R_stg,
                previous_flow_station=previous_flow_station,
                eta_poly=self.eta_poly,
                N_stream=self.N_stream,
                AR=self.AR[i] if isinstance(self.AR, list) else self.AR,
                sc=self.sc[i] if isinstance(self.sc, list) else self.sc,
                tbc=self.tbc[i] if isinstance(self.tbc, list) else self.tbc,
                rgc=self.rgc[i] if isinstance(self.rgc, list) else self.rgc,
                sgc=self.sgc[i] if isinstance(self.sgc, list) else self.sgc
            )
            previous_flow_station = stage.mid_flow_station
            if i > 0 and i < self.N_stg:
                stages[i-1].next_stage = stage
            stages.append(stage)

        return stages

    def to_cad_export(self):
        return TurbomachineryCadExport(
            stages=[
                stage.to_cad_export() for stage in self.stages
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
