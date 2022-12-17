from enum import Enum
from functools import cached_property
from dataclasses import dataclass
from typing import Optional
from turbodesigner.airfoils import AirfoilType, DCAAirfoil
from turbodesigner.blade.deviation.johnson import JohnsonBladeDeviation
from turbodesigner.blade.metal_angles import MetalAngles
from turbodesigner.blade.vortex.common import Vortex
from turbodesigner.flow_station import FlowStation
from turbodesigner.attachments.firetree import FirtreeAttachment
import numpy as np
import numpy.typing as npt
from turbodesigner.units import MM

@dataclass
class BladeRowExport:
    stage_number: int
    "stage number"

    disk_height: float
    "disk height (length)"

    hub_radius: float
    "blade hub radius (length)"

    tip_radius: float
    "blade hub radius (length)"

    radii: npt.NDArray[np.float64]
    "blade station radius (length)"

    airfoils: np.ndarray
    "airfoil coordinates for each blade radius (length)"

    attachment: np.ndarray
    "attachment coordinates (length)"

    attachment_height: float
    "attachment height (length)"

    number_of_blades: int
    "number of blades"

    twist_angle: int
    "twist angle of blade"

    is_rotating: bool
    "whether blade is rotating or not"


@dataclass
class BladeRow:
    "calculates turbomachinery blade row"

    stage_number: int
    "stage number of blade row"

    stage_flow_station: FlowStation
    "blade stage flow station (FlowStation)"

    vortex: Vortex
    "blade vortex calculation for stagger angles"

    AR: float
    "aspect ratio (dimensionless)"

    sc: float
    "spacing to chord ratio (dimensionless)"

    tbc: float
    "max thickness to chord (dimensionless)"

    is_rotating: bool
    "whether blade is rotating or not"

    N_stream: int
    "number of streams per blade (dimensionless)"

    next_flow_station: Optional["FlowStation"] = None
    "next blade row flow station"

    deviation_iterations: int = 20
    "nominal deviation iterations"

    def __post_init__(self):
        if self.is_rotating and self.next_flow_station is None:
            self.next_flow_station = self.stage_flow_station.copyStream(
                alpha=self.vortex.alpha(self.radii, is_rotating=False),
                radius=self.radii
            )

    @cached_property
    def rt(self):
        "blade tip radius (m)"
        rt = self.stage_flow_station.outer_radius
        assert isinstance(rt, float)
        return rt

    @cached_property
    def rh(self):
        "blade hub radius (m)"
        rh = self.stage_flow_station.inner_radius
        assert isinstance(rh, float)
        return rh

    @cached_property
    def rm(self):
        "blade mean radius (m)"
        rm = self.stage_flow_station.radius
        assert isinstance(rm, float)
        return rm

    @cached_property
    def h(self):
        "height of blade (m)"
        return self.rt-self.rh

    @cached_property
    def h_disk(self):
        "disk height of blade row (m)"
        xi = self.metal_angles.xi[0] if self.is_rotating else self.metal_angles.xi[-1]
        return np.abs(self.c*np.cos(xi)* 1.25)

    @cached_property
    def c(self):
        "chord length (m)"
        return self.h/self.AR

    @cached_property
    def tb(self):
        "blade max thickness (m)"
        return self.tbc * self.c

    @cached_property
    def Z(self):
        "number of blades in row (dimensionless)"
        return int(np.ceil(2*np.pi*self.rm/(self.sc*self.c)))

    @cached_property
    def s(self):
        "spacing between blades (m)"
        return 2*np.pi*self.rh/self.Z

    @cached_property
    def sigma(self):
        "spacing between blades (m)"
        return 1 / self.sc

    @cached_property
    def airfoil_type(self):
        # if self.stage_flow_station.MN < 0.7:
        #     return AirfoilType.NACA65
        # elif self.stage_flow_station.MN >= 0.7 and self.stage_flow_station.MN <= 1.20:
        #     return AirfoilType.DCA
        # raise ValueError("MN > 1.20 not currently supported")
        
        # TODO: only have support of DCA airfoil generation at the moment
        return AirfoilType.DCA


    @cached_property
    def deviation(self):
        return JohnsonBladeDeviation(self.beta1, self.beta2, self.sigma, self.tbc, self.airfoil_type)

    @cached_property
    def metal_angles(self):
        # metal_angles = self.deviation.get_metal_angles(self.deviation_iterations)
        # return metal_angles
        return MetalAngles(self.beta1, self.beta2, 0, 0)

    @cached_property
    def radii(self):
        "blade radii (m)"
        return np.linspace(self.rh, self.rt, self.N_stream, endpoint=True)

    @cached_property
    def flow_station(self):
        "flow station (FlowStation)"
        return self.stage_flow_station.copyStream(
            alpha=self.vortex.alpha(self.radii, self.is_rotating),
            radius=self.radii
        )

    @cached_property
    def beta1(self):
        "blade inlet flow angle (rad)"
        if self.is_rotating:
            return self.flow_station.beta   # beta1
        return self.flow_station.alpha      # alpha2
    
    @cached_property
    def beta2(self):
        "blade outlet flow angle (rad)"
        if self.is_rotating:
            assert self.next_flow_station is not None
            return self.next_flow_station.beta                      # beta2
        
        assert self.next_flow_station is not None or self.vortex.Rm == 0.5, "next_flow_station needs to be defined or Rc=0.5"
        if self.next_flow_station is not None:
            return self.next_flow_station.alpha                     # alpha3
        return self.vortex.alpha(self.radii, is_rotating=False)     # alpha3

    @cached_property
    def airfoils(self):
        r0 = self.tb * 0.15
        # TODO: optimize this with Numba
        return [
            DCAAirfoil(self.c, self.metal_angles.theta[i], r0, self.tb, self.metal_angles.xi[i])
            for i in range(self.N_stream)
        ]

    @cached_property
    def attachment(self):
        # TODO: use faster way to get airfoil width
        hub_airfoil = self.airfoils[0].get_coords()
        # max_length = (np.max(hub_airfoil[:, 1]) - np.min(hub_airfoil[:, 1]))  # m

        return FirtreeAttachment(
            gamma=np.radians(40),
            beta=np.radians(40),
            ll=0.1*self.s,
            lu=0.2*self.s,
            Ri=0.05*self.s,
            Ro=0.025*self.s,
            R_dove=0.05*self.s,
            max_length=0.75*self.s,
            num_stages=0,
            disk_radius=self.rh
        )



    def to_export(self):
        attachment = self.attachment.get_coords()
        attachment_height = np.max(attachment[:, 1]) - np.min(attachment[:, 1])
        return BladeRowExport(
            stage_number=self.stage_number,
            disk_height=self.h_disk * MM,
            hub_radius=self.rh * MM,
            tip_radius=self.rt * MM,
            radii=self.radii * MM,
            airfoils=np.array([airfoil.get_coords() for airfoil in self.airfoils]) * MM,
            attachment=attachment * MM,
            attachment_height=attachment_height * MM,
            number_of_blades=self.Z,
            twist_angle=np.degrees(self.metal_angles.xi[-1]-self.metal_angles.xi[0]),
            is_rotating=self.is_rotating,
        )