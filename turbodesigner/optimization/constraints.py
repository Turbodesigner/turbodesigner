from quantities import Quantity
import numpy as np
from jetengine.utils.units import validate_units


class CompressorBladeStationConstraints:

    """

    Parameters
    ==========

    constraint_ht_up_gt_ht: Quantity
        upstream hub-to-tip greater than downstream hub-to-tip (dimensionless)

    """

    def __init__(self,constraint_ht_up_gt_ht: Quantity):
        self.constraint_ht_up_gt_ht = constraint_ht_up_gt_ht

    def to_dict(self) -> dict:
        return {
            "constraint_ht_up_gt_ht": self.constraint_ht_up_gt_ht,
        }

    def __str__(self) -> str:
        dict_from_class = self.to_dict()
        return str(dict_from_class)

@validate_units(ht_up="dimensionless", Z_up="dimensionless", ht="dimensionless", Z="dimensionless")
def calc_blade_station_constraints(ht_up: Quantity, Z_up: Quantity, ht: Quantity, Z: Quantity):

    """calculates compressor blade station constraints

    Parameters
    ==========

    ht_up: Quantity
        upstream hub to tip radius ratio (dimensionless)

    Z_up: Quantity
        upstream number of blades in row (dimensionless)

    ht: Quantity
        hub to tip radius ratio (dimensionless)

    Z: Quantity
        number of blades in row (dimensionless)

    """

    # upstream hub-to-tip greater than downstream hub-to-tip (dimensionless)
    constraint_ht_up_gt_ht = ht - ht_up

    return CompressorBladeStationConstraints(constraint_ht_up_gt_ht)


from quantities import Quantity
import numpy as np
from jetengine.utils.units import validate_units


@validate_units(R="dimensionless", phi="dimensionless", returns="dimensionless")
def calc_limited_loading_coefficient(R: Quantity, phi: Quantity, units = "dimensionless"):

    """calculates stage loading coefficient limit curve

    Parameters
    ==========

    R: Quantity
        reaction rate (dimensionless)

    phi: Quantity
        flow coefficient (dimensionless)

    """

    #  (dimensionless)
    R_hat = abs(R - 0.5) + 0.5

    # stage loading coefficient limit curve (dimensionless)
    psi_lim = (6/17)*R_hat + 0.375148773423528*phi**(2 + 0.1/R_hat)*(R_hat**(-1.0))**1.18

    return psi_lim

class CompressorStageConstraints:

    """

    Parameters
    ==========

    constraint_beta_lte_70_1: Quantity
        limit to keep absolute value of flow angles less than or equal to 70 deg (dimensionless)

    constraint_beta_lte_70_2: Quantity
        limit to keep absolute value of flow angles less than or equal to 70 deg (dimensionless)

    constraint_psi_lte_psi_lim: Quantity
        limit to keep psi less than or equal to $\psi_{lim}$ (dimensionless)

    constraint_psi_lim_lte_1: Quantity
        limit to keep psi less than or equal to 1 (dimensionless)

    """

    def __init__(self,constraint_beta_lte_70_1: Quantity, constraint_beta_lte_70_2: Quantity, constraint_psi_lte_psi_lim: Quantity, constraint_psi_lim_lte_1: Quantity):
        self.constraint_beta_lte_70_1 = constraint_beta_lte_70_1
        self.constraint_beta_lte_70_2 = constraint_beta_lte_70_2
        self.constraint_psi_lte_psi_lim = constraint_psi_lte_psi_lim
        self.constraint_psi_lim_lte_1 = constraint_psi_lim_lte_1

    def to_dict(self) -> dict:
        return {
            "constraint_beta_lte_70_1": self.constraint_beta_lte_70_1,
            "constraint_beta_lte_70_2": self.constraint_beta_lte_70_2,
            "constraint_psi_lte_psi_lim": self.constraint_psi_lte_psi_lim,
            "constraint_psi_lim_lte_1": self.constraint_psi_lim_lte_1,
        }

    def __str__(self) -> str:
        dict_from_class = self.to_dict()
        return str(dict_from_class)

@validate_units(R="dimensionless", psi="dimensionless", phi="dimensionless")
def calc_compressor_stage_constraints(R: Quantity, psi: Quantity, phi: Quantity):

    """calculates compressor stage optimization constraints

    Parameters
    ==========

    R: Quantity
        reaction rate (dimensionless)

    psi: Quantity
        loading coefficient (dimensionless)

    phi: Quantity
        flow coefficient (dimensionless)

    """

    # limit to keep absolute value of flow angles less than or equal to 70 deg (dimensionless)
    constraint_beta_lte_70_1 = (1/2)*psi + R - 2.75*phi

    # limit to keep absolute value of flow angles less than or equal to 70 deg (dimensionless)
    constraint_beta_lte_70_2 = (1/2)*psi - R - 2.75*phi + 1

    # stage loading coefficient limit curve (dimensionless)
    psi_lim = calc_limited_loading_coefficient(R, phi)

    # limit to keep psi less than or equal to $\psi_{lim}$ (dimensionless)
    constraint_psi_lte_psi_lim = psi - psi_lim

    # limit to keep psi less than or equal to 1 (dimensionless)
    constraint_psi_lim_lte_1 = psi - 1.0

    return CompressorStageConstraints(constraint_beta_lte_70_1, constraint_beta_lte_70_2, constraint_psi_lte_psi_lim, constraint_psi_lim_lte_1)

class CompressorSizeConstraints:

    """

    Parameters
    ==========

    constraint_d_casing_lt_d_casing_max: Quantity
        compressor casing less than design casing max diameter (dimensionless)

    """

    def __init__(self,constraint_d_casing_lt_d_casing_max: Quantity):
        self.constraint_d_casing_lt_d_casing_max = constraint_d_casing_lt_d_casing_max

    def to_dict(self) -> dict:
        return {
            "constraint_d_casing_lt_d_casing_max": self.constraint_d_casing_lt_d_casing_max,
        }

    def __str__(self) -> str:
        dict_from_class = self.to_dict()
        return str(dict_from_class)

@validate_units(d_casing_max="m", d_casing="m")
def calc_compressor_size_constraints(d_casing_max: Quantity, d_casing: Quantity):

    """calculates compressor size constraints

    Parameters
    ==========

    d_casing_max: Quantity
        maximum allowed overall casing diameter (m)

    d_casing: Quantity
        overall casing diameter (m)

    """

    # compressor casing less than design casing max diameter (dimensionless)
    constraint_d_casing_lt_d_casing_max = -d_casing_max + d_casing

    return CompressorSizeConstraints(constraint_d_casing_lt_d_casing_max)


