from dataclasses import dataclass
from functools import cached_property
import pickle
from typing import Tuple
import numpy as np
from pymoo.optimize import minimize
from pymoo.core.problem import ElementwiseProblem
from pymoo.algorithms.moo.nsga2 import NSGA2
from pymoo.factory import get_sampling, get_crossover, get_mutation
from pymoo.factory import get_termination
from turbodesigner.turbomachinery import Turbomachinery

@dataclass
class TurbomachineryConstraints:
    psi: float
    "stage loading coefficient (dimensionless)"

    R: float
    "stage reaction coefficient (dimensionless)"

    phi: float
    "stage flow coefficient (dimensionless)"

    @cached_property
    def constraint_beta_lte_70(self):
        "limit to keep absolute value of flow angles less than or equal to 70 deg (dimensionless)"
        return [(1/2)*self.psi + self.R - 2.75*self.phi, (1/2)*self.psi - self.R - 2.75*self.phi + 1]

    @cached_property
    def constraint_psi_lte_psi_lim_and_1(self):
        "limit to keep psi less than or equal to psi_lim and 1(dimensionless)"
        return [self.psi - self.psi_lim, self.psi - 1.0]



    @cached_property
    def psi_lim(self):
        "stage loading coefficient limit curve (dimensionless)"
        R_hat = abs(R - 0.5) + 0.5
        return (6/17)*R_hat + 0.375148773423528*phi**(2 + 0.1/R_hat)*(R_hat**(-1.0))**1.18




@dataclass
class TurbomachineryOptimizationProblem(ElementwiseProblem):
    cx: Tuple[float, float]
    "inlet flow velocity in the radial and axial plane (m/s)"

    N: Tuple[float, float]
    "rotational speed (rpm)"

    mdot: Tuple[float, float]
    "mass flow rate (kg/s)"

    PR: Tuple[float, float]
    "pressure ratio (dimensionless)"

    eta_isen: Tuple[float, float]
    "isentropic efficiency (dimensionless)"

    N_stg: Tuple[int, int]
    "number of stages (dimensionless)"

    ht: Tuple[float, float]
    "hub to tip ratio (dimensionless)"

    gamma: float = 1.4
    "ratio of specific heats (dimensionless)"

    Rs: float = 287
    "specific gas constant (J/(kg*K))"

    P01: float = 101000
    "ambient pressure (Pa)"

    T01: float = 288
    "ambient temperature (K)"


    # B_in: Union[Number, list[Number]]
    # "inlet blockage factor (dimensionless)"

    # B_out: Union[Number, list[Number]]
    # "outlet blockage factor (dimensionless)"

    def __post_init__(self):
        bounds = [
            self.cx,
            self.N,
            self.mdot,
            self.PR,
            self.eta_isen,
            self.N_stg,
            self.ht
        ]

        super().__init__(
            n_var=len(bounds),
            n_obj=1,
            n_constr=29,
            xl=[bound[0] for bound in bounds],
            xu=[bound[1] for bound in bounds]
        )



    def _evaluate(self, x, out, *args, **kwargs):
        cx, N, mdot, PR, eta_isen, N_stg, ht = x
        turbomachinery = Turbomachinery(
            gamma=self.gamma,
            cx=cx,
            N=N,
            Rs=self.Rs,
            mdot=mdot,
            PR=PR,
            P01=self.P01,
            T01=self.T01,
            eta_isen=eta_isen,
            N_stg=N_stg,
            B_in=0,
            B_out=0,
            ht=ht,
            N_stream=None,
            Delta_T0_stg=None,
            R_stg=None,
            rgc=None,
            sgc=None,
            AR=None,
            sc=None,
            tbc=None
        )


        # constraints = []


        # # Compressor Size Constraints
        # casing_cad_specification = CompressorCadService.get_casing_cad_specification()
        # casing_cad_specification.initialize(StageExport.from_stage(compressor.stages[0]))
        # constraints += calc_compressor_size_constraints(
        #     d_casing_max=self.spec.max_diameter,
        #     d_casing=casing_cad_specification.radius * pq.mm
        # ).to_dict().values()
        # constraint_d_casing_lt_d_casing_max = -d_casing_max + d_casing



    return CompressorStageConstraints(constraint_beta_lte_70_1, constraint_beta_lte_70_2, constraint_psi_lte_psi_lim, constraint_psi_lim_lte_1)



        #     compressor_stage_constraints = calc_compressor_stage_constraints(
        #         R=stage.info.velocity_triangle.reaction,
        #         psi=stage.info.velocity_triangle.loading_coefficient,
        #         phi=stage.info.velocity_triangle.flow_coefficient
        #     )
        #     constraints += list(compressor_stage_constraints.to_dict().values())
        
        
        
        out["F"] = np.column_stack([-design_specification.pressure_ratio]).astype(np.float)
        out["G"] = np.column_stack(constraints).astype(np.float)


class CompressorOptimizationService:
    @staticmethod
    def optimize(spec: CompressorOptimizationSpecification):

        problem = CompressorOptimizationProblem(spec)
        algorithm = NSGA2(
            pop_size=100,
            n_offsprings=10,
            sampling=get_sampling("real_random"),
            crossover=get_crossover("real_sbx", prob=0.9, eta=15),
            mutation=get_mutation("real_pm", eta=20),
            eliminate_duplicates=True
        )

        termination = get_termination("n_gen", 100)

        res = minimize(
            problem,
            algorithm,
            termination,
            seed=1,
            save_history=True,
            verbose=True
        )
        with open('optimization.pkl', 'wb') as optimization_result_file:
            pickle.dump(res, optimization_result_file, pickle.HIGHEST_PROTOCOL)


# Objectives
# lower number of stages
# maximize efficiency

# Constraints

# abs(flow angles) < 70 deg
# deHallers criterion 0.72 <= (w2/w1) <= 1
# psi < 1
# phi <= 1

# Mtip = 1.4-1.8 for a single stage fan,
# Mtip = 0.9-1.3 for a multi-stage compressor

# M = 0.5-0.65 for a single-stage fan,
# M = 0.25-0.35 for multi-stage compressor

# %%