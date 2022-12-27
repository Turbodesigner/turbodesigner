from functools import cached_property
from typing import Optional
import numpy as np
from dataclasses import dataclass
import plotly.graph_objects as go

def get_line(
    y: np.ndarray,
    point1: np.ndarray,
    point2: np.ndarray,
    y_int: Optional[int] = None
):
    x2, y2 = point2[0], point2[1]
    x1, y1 = point1[0], point1[1]
    m = (y2 - y1) / (x2 - x1)
    b = y2 - m*x2 if y_int is None else 0
    x = (y-b)/m
    return np.array([x, y]).T


def get_arc(
    lower_point: np.ndarray,
    upper_point: np.ndarray,
    radius: float,
    center: np.ndarray,
    num_points: int,
    is_clockwise: bool = True,
    endpoint: bool = True
):
    lower_distance = lower_point - center
    upper_distance = upper_point - center
    angle1 = np.arctan2(lower_distance[1], lower_distance[0])
    angle2 = np.arctan2(upper_distance[1], upper_distance[0])

    if not is_clockwise:
        angle1 = angle1 + 2*np.pi

    angle = np.linspace(angle1, angle2, num_points, endpoint=endpoint)
    return center + np.array([
        radius * np.cos(angle),
        radius * np.sin(angle),
    ]).T


@dataclass
class FirtreeAttachment:
    gamma: float
    "angle of upper flank line (rad)"

    beta: float
    "angle of lower flank line (rad)"

    ll: float
    "lower flank line length (m)"

    lu: float
    "upper flank line length (m)"

    Ri: float
    "inner circle radius"

    Ro: float
    "outer circle radius"

    R_dove: float
    "dove circle radius"

    max_length: float
    "max length of attachment"

    num_stages: int
    "number of stages"

    disk_radius: float
    "disk radius of attachment"

    tolerance: float
    "attachment side tolerance"

    num_arc_points: int = 20
    "number of arc points"

    def __post_init__(self):
        self.origin = np.array([0, 0])

        # Outer Circle
        self.outer_circle_lower_tangent = self.origin + (
            np.array([self.ll*np.cos(self.beta), self.ll*np.sin(self.beta)])
        )
        self.outer_circle_upper_tangent = self.outer_circle_lower_tangent + (
            np.array([0, 2*self.Ro*np.cos(self.gamma)])
        )
        self.outer_circle_tanget_intersect = self.outer_circle_lower_tangent + (
            np.array([self.Ro*(1 - np.sin(self.gamma)**2)/np.sin(self.gamma), self.Ro*np.cos(self.gamma),])
        )
        self.outer_circle_center = self.outer_circle_lower_tangent + (
            np.array([-self.Ro*np.sin(self.gamma), self.Ro*np.cos(self.gamma)])
        )

        # Inner Circle
        self.inner_circle_lower_tangent = self.outer_circle_upper_tangent + (
            np.array([-self.lu*np.cos(self.gamma), self.lu*np.sin(self.gamma)])
        )
        self.inner_circle_upper_tangent = self.inner_circle_lower_tangent + (
            np.array([0, 2*self.Ri*np.cos(self.gamma)])
        )
        self.inner_circle_center = self.inner_circle_lower_tangent + (
            np.array([self.Ri*np.sin(self.beta), self.Ri*np.cos(self.beta)])
        )

        # Dove
        self.dove_circle_center = np.array([self.R_dove*np.sin(self.beta), -self.R_dove*np.cos(self.beta)])
        self.dove_lower_point = self.dove_circle_center + np.array([0,-self.R_dove])

    @cached_property
    def dove_arc(self):
        "calculates firtree dove arc"
        return get_arc(self.dove_lower_point, self.origin, self.R_dove, self.dove_circle_center, self.num_arc_points, is_clockwise=False)

    def get_top_arc(self, include_tolerance: bool):
        "calculates firtree top arc"
        max_length = self.max_length + self.tolerance if include_tolerance else self.max_length
        sector_angle = 2*np.arcsin((max_length/2)/self.disk_radius)
        top_arc_left_point = self.left_side[-1]
        top_arc_right_point = top_arc_left_point + np.array([max_length, 0])
        top_arc_height = self.disk_radius - (max_length/2)/np.tan(sector_angle/2)
        disk_center = np.array([0,top_arc_left_point[1]-self.disk_radius+top_arc_height])
        return get_arc(top_arc_left_point, top_arc_right_point, self.disk_radius, disk_center, self.num_arc_points)

    def get_stage(self, end_stage: bool = False):
        "calculates firtree single stage coordinates"
        # Flank Lines
        yl = np.linspace(self.origin[1], self.outer_circle_lower_tangent[1], 2, endpoint=False)
        lower_flank_line = get_line(yl, self.origin, self.outer_circle_lower_tangent)

        yu = np.linspace(self.outer_circle_upper_tangent[1], self.inner_circle_lower_tangent[1], 2, endpoint=False)
        upper_flank_line = get_line(yu, self.outer_circle_tanget_intersect, self.outer_circle_upper_tangent)

        # Arcs
        outer_arc = get_arc(self.outer_circle_lower_tangent, self.outer_circle_upper_tangent, self.Ro, self.outer_circle_center, self.num_arc_points, endpoint=False)
        inner_arc = get_arc(self.inner_circle_lower_tangent, self.inner_circle_upper_tangent, self.Ri, self.inner_circle_center, self.num_arc_points, is_clockwise=False)
        
        stage_elements = [lower_flank_line,outer_arc, upper_flank_line]
        if not end_stage:
            stage_elements.append(inner_arc)
        
        return np.concatenate(stage_elements)
    
    @cached_property
    def left_side(self) -> np.ndarray:
        "calculates firtree attachment left side coordinates"
        stage = self.get_stage()
        attachment_stage_side:Optional[np.ndarray] = None
        assert self.num_stages > 0, "num stages must greater than 0"
        # TODO: make this more efficient with Numba
        for i in range(self.num_stages):
            next_stage = stage
            if attachment_stage_side is not None:
                attachment_stage_side = np.concatenate([
                    attachment_stage_side[:-1], 
                    next_stage + attachment_stage_side[-1]
                ])
            else:
                attachment_stage_side = next_stage
        
        # offset side to max length
        assert attachment_stage_side is not None
        attachment_center_offset = np.array([-attachment_stage_side[-1][0]-self.max_length/2, 0])
        return np.concatenate([self.dove_arc[:-1], attachment_stage_side]) + attachment_center_offset
    

    def get_coords(self, include_tolerance: bool):
        "calculates firtree attachment coordinates"
        top_arc = self.get_top_arc(include_tolerance)
        left_side = self.left_side + np.array([-self.tolerance/2, 0]) if include_tolerance else self.left_side
        attachment_right_side = np.flip(left_side, axis=0) * np.array([-1, 1])        
        attachment = np.concatenate([left_side[:-1], top_arc[:-1], attachment_right_side, [left_side[0]]])
        return attachment + np.array([0, -np.max(attachment[:,1])])

    @cached_property
    def coords(self):
        return self.get_coords(include_tolerance=False)

    @cached_property
    def coords_with_tolerance(self):
        return self.get_coords(include_tolerance=True)

    @cached_property
    def height(self):
        return np.max(self.coords[:, 1]) - np.min(self.coords[:, 1]) # type: ignore

    @cached_property
    def bottom_width(self):
        return np.abs(self.left_side[0][0])*2

    def visualize(self):
        fig = go.Figure()
        fig.add_trace(go.Scatter(
            x=self.coords[:, 0],
            y=self.coords[:, 1],
        ))
        fig.layout.yaxis.scaleanchor = "x"
        fig.show()

