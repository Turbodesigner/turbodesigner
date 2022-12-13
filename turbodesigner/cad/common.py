import cadquery as cq


class ExtendedWorkplane(cq.Workplane):
    def ring(self, radius: float, thickness: float, depth: float) -> "ExtendedWorkplane":
        return (
            self
            .circle(radius)
            .circle(radius - thickness)
            .extrude(depth)
        )


    def truncated_cone(self, start_radius: float, end_radius: float, height: float):
        plane_world_coords = self.plane.toWorldCoords((0, 0, 0))
        path = cq.Workplane("XZ").moveTo(0, plane_world_coords.z).lineTo(0, height + plane_world_coords.z)

        return (
            self
            .circle(start_radius)
            .transformed(offset=cq.Vector(0, 0, height))
            .circle(end_radius)
            .sweep(path, multisection=True, makeSolid=True)
            .clean()
        )

    def hollow_truncated_cone(self, inner_start_radius: float, inner_end_radius: float, height: float, start_thickness: float, end_thickness: float):
        outer_radius = inner_start_radius + start_thickness

        return (
            self
            .truncated_cone(outer_radius, outer_radius, height)
            .cut(
                ExtendedWorkplane("XY")
                .truncated_cone(inner_start_radius, inner_end_radius, height)
            )
        )

