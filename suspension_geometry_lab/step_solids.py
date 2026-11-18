"""Dependency-free AP214 export of independent, closed analytical cylinders."""
import math
from pathlib import Path
from tempfile import NamedTemporaryFile

import numpy as np


def write_cylinders(path, cylinders, description):
    """Write (name, start_m, end_m, radius_m) as manifold solid B-reps.

    Each cylinder has two half-cylinder faces and two planar caps. Splitting
    the lateral face avoids a periodic seam used twice by the same face; all
    six edges are shared by exactly two faces with opposite orientations.
    """
    entities = []

    def entity(value):
        entities.append(value)
        return f"#{len(entities)}"

    def number(value):
        if not math.isfinite(value):
            raise ValueError("STEP 實體尺寸與座標必須為有限數值。")
        result = f"{float(value):.15g}"
        if "e" in result:
            mantissa, exponent = result.split("e")
            return (mantissa if "." in mantissa else mantissa+".")+"E"+exponent
        return result if "." in result else result+"."

    def vector(values):
        return ",".join(number(value) for value in values)

    def point(values):
        return entity(f"CARTESIAN_POINT('',({vector(values)}))")

    def direction(values):
        return entity(f"DIRECTION('',({vector(values)}))")

    def placement(center, axis, reference):
        location, normal, x = point(center), direction(axis), direction(reference)
        return entity(f"AXIS2_PLACEMENT_3D('',{location},{normal},{x})")

    application = entity("APPLICATION_CONTEXT('automotive_design')")
    entity(f"APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,{application})")
    context = entity(f"PRODUCT_CONTEXT('',{application},'mechanical')")
    product = entity(f"PRODUCT('SuspensionGeometryLab','Suspension solids','',({context}))")
    formation = entity(f"PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('','',{product},.NOT_KNOWN.)")
    definition_context = entity(f"PRODUCT_DEFINITION_CONTEXT('part definition',{application},'design')")
    definition = entity(f"PRODUCT_DEFINITION('design','',{formation},{definition_context})")
    shape = entity(f"PRODUCT_DEFINITION_SHAPE('','',{definition})")
    mm = entity("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))")
    angle = entity("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))")
    solid_angle = entity("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())")
    uncertainty = entity(f"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(0.000001),{mm},'distance_accuracy_value','')")
    representation_context = entity(
        f"(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT(({uncertainty}))"
        f"GLOBAL_UNIT_ASSIGNED_CONTEXT(({mm},{angle},{solid_angle}))REPRESENTATION_CONTEXT('','3D'))")
    origin = placement(np.zeros(3), [0., 0., 1.], [1., 0., 0.])
    solids = []
    for name, start, end, radius in cylinders:
        start, end = np.asarray(start, dtype=float)*1000, np.asarray(end, dtype=float)*1000
        radius = float(radius)*1000
        if (start.shape != (3,) or end.shape != (3,) or not np.isfinite(start).all()
                or not np.isfinite(end).all() or not math.isfinite(radius) or radius <= 0):
            raise ValueError(f"{name}：STEP 圓柱需要有效的端點及正半徑。")
        length = float(np.linalg.norm(end-start))
        if not math.isfinite(length) or length <= 1e-6:
            raise ValueError(f"{name}：STEP 圓柱長度必須大於 0.000001 mm。")
        axis = (end-start)/length
        reference = np.eye(3)[int(np.argmin(np.abs(axis)))]
        reference = reference-axis*float(reference@axis)
        reference /= np.linalg.norm(reference)
        positions = (start+radius*reference, start-radius*reference,
                     end+radius*reference, end-radius*reference)
        locations = [point(position) for position in positions]
        vertices = [entity(f"VERTEX_POINT('',{location})") for location in locations]
        base = placement(start, axis, reference)
        top = placement(end, axis, reference)
        base_circle = entity(f"CIRCLE('',{base},{number(radius)})")
        top_circle = entity(f"CIRCLE('',{top},{number(radius)})")

        def edge(a, b, curve):
            return entity(f"EDGE_CURVE('',{vertices[a]},{vertices[b]},{curve},.T.)")

        bottom_a, bottom_b = edge(0, 1, base_circle), edge(1, 0, base_circle)
        top_a, top_b = edge(2, 3, top_circle), edge(3, 2, top_circle)
        displacement = entity(f"VECTOR('',{direction(axis)},{number(length)})")
        seam_a = edge(0, 2, entity(f"LINE('',{locations[0]},{displacement})"))
        seam_b = edge(1, 3, entity(f"LINE('',{locations[1]},{displacement})"))

        def face(surface, edges):
            oriented = [entity(f"ORIENTED_EDGE('',*,*,{ref},{'.T.' if forward else '.F.'})")
                        for ref, forward in edges]
            loop = entity(f"EDGE_LOOP('',({','.join(oriented)}))")
            bound = entity(f"FACE_OUTER_BOUND('',{loop},.T.)")
            return entity(f"ADVANCED_FACE('',({bound}),{surface},.T.)")

        side_a = entity(f"CYLINDRICAL_SURFACE('',{base},{number(radius)})")
        side_b = entity(f"CYLINDRICAL_SURFACE('',{placement(start, axis, -reference)},{number(radius)})")
        cap_bottom = entity(f"PLANE('',{placement(start, -axis, reference)})")
        cap_top = entity(f"PLANE('',{top})")
        faces = [
            face(side_a, [(bottom_a, True), (seam_b, True), (top_a, False), (seam_a, False)]),
            face(side_b, [(bottom_b, True), (seam_a, True), (top_b, False), (seam_b, False)]),
            face(cap_bottom, [(bottom_b, False), (bottom_a, False)]),
            face(cap_top, [(top_a, True), (top_b, True)]),
        ]
        shell = entity(f"CLOSED_SHELL('',({','.join(faces)}))")
        safe_name = name.encode("ascii").decode("ascii").replace("'", "''")
        solids.append(entity(f"MANIFOLD_SOLID_BREP('{safe_name}',{shell})"))
    if not solids:
        raise ValueError("目前沒有可匯出的 STEP 實體。")
    representation = entity(
        f"ADVANCED_BREP_SHAPE_REPRESENTATION('',({','.join([origin, *solids])}),{representation_context})")
    entity(f"SHAPE_DEFINITION_REPRESENTATION({shape},{representation})")
    header = ("ISO-10303-21;\nHEADER;\n"
              f"FILE_DESCRIPTION(('SuspensionGeometryLab cylinder solids','{description}'),'2;1');\n"
              "FILE_NAME('suspension.step','',(''),(''),'SuspensionGeometryLab','SuspensionGeometryLab','');\n"
              "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));\nENDSEC;\nDATA;\n")
    content = (header+"\n".join(f"#{i}={value};" for i, value in enumerate(entities, 1))
               +"\nENDSEC;\nEND-ISO-10303-21;\n").encode("ascii")
    destination, temporary = Path(path), None
    try:
        with NamedTemporaryFile(dir=destination.parent, prefix=".step-", delete=False) as handle:
            temporary = Path(handle.name)
            handle.write(content)
        temporary.replace(destination)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
