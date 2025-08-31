import bpy
import math
import colorsys

def create_hopf_fiber_circle(theta, phi, s, num_points=64, fiber_width=0.01, fiber_alpha=0.5, hue_start=0.333, hue_end=0.666):
    """
    Creates a single fiber circle for a given (theta, phi) point on the sphere.
    Uses stereographic projection and colors it based on s from green to blue with transparency.
    """
    vertices = []
    cos_ph2 = math.cos(phi / 2)
    sin_ph2 = math.sin(phi / 2)

    # Generate vertices for the circle
    for i in range(num_points):
        alpha = 2 * math.pi * i / num_points
        sin_ta = math.sin(theta + alpha)
        cos_ta = math.cos(theta + alpha)
        denom = 1 - sin_ph2 * sin_ta
        if denom == 0:
            denom = 0.0001  # Avoid division by zero
        x = cos_ph2 * math.cos(alpha) / denom
        y = cos_ph2 * math.sin(alpha) / denom
        z = sin_ph2 * cos_ta / denom
        vertices.append((x, y, 1.5*z))

    # Close the circle
    vertices.append(vertices[0])

    # Create curve data
    curve_data = bpy.data.curves.new(name=f"Fiber_{theta:.2f}_{phi:.2f}", type='CURVE')
    curve_data.dimensions = '3D'
    curve_data.resolution_u = 4  # Increased for smoother rendering
    curve_data.fill_mode = 'FULL'
    curve_data.bevel_depth = fiber_width  # Thickness of the fiber

    polyline = curve_data.splines.new('POLY')
    polyline.points.add(len(vertices) - 1)
    for i, coord in enumerate(vertices):
        polyline.points[i].co = (*coord, 1)  # w=1

    # Create curve object
    curve_obj = bpy.data.objects.new(f"Fiber_{theta:.2f}_{phi:.2f}", curve_data)
    bpy.context.collection.objects.link(curve_obj)

    # Add material with color based on s from green to blue and transparency
    mat = bpy.data.materials.new(name=f"FiberMaterial_{theta:.2f}_{phi:.2f}")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    curve_obj.data.materials.append(mat)

    # Color gradient from green (hue=0.333) to blue (hue=0.666) based on s
    hue = hue_start + s * (hue_end - hue_start)
    r, g, b = colorsys.hsv_to_rgb(hue, 1.0, 1.0)
    bsdf.inputs[0].default_value = (r, g, b, fiber_alpha)  # Set transparency
    mat.blend_method = 'HASHED'  # Changed to HASHED to fix transparency sorting issues in Eevee

    return curve_obj

# Parameters
num_points = 64
theta_start = 0.5 * math.pi
theta_end = 1.5 * math.pi
fiber_width = 0.05
fiber_alpha = 0.1

# Define paths
phis = [0.5 * math.pi, 0.7 * math.pi]
paths = []
for phi in phis:
    path = [(theta_start + (theta_end - theta_start) * k / (num_points - 1), phi) for k in range(num_points)]
    paths.append(path)

# Add vertical path (constant theta, varying phi from 0.5*pi to 0.7*pi)
theta_fixed = (theta_start + theta_end) / 2  # Fixed theta at midpoint
phi_start = 0.5 * math.pi
phi_end = 0.7 * math.pi
vertical_path = [(theta_fixed, phi_start + (phi_end - phi_start) * k / (num_points - 1)) for k in range(num_points)]
paths.append(vertical_path)

# Clear existing scene (optional)
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Create individual fiber circles for each path
for path in paths:
    for k, (theta, phi) in enumerate(path):
        s = k / (num_points - 1) if num_points > 1 else 0
        create_hopf_fiber_circle(theta, phi, s, num_points=256, fiber_width=fiber_width, fiber_alpha=fiber_alpha)
