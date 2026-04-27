import bpy
import math
import mathutils
from mathutils import Vector
import random

# Add this variable
display_sphere = False  # Set to True to display the sphere instead of the Hopf fibration

# Number of random samples to place on the fibers
N = 1000  # Set N to the desired number of samples

# Global color variables for horizontal segments
COLOR_START = (102/255, 229/255, 102/255)  # RGB for phi=0.5
COLOR_END = (180/255, 240/255, 180/255)    # RGB for phi=0.7
# Pastel orange colors for vertical segments
PASTEL_ORANGE_START = (255/255, 204/255, 153/255)  # Light pastel orange
PASTEL_ORANGE_END = (255/255, 179/255, 128/255)    # Slightly darker pastel orange

MAGENTA_START = (150/255, 80/255, 155/255)
MAGENTA_END = (255/255, 0/255, 255/255)

def setBackgroundColor():
    """
    Sets the world background to fully transparent.
    """
    world = bpy.context.scene.world
    if world is None:
        new_world = bpy.data.worlds.new("New World")
        world = new_world
    world.use_nodes = True
    bg = world.node_tree.nodes.get('Background')
    if bg is None:
        bg = world.node_tree.nodes.new('ShaderNodeBackground')
    output = world.node_tree.nodes.get('World Output')
    if output is None:
        output = world.node_tree.nodes.new('ShaderNodeOutputWorld')
    world.node_tree.links.new(bg.outputs['Background'], output.inputs['Surface'])
    bg.inputs['Color'].default_value = (0, 0, 0, 0)  # Fully transparent
    bg.inputs['Strength'].default_value = 0.0

def dist(p1, p2):
    """
    Calculate Euclidean distance between two 3D points.
    """
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2 + (p1[2] - p2[2])**2)

def get_projection(theta, phi, alpha):
    cos_ph2 = math.cos(phi / 2)
    sin_ph2 = math.sin(phi / 2)
    sin_ta = math.sin(theta + alpha)
    cos_ta = math.cos(theta + alpha)
    denom = 1 - sin_ph2 * sin_ta
    if denom == 0:
        denom = 0.0001  # Avoid division by zero
    x = cos_ph2 * math.cos(alpha) / denom
    y = cos_ph2 * math.sin(alpha) / denom
    z = sin_ph2 * cos_ta / denom
    return (x, y, 1.5 * z)

def get_q_from_pos(pos):
    x, y, z_pos = pos
    z_un = z_pos / 1.5
    r2 = x**2 + y**2 + z_un**2
    if r2 + 1 == 0:
        return [0, 0, 0, 1.0]
    q3 = (r2 - 1) / (r2 + 1)
    q0 = 2 * x / (r2 + 1)
    q1 = 2 * y / (r2 + 1)
    q2 = 2 * z_un / (r2 + 1)
    norm = math.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
    if norm == 0:
        norm = 1e-10
    return [q0 / norm, q1 / norm, q2 / norm, q3 / norm]

def get_pos_from_q(q):
    denom = 1 - q[3]
    if abs(denom) < 1e-6:
        return None
    x = q[0] / denom
    y = q[1] / denom
    z_un = q[2] / denom
    return (x, y, 1.5 * z_un)

def createHopfFiberCircle(theta, phi, color_start, color_end, s, num_points=64, fiber_width=0.01, fiber_alpha=0.5):
    vertices = []
    for i in range(num_points):
        alpha = 2 * math.pi * i / num_points
        pos = get_projection(theta, phi, alpha)
        vertices.append(pos)
    vertices.append(vertices[0])

    curve_data = bpy.data.curves.new(name=f"Fiber_{theta:.2f}_{phi:.2f}", type='CURVE')
    curve_data.dimensions = '3D'
    curve_data.resolution_u = 4
    curve_data.fill_mode = 'FULL'
    curve_data.bevel_depth = fiber_width

    polyline = curve_data.splines.new('POLY')
    polyline.points.add(len(vertices) - 1)
    for i, coord in enumerate(vertices):
        polyline.points[i].co = (*coord, 1)

    curve_obj = bpy.data.objects.new(f"Fiber_{theta:.2f}_{phi:.2f}", curve_data)
    bpy.context.collection.objects.link(curve_obj)

    mat = bpy.data.materials.new(name=f"FiberMaterial_{theta:.2f}_{phi:.2f}")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    curve_obj.data.materials.append(mat)

    r = color_start[0] + s * (color_end[0] - color_start[0])
    g = color_start[1] + s * (color_end[1] - color_start[1])
    b = color_start[2] + s * (color_end[2] - color_start[2])
    bsdf.inputs[0].default_value = (r, g, b, fiber_alpha)
    mat.blend_method = 'HASHED'

    return curve_obj

def addCamera(theta_start, theta_end, phis, display_sphere):
    if display_sphere:
        max_extent = 1.0
        z_extent = -5.0
        theta_center = (theta_start + theta_end) / 2
        phi_center = (min(phis) + max(phis)) / 2
        x_center = math.sin(phi_center) * math.cos(theta_center)
        y_center = math.sin(phi_center) * math.sin(theta_center)
        z_center = math.cos(phi_center)
        cam_dir_x = x_center
        cam_dir_y = y_center
        cam_dir_z = z_center
        norm = math.sqrt(cam_dir_x**2 + cam_dir_y**2 + cam_dir_z**2)
        if norm > 0:
            cam_dir_x /= norm
            cam_dir_y /= norm
            cam_dir_z /= norm
        distance = 7.0
        cam_loc = (cam_dir_x * distance, cam_dir_y * distance, cam_dir_z * distance)
    else:
        max_extent = 2.0
        z_extent = 20.5
        distance = max_extent * 12
        angle = math.radians(20)
        cam_loc = (distance * math.sin(angle), distance * math.cos(angle), z_extent / 2)

    camera_data = bpy.data.cameras.new(name="HopfCamera")
    camera_data.lens = 50
    camera_obj = bpy.data.objects.new(name="HopfCamera", object_data=camera_data)
    bpy.context.collection.objects.link(camera_obj)

    camera_obj.location = cam_loc

    target = mathutils.Vector((0, 0, 0))
    direction = target - mathutils.Vector(cam_loc)
    rot_quat = direction.to_track_quat('-Z', 'Y')
    camera_obj.rotation_euler = rot_quat.to_euler()

    bpy.context.scene.camera = camera_obj
    camera_data.type = 'PERSP'
    camera_data.angle = math.radians(45)
    return camera_obj

def addLightSourcePoint(location, energy=5000):
    name = "light_source_point_" + str(location)
    light_data = bpy.data.lights.new(name=name, type='POINT')
    light_data.energy = energy
    light_data.specular_factor = 0.4
    light_data.use_contact_shadow = True
    light_object = bpy.data.objects.new(name="light_2.80", object_data=light_data)
    bpy.context.collection.objects.link(light_object)
    light_object.location = location

def addSpherePaths(paths, path_colors, fiber_width):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=128, ring_count=64, radius=1.0, location=(0, 0, 0))
    sphere_obj = bpy.context.object
    mat = bpy.data.materials.new(name="SphereMaterial")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()
    output = nodes.new('ShaderNodeOutputMaterial')
    mix = nodes.new('ShaderNodeMixShader')
    transparent = nodes.new('ShaderNodeBsdfTransparent')
    emission = nodes.new('ShaderNodeEmission')
    emission.inputs['Color'].default_value = (0.8, 0.8, 0.8, 1)
    emission.inputs['Strength'].default_value = 1.0
    wireframe = nodes.new('ShaderNodeWireframe')
    wireframe.inputs['Size'].default_value = 0.005
    links.new(wireframe.outputs['Fac'], mix.inputs['Fac'])
    links.new(transparent.outputs['BSDF'], mix.inputs[1])
    links.new(emission.outputs['Emission'], mix.inputs[2])
    links.new(mix.outputs['Shader'], output.inputs['Surface'])
    mat.blend_method = 'BLEND'
    sphere_obj.data.materials.append(mat)

    for path_idx, path in enumerate(paths):
        color_start, color_end = path_colors[path_idx]
        curve_data = bpy.data.curves.new(name=f"Path_{path_idx}", type='CURVE')
        curve_data.dimensions = '3D'
        curve_data.resolution_u = 4
        curve_data.fill_mode = 'FULL'
        curve_data.bevel_depth = fiber_width / 2
        polyline = curve_data.splines.new('POLY')
        polyline.points.add(len(path) - 1)
        for k, (theta, phi) in enumerate(path):
            x = math.sin(phi) * math.cos(theta)
            y = math.sin(phi) * math.sin(theta)
            z = math.cos(phi)
            polyline.points[k].co = (x, y, z, 1)
        curve_obj = bpy.data.objects.new(f"Path_{path_idx}", curve_data)
        bpy.context.collection.objects.link(curve_obj)
        mat = bpy.data.materials.new(name=f"PathMaterial_{path_idx}")
        mat.use_nodes = True
        bsdf = mat.node_tree.nodes["Principled BSDF"]
        gradient = mat.node_tree.nodes.new('ShaderNodeTexGradient')
        gradient.gradient_type = 'LINEAR'
        mapping = mat.node_tree.nodes.new('ShaderNodeMapping')
        mapping.vector_type = 'POINT'
        combine = mat.node_tree.nodes.new('ShaderNodeCombineXYZ')
        combine.inputs['X'].default_value = 1.0
        combine.inputs['Y'].default_value = 0.0
        combine.inputs['Z'].default_value = 0.0
        color_ramp = mat.node_tree.nodes.new('ShaderNodeValToRGB')
        color_ramp.color_ramp.elements[0].color = (*color_start, 1)
        color_ramp.color_ramp.elements[1].color = (*color_end, 1)
        mat.node_tree.links.new(combine.outputs['Vector'], mapping.inputs['Vector'])
        mat.node_tree.links.new(mapping.outputs['Vector'], gradient.inputs['Vector'])
        mat.node_tree.links.new(gradient.outputs['Color'], color_ramp.inputs['Fac'])
        mat.node_tree.links.new(color_ramp.outputs['Color'], bsdf.inputs['Base Color'])
        bsdf.inputs['Alpha'].default_value = 1.0
        mat.blend_method = 'HASHED'
        curve_obj.data.materials.append(mat)

# Parameters
num_points = 64
theta_start = 0.3 * math.pi
theta_end = 1.6 * math.pi
fiber_width = 0.05
fiber_alpha = 0.01

# Define paths
phis = [0.5 * math.pi, 0.7 * math.pi, 0.8 * math.pi]
paths = []
path_colors = []

paths.append([(theta_start + (theta_end - theta_start) * k / (num_points - 1), phis[0]) for k in range(num_points)])
path_colors.append((MAGENTA_START, MAGENTA_END))

theta_fixed = (theta_start + theta_end) / 2
phi_start = 0.5 * math.pi
phi_end = 0.7 * math.pi
paths.append([(theta_fixed, phi_start + (phi_end - phi_start) * k / (num_points - 1)) for k in range(num_points)])
path_colors.append((PASTEL_ORANGE_START, PASTEL_ORANGE_END))

paths.append([(theta_start + (theta_end - theta_start) * k / (num_points - 1), phis[1]) for k in range(num_points)])
path_colors.append((PASTEL_ORANGE_START, PASTEL_ORANGE_END))

phi_start = 0.7 * math.pi
phi_end = 0.8 * math.pi
paths.append([(theta_fixed, phi_start + (phi_end - phi_start) * k / (num_points - 1)) for k in range(num_points)])
path_colors.append((COLOR_START, COLOR_END))

paths.append([(theta_start + (theta_end - theta_start) * k / (num_points - 1), phis[2]) for k in range(num_points)])
path_colors.append((COLOR_START, COLOR_END))

# Clear existing scene
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Set color management to Standard
bpy.context.scene.display_settings.display_device = 'sRGB'
bpy.context.scene.view_settings.view_transform = 'Standard'

addCamera(theta_start, theta_end, phis, display_sphere)

setBackgroundColor()

lc = Vector((7.5244, 20.6732, 0.7500))

addLightSourcePoint(-0.15 * lc + Vector((+0.5, 0, -0.5)), 500)
addLightSourcePoint(-0.2 * lc + Vector((1.5, -0.0, 0)), 500)
addLightSourcePoint(+0.3 * lc + Vector((0.0, -0.0, -1)), 500)

addLightSourcePoint(+0.1 * lc + Vector((+2.0, 0.0, -2)), 50)
addLightSourcePoint(+0.1 * lc + Vector((+3.5, 0.0, -2.5)), 50)

addLightSourcePoint(0.3 * lc + Vector((0, 0, +2)), 500)
addLightSourcePoint(Vector((+0.0, 0.0, +10)), 1000)
addLightSourcePoint(Vector((+0.0, 0.0, -10)), 1000)
addLightSourcePoint(Vector((+10.0, 0.0, 0)), 1000)
addLightSourcePoint(Vector((-10.0, 0.0, 0)), 1000)

fiber_parameters = []

if not display_sphere:
    for path_idx, path in enumerate(paths):
        color_start, color_end = path_colors[path_idx]
        for k, (theta, phi) in enumerate(path):
            s = k / (num_points - 1) if num_points > 1 else 0
            createHopfFiberCircle(theta, phi, color_start, color_end, s, num_points=256, fiber_width=fiber_width, fiber_alpha=fiber_alpha)
            fiber_parameters.append((theta, phi))

# Uniform sampling and RRT
if not display_sphere and fiber_parameters:
    random.seed(42)

    unique_fibers = list(set(fiber_parameters))
    num_fibers = len(unique_fibers)
    lengths = []
    cum_lengths_list = []
    alphas_list = []
    pos_list_list = []
    num_length_samples = 500

    for theta, phi in unique_fibers:
        alphas = [2 * math.pi * k / num_length_samples for k in range(num_length_samples)]
        pos_list = [get_projection(theta, phi, a) for a in alphas]
        fiber_length = 0.0
        cum_lengths = [0.0]
        for k in range(1, num_length_samples):
            d = dist(pos_list[k], pos_list[k-1])
            fiber_length += d
            cum_lengths.append(cum_lengths[-1] + d)
        d_close = dist(pos_list[0], pos_list[-1])
        fiber_length += d_close
        lengths.append(fiber_length)
        cum_lengths_list.append(cum_lengths)
        alphas_list.append(alphas)
        pos_list_list.append(pos_list)

    total_length = sum(lengths)
    if total_length == 0:
        total_length = 1e-6

    def random_point_on_fibers():
        """
        Returns a uniformly sampled point on the fibers, considering their lengths.
        Returns (position, quaternion).
        """
        u = random.uniform(0, total_length)
        cum = 0.0
        for fib_idx in range(num_fibers):
            if u < cum + lengths[fib_idx]:
                u_local = u - cum
                cum_lengths = cum_lengths_list[fib_idx]
                alphas = alphas_list[fib_idx]
                pos_list = pos_list_list[fib_idx]
                if u_local <= cum_lengths[-1]:
                    for k in range(1, num_length_samples):
                        if cum_lengths[k] >= u_local:
                            frac = (u_local - cum_lengths[k-1]) / (cum_lengths[k] - cum_lengths[k-1])
                            pos_x = (1 - frac) * pos_list[k-1][0] + frac * pos_list[k][0]
                            pos_y = (1 - frac) * pos_list[k-1][1] + frac * pos_list[k][1]
                            pos_z = (1 - frac) * pos_list[k-1][2] + frac * pos_list[k][2]
                            pos = (pos_x, pos_y, pos_z)
                            q = get_q_from_pos(pos)
                            return pos, q
                else:
                    u_local -= cum_lengths[-1]
                    d_close = lengths[fib_idx] - cum_lengths[-1]
                    if d_close > 0:
                        frac = u_local / d_close
                    else:
                        frac = 0.0
                    pos_x = (1 - frac) * pos_list[-1][0] + frac * pos_list[0][0]
                    pos_y = (1 - frac) * pos_list[-1][1] + frac * pos_list[0][1]
                    pos_z = (1 - frac) * pos_list[-1][2] + frac * pos_list[0][2]
                    pos = (pos_x, pos_y, pos_z)
                    q = get_q_from_pos(pos)
                    return pos, q
            cum += lengths[fib_idx]
        theta, phi = unique_fibers[0]
        pos = get_projection(theta, phi, 0.0)
        q = get_q_from_pos(pos)
        return pos, q

    # Neon green material
    neon_green_mat = bpy.data.materials.new(name="NeonGreen")
    neon_green_mat.use_nodes = True
    nodes = neon_green_mat.node_tree.nodes
    links = neon_green_mat.node_tree.links
    nodes.clear()
    output = nodes.new('ShaderNodeOutputMaterial')
    emission = nodes.new('ShaderNodeEmission')
    emission.inputs['Color'].default_value = (0.1, 1.0, 0.1, 1)
    emission.inputs['Strength'].default_value = 5.0
    links.new(emission.outputs['Emission'], output.inputs['Surface'])

    def create_node(pos, radius=0.1):
        bpy.ops.mesh.primitive_ico_sphere_add(subdivisions=2, radius=radius, location=pos)
        obj = bpy.context.object
        obj.data.materials.append(neon_green_mat)

    # RRT implementation
    step_size = 0.1  # Angular step size in S^3

    # Select an outer fiber
    outer_phi = phis[2]  # Choose the outermost phi (0.8 * pi)
    outer_fibers = [(t, p) for t, p in unique_fibers if p == outer_phi]
    if outer_fibers:
        theta, phi = random.choice(outer_fibers)
    else:
        theta, phi = unique_fibers[0]
    alpha = random.uniform(0, 2 * math.pi)
    start_pos = get_projection(theta, phi, alpha)
    start_q = get_q_from_pos(start_pos)

    nodes_pos = [start_pos]
    nodes_q = [start_q]
    parents = [-1]

    iteration = 0
    max_attempts = N * 10  # Allow more attempts to reach N nodes
    while len(nodes_pos) < N and iteration < max_attempts:
        sample_pos, sample_q = random_point_on_fibers()

        max_dot = -2.0
        nearest_idx = -1
        for j in range(len(nodes_q)):
            dot = sum(nodes_q[j][k] * sample_q[k] for k in range(4))
            if dot > max_dot:
                max_dot = dot
                nearest_idx = j

        if max_dot >= 0.9999:  # Skip if too close
            iteration += 1
            continue

        theta_ang = math.acos(max_dot)
        f = min(step_size / theta_ang, 1.0) if theta_ang != 0 else 1.0
        sin_omt = math.sin((1 - f) * theta_ang)
        sin_ft = math.sin(f * theta_ang)
        sin_t = math.sin(theta_ang) if theta_ang != 0 else 1.0

        q_inter = [sin_omt / sin_t * nodes_q[nearest_idx][k] + sin_ft / sin_t * sample_q[k] for k in range(4)]
        norm = math.sqrt(sum(x**2 for x in q_inter))
        if norm > 0:
            q_inter = [x / norm for x in q_inter]

        # Snap to closest on finite fibers
        max_dot_snap = -2.0
        best_q_closest = None
        for fib_theta, fib_phi in unique_fibers:
            q0 = get_q_from_pos(get_projection(fib_theta, fib_phi, 0.0))
            a, b, c, d = q0
            A = q_inter[0] * a + q_inter[1] * b + q_inter[2] * c + q_inter[3] * d
            B = -q_inter[0] * b + q_inter[1] * a - q_inter[2] * d + q_inter[3] * c
            dot_fib = math.sqrt(A**2 + B**2)
            if dot_fib > max_dot_snap:
                max_dot_snap = dot_fib
                if A == 0 and B == 0:
                    beta = 0
                else:
                    beta = math.atan2(B, A)
                cos_b = math.cos(beta)
                sin_b = math.sin(beta)
                q_closest = [
                    cos_b * a - sin_b * b,
                    cos_b * b + sin_b * a,
                    cos_b * c - sin_b * d,
                    cos_b * d + sin_b * c
                ]
                best_q_closest = q_closest

        if best_q_closest is None:
            iteration += 1
            continue

        norm = math.sqrt(sum(x**2 for x in best_q_closest))
        if norm > 0:
            q_new = [x / norm for x in best_q_closest]

        # Check if too close to existing
        too_close = False
        for existing_q in nodes_q:
            dot = sum(existing_q[k] * q_new[k] for k in range(4))
            if dot >= 0.9999:
                too_close = True
                break
        if too_close:
            iteration += 1
            continue

        pos_new = get_pos_from_q(q_new)
        if pos_new is None or (abs(pos_new[0]) > 50 or abs(pos_new[1]) > 50 or abs(pos_new[2]) > 50):
            iteration += 1
            continue

        nodes_pos.append(pos_new)
        nodes_q.append(q_new)
        parents.append(nearest_idx)

        iteration += 1

    # Visualize the tree
    for pos in nodes_pos:
        create_node(pos)

    for i in range(1, len(nodes_pos)):
        parent = parents[i]
        q1 = nodes_q[parent]
        q2 = nodes_q[i]
        dot = sum(q1[k] * q2[k] for k in range(4))
        if dot >= 0.9999:
            continue
        theta = math.acos(dot)
        sin_theta = math.sin(theta)
        num_samples = 50  # Increased for smoother projection
        vertices = []
        for k in range(num_samples + 1):
            t = k / num_samples
            a = math.sin((1 - t) * theta) / sin_theta
            b = math.sin(t * theta) / sin_theta
            q_t = [a * q1[m] + b * q2[m] for m in range(4)]
            norm_t = math.sqrt(sum(x**2 for x in q_t))
            if norm_t > 0:
                q_t = [x / norm_t for x in q_t]

            # Project q_t onto the closest fiber
            max_dot_snap = -2.0
            best_q_closest = None
            for fib_theta, fib_phi in unique_fibers:
                q0 = get_q_from_pos(get_projection(fib_theta, fib_phi, 0.0))
                aa, bb, cc, dd = q0
                A = q_t[0] * aa + q_t[1] * bb + q_t[2] * cc + q_t[3] * dd
                B = -q_t[0] * bb + q_t[1] * aa - q_t[2] * dd + q_t[3] * cc
                dot_fib = math.sqrt(A**2 + B**2)
                if dot_fib > max_dot_snap:
                    max_dot_snap = dot_fib
                    beta = math.atan2(B, A)
                    cos_b = math.cos(beta)
                    sin_b = math.sin(beta)
                    q_closest = [
                        cos_b * aa - sin_b * bb,
                        cos_b * bb + sin_b * aa,
                        cos_b * cc - sin_b * dd,
                        cos_b * dd + sin_b * cc
                    ]
                    best_q_closest = q_closest

            if best_q_closest is None:
                continue

            norm = math.sqrt(sum(x**2 for x in best_q_closest))
            if norm > 0:
                q_snapped = [x / norm for x in best_q_closest]
            else:
                continue

            pos_t = get_pos_from_q(q_snapped)
            if pos_t is None or (abs(pos_t[0]) > 50 or abs(pos_t[1]) > 50 or abs(pos_t[2]) > 50):
                continue

            vertices.append(pos_t)

        if len(vertices) > 1:
            curve_data = bpy.data.curves.new(name=f"Edge_{i}", type='CURVE')
            curve_data.dimensions = '3D'
            curve_data.resolution_u = 4
            curve_data.fill_mode = 'FULL'
            curve_data.bevel_depth = 0.02
            polyline = curve_data.splines.new('POLY')
            polyline.points.add(len(vertices) - 1)
            for j, coord in enumerate(vertices):
                polyline.points[j].co = (*coord, 1)
            curve_obj = bpy.data.objects.new(f"Edge_{i}", curve_data)
            bpy.context.collection.objects.link(curve_obj)
            curve_obj.data.materials.append(neon_green_mat)

else:
    addSpherePaths(paths, path_colors, fiber_width)

# Optimize rendering settings for Eevee with transparency
bpy.context.scene.render.engine = 'BLENDER_EEVEE'
bpy.context.scene.eevee.use_bloom = True
bpy.context.scene.eevee.bloom_intensity = 0.1
bpy.context.scene.eevee.use_soft_shadows = True
bpy.context.scene.eevee.use_ssr = True
bpy.context.scene.eevee.use_ssr_refraction = True
bpy.context.scene.render.film_transparent = True

# Set render output settings
bpy.context.scene.render.image_settings.file_format = 'PNG'
bpy.context.scene.render.image_settings.color_mode = 'RGBA'
if display_sphere:
    bpy.context.scene.render.filepath = 'hopf_fibration_sphere.png'
else:
    bpy.context.scene.render.filepath = 'hopf_fibration.png'

# Render the scene
bpy.ops.render.render(write_still=True)
