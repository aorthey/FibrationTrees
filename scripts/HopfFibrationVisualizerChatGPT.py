import bpy
import math
import mathutils
from mathutils import Vector

# ---------------------------
# Colors (your specified start/end)
# ---------------------------
COLOR_START = (102/255, 229/255, 102/255)  # RGB for phi=0.5
COLOR_END = (180/255, 240/255, 180/255)  # RGB for phi=0.7

# ---------------------------
# Scene & render setup
# ---------------------------
# Clear scene
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Color management
bpy.context.scene.display_settings.display_device = 'sRGB'
bpy.context.scene.view_settings.view_transform = 'Standard'

# Use Cycles for correct Z-order and transparency handling
bpy.context.scene.render.engine = 'CYCLES'
bpy.context.scene.render.film_transparent = True
bpy.context.scene.render.image_settings.file_format = 'PNG'
bpy.context.scene.render.image_settings.color_mode = 'RGBA'  # keep alpha
bpy.context.scene.render.resolution_x = 2560
bpy.context.scene.render.resolution_y = 1440
bpy.context.scene.render.resolution_percentage = 100

# World (color irrelevant because film is transparent, but keep white for viewport)
world = bpy.context.scene.world or bpy.data.worlds.new("World")
bpy.context.scene.world = world
world.use_nodes = True
bg = world.node_tree.nodes.get('Background') or world.node_tree.nodes.new('ShaderNodeBackground')
bg.inputs[0].default_value = (1, 1, 1, 1)
bg.inputs[1].default_value = 1.0

# Remove any lights (unlit look)
for obj in list(bpy.data.objects):
    if obj.type == 'LIGHT':
        bpy.data.objects.remove(obj, do_unlink=True)

# ---------------------------
# Camera
# ---------------------------
def setup_camera():
    cam_data = bpy.data.cameras.new(name="HopfCamera")
    cam_data.lens = 50
    cam = bpy.data.objects.new("HopfCamera", cam_data)
    bpy.context.collection.objects.link(cam)

    max_extent = 2.0
    z_extent = 1.5
    distance = max_extent * 9
    angle = math.radians(20)

    x = distance * math.sin(angle)
    y = distance * math.cos(angle)
    z = z_extent / 2
    cam.location = (x, y, z)

    target = Vector((0, 0, 0))
    direction = target - Vector((x, y, z))
    rot_quat = direction.to_track_quat('-Z', 'Y')
    cam.rotation_euler = rot_quat.to_euler()

    bpy.context.scene.camera = cam
    cam_data.type = 'PERSP'
    cam_data.angle = math.radians(45)

setup_camera()

# ---------------------------
# Fiber creation (original geometry; stereographic projection)
# ---------------------------
def create_hopf_fiber_circle(theta, phi, segment_index, s,
                            num_points=256, fiber_width=0.05, fiber_alpha=0.6):
    """Create one fiber curve using your original projection and per-segment color."""
    vertices = []
    cos_ph2 = math.cos(phi / 2.0)
    sin_ph2 = math.sin(phi / 2.0)

    for i in range(num_points):
        alpha = 2.0 * math.pi * i / num_points
        sin_ta = math.sin(theta + alpha)
        cos_ta = math.cos(theta + alpha)
        denom = 1.0 - sin_ph2 * sin_ta
        if abs(denom) < 1e-6:
            denom = 1e-6
        x = cos_ph2 * math.cos(alpha) / denom
        y = cos_ph2 * math.sin(alpha) / denom
        z = sin_ph2 * cos_ta / denom
        vertices.append((x, y, 1.5 * z))
    vertices.append(vertices[0])

    # Curve data
    curve_data = bpy.data.curves.new(name=f"Fiber_{theta:.2f}_{phi:.2f}", type='CURVE')
    curve_data.dimensions = '3D'
    curve_data.resolution_u = 4
    curve_data.fill_mode = 'FULL'
    curve_data.bevel_depth = fiber_width

    poly = curve_data.splines.new('POLY')
    poly.points.add(len(vertices) - 1)
    for i, co in enumerate(vertices):
        poly.points[i].co = (*co, 1.0)

    obj = bpy.data.objects.new(f"Fiber_{theta:.2f}_{phi:.2f}", curve_data)
    bpy.context.collection.objects.link(obj)

    # Choose color per segment (start/end fixed, vertical interpolates)
    if segment_index == 0:
        r, g, b = COLOR_START
    elif segment_index == 2:
        r, g, b = COLOR_END
    else:
        r = COLOR_START[0] + s * (COLOR_END[0] - COLOR_START[0])
        g = COLOR_START[1] + s * (COLOR_END[1] - COLOR_START[1])
        b = COLOR_START[2] + s * (COLOR_END[2] - COLOR_START[2])

    # Unlit emission + transparency for Cycles
    mat = bpy.data.materials.new(name=f"FiberMat_{theta:.2f}_{phi:.2f}")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()

    out = nodes.new('ShaderNodeOutputMaterial')
    emit = nodes.new('ShaderNodeEmission')
    transparent_bsdf = nodes.new('ShaderNodeBsdfTransparent')
    mix_shader = nodes.new('ShaderNodeMixShader')

    emit.inputs['Color'].default_value = (r, g, b, 1.0)
    emit.inputs['Strength'].default_value = 1.0
    mix_shader.inputs['Fac'].default_value = 1.0 - float(fiber_alpha)

    links.new(transparent_bsdf.outputs[0], mix_shader.inputs[1])
    links.new(emit.outputs[0], mix_shader.inputs[2])
    links.new(mix_shader.outputs[0], out.inputs['Surface'])

    mat.cycles.use_transparent_shadow = True

    obj.data.materials.append(mat)
    return obj

# ---------------------------
# Paths (your original three segments)
# ---------------------------
num_points = 64
theta_start = 0.5 * math.pi
theta_end = 1.5 * math.pi
fiber_width = 0.05
fiber_alpha = 0.6

# Set render settings for speed
bpy.context.scene.cycles.samples = 128
bpy.context.scene.cycles.max_bounces = 2
bpy.context.scene.cycles.transparent_max_bounces = 8
bpy.context.scene.cycles.denoising_store_passes = True
# Change denoiser to a universal one
#bpy.context.scene.cycles.denoiser = 'OPENIMAGEDENOISE'
#bpy.context.scene.cycles.use_denoising = True

# Check for GPU and enable if available
try:
    bpy.context.preferences.addons['cycles'].preferences.compute_device_type = 'CUDA'
    bpy.context.preferences.addons['cycles'].preferences.get_devices()
    for d in bpy.context.preferences.addons['cycles'].preferences.devices:
        if d.type == 'CUDA':
            d.use = True
            print(f"Using GPU device: {d.name}")
            bpy.context.scene.cycles.device = 'GPU'
            break
except Exception as e:
    print(f"Could not set GPU as render device: {e}")
    bpy.context.scene.cycles.device = 'CPU'

phis = [0.5 * math.pi, 0.7 * math.pi]
paths = []
# First horizontal (phi = 0.5*pi)
paths.append([(theta_start + (theta_end - theta_start) * k / (num_points - 1), phis[0])
              for k in range(num_points)])
# Vertical (theta fixed, phi from 0.5*pi to 0.7*pi)
theta_fixed = (theta_start + theta_end) / 2.0
phi_start = 0.5 * math.pi
phi_end = 0.7 * math.pi
paths.append([(theta_fixed, phi_start + (phi_end - phi_start) * k / (num_points - 1))
              for k in range(num_points)])
# Second horizontal (phi = 0.7*pi)
paths.append([(theta_start + (theta_end - theta_start) * k / (num_points - 1), phis[1])
              for k in range(num_points)])

# Create fibers (no manual sorting needed for Cycles)
for segment_index, path in enumerate(paths):
    for k, (theta, phi) in enumerate(path):
        s = k / (num_points - 1) if num_points > 1 else 0.0
        create_hopf_fiber_circle(theta, phi, segment_index, s,
                                 num_points=256, fiber_width=fiber_width, fiber_alpha=fiber_alpha)

# ---------------------------
# Render to image (with alpha)
# ---------------------------
bpy.context.scene.render.filepath = 'hopf_fibration.png'
bpy.ops.render.render(write_still=True)
