import bpy
import bmesh
import math
import mathutils
from mathutils import Vector
import numpy as np

# ---------------------------
# Colors (enhanced color palette)
# ---------------------------
COLOR_START = (0.2, 0.8, 1.0)    # Cyan
COLOR_MID1 = (0.4, 1.0, 0.4)     # Green
COLOR_MID2 = (1.0, 0.8, 0.2)     # Orange
COLOR_END = (1.0, 0.3, 0.8)      # Magenta

# ---------------------------
# Scene & render setup
# ---------------------------
# Clear scene
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Color management
bpy.context.scene.display_settings.display_device = 'sRGB'
bpy.context.scene.view_settings.view_transform = 'Standard'

# Use Cycles for correct rendering
# bpy.context.scene.render.engine = 'CYCLES'
# bpy.context.scene.render.film_transparent = True
# bpy.context.scene.render.image_settings.file_format = 'PNG'
# bpy.context.scene.render.image_settings.color_mode = 'RGBA'
# bpy.context.scene.render.resolution_x = 2560
# bpy.context.scene.render.resolution_y = 1440
# bpy.context.scene.render.resolution_percentage = 100

# # Enhanced render settings for quality
# bpy.context.scene.cycles.samples = 512
# bpy.context.scene.cycles.max_bounces = 12
# bpy.context.scene.cycles.transparent_max_bounces = 12
# bpy.context.scene.cycles.use_denoising = True

bpy.context.scene.render.engine = 'BLENDER_EEVEE'
bpy.context.scene.eevee.use_bloom = True  # Add subtle glow
bpy.context.scene.eevee.bloom_intensity = 0.1
bpy.context.scene.eevee.use_soft_shadows = True

bpy.context.scene.eevee.use_ssr = True
bpy.context.scene.eevee.use_ssr_refraction = True

# World setup with transparent background
world = bpy.context.scene.world or bpy.data.worlds.new("World")
bpy.context.scene.world = world
world.use_nodes = True
bg = world.node_tree.nodes.get('Background') or world.node_tree.nodes.new('ShaderNodeBackground')
bg.inputs[0].default_value = (0, 0, 0, 0)  # Transparent
bg.inputs[1].default_value = 0.0

# Remove any lights
for obj in list(bpy.data.objects):
    if obj.type == 'LIGHT':
        bpy.data.objects.remove(obj, do_unlink=True)

# ---------------------------
# Camera setup
# ---------------------------
def setup_camera():
    cam_data = bpy.data.cameras.new(name="HopfCamera")
    cam_data.lens = 85  # Slightly longer lens for better perspective
    cam = bpy.data.objects.new("HopfCamera", cam_data)
    bpy.context.collection.objects.link(cam)

    # Position camera for aesthetic view
    distance = 8.0
    angle = math.radians(25)

    x = distance * math.sin(angle)
    y = distance * math.cos(angle)
    z = 1.2
    cam.location = (x, y, z)

    target = Vector((0, 0, 0))
    direction = target - Vector((x, y, z))
    rot_quat = direction.to_track_quat('-Z', 'Y')
    cam.rotation_euler = rot_quat.to_euler()

    bpy.context.scene.camera = cam
    cam_data.type = 'PERSP'
    cam_data.angle = math.radians(40)

setup_camera()

# ---------------------------
# Enhanced Hopf fibration functions
# ---------------------------
def hopf_to_s2(theta, phi):
    """Convert Hopf coordinates to S2 (base space)"""
    return (
        math.cos(phi) * math.cos(theta),
        math.cos(phi) * math.sin(theta),
        math.sin(phi)
    )

def s2_to_s3_fiber(x, y, z, t):
    """Lift point on S2 to fiber on S3"""
    # Stereographic projection from S2 to S3
    norm_sq = x*x + y*y + z*z
    if norm_sq < 1e-10:
        return (math.cos(t), math.sin(t), 0, 0)

    factor = 1.0 / (1.0 + z)

    # Parameterize the fiber
    cos_t = math.cos(t)
    sin_t = math.sin(t)

    w = cos_t * math.sqrt((1 + z) / 2)
    x_s3 = sin_t * math.sqrt((1 + z) / 2)
    y_s3 = cos_t * x * factor
    z_s3 = cos_t * y * factor

    return (w, x_s3, y_s3, z_s3)

def s3_to_r3_stereo(w, x, y, z):
    """Stereographic projection from S3 to R3"""
    denom = 1.0 - w
    if abs(denom) < 1e-6:
        denom = 1e-6
    return (x / denom, y / denom, z / denom)

def create_smooth_surface_segment(path_points, width_samples=32, tube_radius=0.15, segment_index=0):
    """Create a smooth tubular surface along the path"""

    # Create mesh data
    mesh = bpy.data.meshes.new(f"HopfSurface_{segment_index}")

    # Generate vertices for tubular surface
    vertices = []
    faces = []

    path_length = len(path_points)

    for i, (theta, phi) in enumerate(path_points):
        # Get the fiber circle for this point
        fiber_points = []
        for j in range(width_samples):
            alpha = 2.0 * math.pi * j / width_samples

            # Original stereographic projection math
            cos_ph2 = math.cos(phi / 2.0)
            sin_ph2 = math.sin(phi / 2.0)
            sin_ta = math.sin(theta + alpha)
            cos_ta = math.cos(theta + alpha)
            denom = 1.0 - sin_ph2 * sin_ta
            if abs(denom) < 1e-6:
                denom = 1e-6

            x = cos_ph2 * math.cos(alpha) / denom
            y = cos_ph2 * math.sin(alpha) / denom
            z = sin_ph2 * cos_ta / denom

            # Scale and add to vertices
            vertices.append((x, y, 1.5 * z))

        # Create faces connecting this ring to the previous ring
        if i > 0:
            for j in range(width_samples):
                j_next = (j + 1) % width_samples

                # Current ring indices
                curr_base = i * width_samples
                prev_base = (i - 1) * width_samples

                # Create quad face
                face = [
                    prev_base + j,
                    prev_base + j_next,
                    curr_base + j_next,
                    curr_base + j
                ]
                faces.append(face)

    # Create the mesh
    mesh.from_pydata(vertices, [], faces)
    mesh.update()

    # Create object
    obj = bpy.data.objects.new(f"HopfSurface_{segment_index}", mesh)
    bpy.context.collection.objects.link(obj)

    # Add smooth shading
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.faces_shade_smooth()
    bpy.ops.object.mode_set(mode='OBJECT')

    # Create vibrant material with gradient effect
    mat = bpy.data.materials.new(name=f"HopfMaterial_{segment_index}")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()

    # Material output
    output = nodes.new('ShaderNodeOutputMaterial')

    # Mix shader for transparency
    mix_shader = nodes.new('ShaderNodeMixShader')

    # Transparent BSDF
    transparent = nodes.new('ShaderNodeBsdfTransparent')

    # Emission shader for glow effect
    emission = nodes.new('ShaderNodeEmission')

    # Color ramp for gradient
    color_ramp = nodes.new('ShaderNodeValToRGB')
    color_ramp.color_ramp.elements[0].color = (*COLOR_START, 1.0) if segment_index == 0 else (
        *COLOR_MID1, 1.0) if segment_index == 1 else (*COLOR_END, 1.0)
    color_ramp.color_ramp.elements[1].color = (*COLOR_MID2, 1.0) if segment_index == 0 else (
        *COLOR_END, 1.0) if segment_index == 1 else (*COLOR_START, 1.0)

    # Texture coordinate for gradient
    tex_coord = nodes.new('ShaderNodeTexCoord')
    mapping = nodes.new('ShaderNodeMapping')
    noise = nodes.new('ShaderNodeTexNoise')
    noise.inputs['Scale'].default_value = 5.0
    noise.inputs['Detail'].default_value = 3.0

    # Connect nodes
    links.new(tex_coord.outputs['Generated'], mapping.inputs['Vector'])
    links.new(mapping.outputs['Vector'], noise.inputs['Vector'])
    links.new(noise.outputs['Fac'], color_ramp.inputs['Fac'])
    links.new(color_ramp.outputs['Color'], emission.inputs['Color'])

    # Emission strength for glow
    emission.inputs['Strength'].default_value = 2.0

    # Mix transparent and emission
    mix_shader.inputs['Fac'].default_value = 0.15  # More transparent
    links.new(transparent.outputs['BSDF'], mix_shader.inputs[1])
    links.new(emission.outputs['Emission'], mix_shader.inputs[2])
    links.new(mix_shader.outputs['Shader'], output.inputs['Surface'])

    # Enable transparency
    mat.blend_method = 'BLEND'
    mat.use_screen_refraction = True

    obj.data.materials.append(mat)

    return obj

def create_flowing_ribbons(num_ribbons=24, ribbon_width=0.3):
    """Create flowing ribbon-like surfaces"""

    phi_values = np.linspace(0.3 * math.pi, 0.8 * math.pi, num_ribbons)

    for ribbon_idx, phi in enumerate(phi_values):
        # Create path points for this ribbon
        num_path_points = 128
        theta_start = 0.0
        theta_end = 4.0 * math.pi  # Multiple revolutions

        path_points = []
        for i in range(num_path_points):
            t = i / (num_path_points - 1)
            theta = theta_start + (theta_end - theta_start) * t

            # Add some variation to phi for flowing effect
            phi_var = phi + 0.1 * math.sin(3 * theta) * math.exp(-0.5 * t)
            path_points.append((theta, phi_var))

        # Create surface for this ribbon
        create_smooth_surface_segment(path_points, width_samples=24,
                                    tube_radius=ribbon_width, segment_index=ribbon_idx % 3)

# ---------------------------
# Main execution
# ---------------------------

# Create the flowing ribbons effect
create_flowing_ribbons(num_ribbons=18, ribbon_width=0.25)

# Add some additional decorative elements
def create_core_torus():
    """Create a central torus for visual interest"""
    bpy.ops.mesh.primitive_torus_add(
        major_radius=0.8,
        minor_radius=0.15,
        location=(0, 0, 0)
    )

    torus = bpy.context.active_object

    # Core material with subtle glow
    mat = bpy.data.materials.new(name="CoreMaterial")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()

    output = nodes.new('ShaderNodeOutputMaterial')
    emission = nodes.new('ShaderNodeEmission')
    transparent = nodes.new('ShaderNodeBsdfTransparent')
    mix = nodes.new('ShaderNodeMixShader')

    emission.inputs['Color'].default_value = (0.8, 0.9, 1.0, 1.0)
    emission.inputs['Strength'].default_value = 1.5
    mix.inputs['Fac'].default_value = 0.3

    links.new(transparent.outputs['BSDF'], mix.inputs[1])
    links.new(emission.outputs['Emission'], mix.inputs[2])
    links.new(mix.outputs['Shader'], output.inputs['Surface'])

    mat.blend_method = 'BLEND'
    torus.data.materials.append(mat)

# Uncomment to add central torus
# create_core_torus()

# ---------------------------
# Enhanced lighting setup
# ---------------------------
def setup_lighting():
    """Add area lights for better illumination"""

    # Key light
    bpy.ops.object.light_add(type='AREA', location=(5, 5, 5))
    key_light = bpy.context.active_object
    key_light.data.energy = 50
    key_light.data.size = 2.0
    key_light.data.color = (1.0, 0.95, 0.9)

    # Fill light
    bpy.ops.object.light_add(type='AREA', location=(-3, 2, 3))
    fill_light = bpy.context.active_object
    fill_light.data.energy = 20
    fill_light.data.size = 3.0
    fill_light.data.color = (0.9, 0.95, 1.0)

    # Rim light
    bpy.ops.object.light_add(type='AREA', location=(2, -4, 2))
    rim_light = bpy.context.active_object
    rim_light.data.energy = 30
    rim_light.data.size = 1.5
    rim_light.data.color = (1.0, 0.8, 1.0)

setup_lighting()

# ---------------------------
# Post-processing and effects
# ---------------------------
def setup_compositor():
    """Setup compositor for glow and enhancement effects"""
    bpy.context.scene.use_nodes = True
    tree = bpy.context.scene.node_tree

    # Clear existing nodes
    for node in tree.nodes:
        tree.nodes.remove(node)

    # Input and output
    render_layers = tree.nodes.new('CompositorNodeRLayers')
    composite = tree.nodes.new('CompositorNodeComposite')

    # Blur for glow effect
    blur = tree.nodes.new('CompositorNodeBlur')
    blur.size_x = 10
    blur.size_y = 10
    blur.use_relative = True

    # Mix for adding glow
    mix = tree.nodes.new('CompositorNodeMixRGB')
    mix.blend_type = 'ADD'
    mix.inputs['Fac'].default_value = 0.3

    # Color balance for enhancement
    color_balance = tree.nodes.new('CompositorNodeColorBalance')
    color_balance.correction_method = 'LIFT_GAMMA_GAIN'
    color_balance.lift = (1.0, 1.0, 1.05)
    color_balance.gamma = (0.95, 0.95, 0.95)
    color_balance.gain = (1.05, 1.0, 1.05)

    # Connect nodes
    tree.links.new(render_layers.outputs['Image'], blur.inputs['Image'])
    #tree.links.new(render_layers.outputs['Image'], mix.inputs['Color1'])
    #tree.links.new(blur.outputs['Image'], mix.inputs['Color2'])
    tree.links.new(mix.outputs['Image'], color_balance.inputs['Image'])
    tree.links.new(color_balance.outputs['Image'], composite.inputs['Image'])
    tree.links.new(render_layers.outputs['Alpha'], composite.inputs['Alpha'])

setup_compositor()

# ---------------------------
# Original fiber creation (keeping your paths)
# ---------------------------
def create_hopf_fiber_circle(theta, phi, segment_index, s,
                            num_points=256, fiber_width=0.03, fiber_alpha=0.4):
    """Create one fiber curve using your original projection (now thinner)"""
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

    # Curve data (thinner than before)
    curve_data = bpy.data.curves.new(name=f"Fiber_{theta:.2f}_{phi:.2f}", type='CURVE')
    curve_data.dimensions = '3D'
    curve_data.resolution_u = 8
    curve_data.fill_mode = 'FULL'
    curve_data.bevel_depth = fiber_width

    poly = curve_data.splines.new('POLY')
    poly.points.add(len(vertices) - 1)
    for i, co in enumerate(vertices):
        poly.points[i].co = (*co, 1.0)

    obj = bpy.data.objects.new(f"Fiber_{theta:.2f}_{phi:.2f}", curve_data)
    bpy.context.collection.objects.link(obj)

    # Color interpolation
    if segment_index == 0:
        r, g, b = COLOR_START
    elif segment_index == 2:
        r, g, b = COLOR_END
    else:
        r = COLOR_START[0] + s * (COLOR_END[0] - COLOR_START[0])
        g = COLOR_START[1] + s * (COLOR_END[1] - COLOR_START[1])
        b = COLOR_START[2] + s * (COLOR_END[2] - COLOR_START[2])

    # Enhanced material for fibers
    mat = bpy.data.materials.new(name=f"FiberMat_{theta:.2f}_{phi:.2f}")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    nodes.clear()

    output = nodes.new('ShaderNodeOutputMaterial')
    emission = nodes.new('ShaderNodeEmission')
    transparent = nodes.new('ShaderNodeBsdfTransparent')
    mix_shader = nodes.new('ShaderNodeMixShader')

    emission.inputs['Color'].default_value = (r, g, b, 1.0)
    emission.inputs['Strength'].default_value = 1.5
    mix_shader.inputs['Fac'].default_value = 1.0 - fiber_alpha

    links.new(transparent.outputs[0], mix_shader.inputs[1])
    links.new(emission.outputs[0], mix_shader.inputs[2])
    links.new(mix_shader.outputs[0], output.inputs['Surface'])

    mat.blend_method = 'BLEND'
    obj.data.materials.append(mat)
    return obj

# ---------------------------
# Create original paths (keeping your structure)
# ---------------------------
num_points = 32  # Reduced for performance with surfaces
theta_start = 0.5 * math.pi
theta_end = 1.5 * math.pi
fiber_width = 0.02  # Thinner fibers
fiber_alpha = 0.3

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

# Create surface segments for your original paths
for segment_index, path in enumerate(paths):
    create_smooth_surface_segment(path, width_samples=24, tube_radius=0.12, segment_index=segment_index)

# Also create some fiber curves for detail (every 4th point)
for segment_index, path in enumerate(paths):
    for k in range(0, len(path), 4):  # Every 4th point
        theta, phi = path[k]
        s = k / (num_points - 1) if num_points > 1 else 0.0
        create_hopf_fiber_circle(theta, phi, segment_index, s,
                                 num_points=128, fiber_width=fiber_width, fiber_alpha=fiber_alpha)

# ---------------------------
# Enhanced GPU settings
# ---------------------------
try:
    # Try CUDA first
    bpy.context.preferences.addons['cycles'].preferences.compute_device_type = 'CUDA'
    bpy.context.preferences.addons['cycles'].preferences.get_devices()
    gpu_found = False

    for device in bpy.context.preferences.addons['cycles'].preferences.devices:
        if device.type == 'CUDA' and 'GeForce' in device.name or 'RTX' in device.name:
            device.use = True
            gpu_found = True
            print(f"Using GPU: {device.name}")

    if not gpu_found:
        # Try OpenCL as fallback
        bpy.context.preferences.addons['cycles'].preferences.compute_device_type = 'OPENCL'
        bpy.context.preferences.addons['cycles'].preferences.get_devices()
        for device in bpy.context.preferences.addons['cycles'].preferences.devices:
            if device.type == 'OPENCL':
                device.use = True
                gpu_found = True
                print(f"Using OpenCL GPU: {device.name}")
                break

    if gpu_found:
        bpy.context.scene.cycles.device = 'GPU'
    else:
        print("No GPU found, using CPU")
        bpy.context.scene.cycles.device = 'CPU'

except Exception as e:
    print(f"GPU setup failed: {e}")
    bpy.context.scene.cycles.device = 'CPU'

# ---------------------------
# Final render
# ---------------------------
print("Rendering enhanced Hopf fibration...")
bpy.context.scene.render.filepath = 'hopf_fibration_enhanced.png'
bpy.ops.render.render(write_still=True)
print("Render complete!")
