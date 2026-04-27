import bpy
import math
import mathutils
from mathutils import Vector

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

def createHopfFiberCircle(theta, phi, color_start, color_end, s, num_points=64, fiber_width=0.01, fiber_alpha=0.5):
    """
    Creates a single fiber circle for a given (theta, phi) point on the sphere.
    Uses stereographic projection and interpolates between color_start and color_end based on s.
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

    # Add material with specific RGB colors
    mat = bpy.data.materials.new(name=f"FiberMaterial_{theta:.2f}_{phi:.2f}")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    curve_obj.data.materials.append(mat)

    # Interpolate color
    r = color_start[0] + s * (color_end[0] - color_start[0])
    g = color_start[1] + s * (color_end[1] - color_start[1])
    b = color_start[2] + s * (color_end[2] - color_start[2])
    bsdf.inputs[0].default_value = (r, g, b, fiber_alpha)  # Set color with transparency
    mat.blend_method = 'HASHED'  # For transparency sorting in Eevee

    return curve_obj

def createHopfFiberRibbon(theta, phi, color_start, color_end, s, num_points=64, fiber_width=0.1, fiber_alpha=0.2):
    """
    Creates a single fiber ribbon for a given (theta, phi) point on the sphere.
    Uses stereographic projection and interpolates between color_start and color_end based on s.
    Configures material for true transparency using fiber_alpha and creates flat ribbons.
    """
    import bpy
    import math
    from mathutils import Vector

    vertices = []
    cos_ph2 = math.cos(phi / 2)
    sin_ph2 = math.sin(phi / 2)

    # Generate vertices for the ribbon's centerline
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
        vertices.append(Vector((x, y, 1.5*z)))

    # Close the ribbon
    vertices.append(vertices[0])

    # Create mesh data
    mesh_data = bpy.data.meshes.new(name=f"RibbonMesh_{theta:.2f}_{phi:.2f}")
    ribbon_obj = bpy.data.objects.new(f"Ribbon_{theta:.2f}_{phi:.2f}", mesh_data)
    bpy.context.collection.objects.link(ribbon_obj)

    # Create vertices and edges for the centerline
    edges = [(i, i + 1) for i in range(len(vertices) - 1)]
    mesh_data.from_pydata(vertices, edges, [])

    # Add a Skin Modifier to extrude the ribbon
    skin_mod = ribbon_obj.modifiers.new(name="Skin", type='SKIN')

    # Set vertex radii for flat ribbon (wide in X, thin in Y)
    skin_verts = mesh_data.skin_vertices[0]  # Access the skin vertices layer
    for i, vert in enumerate(mesh_data.vertices):
        skin_verts.data[i].radius = (fiber_width, 0.001)  # Flat ribbon: wide in X, thin in Y

    # Add Subdivision Surface Modifier for smoothness
    subsurf_mod = ribbon_obj.modifiers.new(name="Subsurf", type='SUBSURF')
    subsurf_mod.levels = 2
    subsurf_mod.render_levels = 2

    # Create material with transparency
    mat = bpy.data.materials.new(name=f"RibbonMaterial_{theta:.2f}_{phi:.2f}")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links

    # Clear default nodes
    nodes.clear()

    # Create nodes
    output = nodes.new('ShaderNodeOutputMaterial')
    mix = nodes.new('ShaderNodeMixShader')
    transparent = nodes.new('ShaderNodeBsdfTransparent')
    bsdf = nodes.new('ShaderNodeBsdfPrincipled')

    # Interpolate color
    r = color_start[0] + s * (color_end[0] - color_start[0])
    g = color_start[1] + s * (color_end[1] - color_start[1])
    b = color_start[2] + s * (color_end[2] - color_start[2])
    bsdf.inputs['Base Color'].default_value = (r, g, b, fiber_alpha)

    # Set transparency
    mix.inputs['Fac'].default_value = fiber_alpha 
    transparent.inputs['Color'].default_value = (1.0, 1.0, 1.0, fiber_alpha)

    # Connect nodes
    links.new(bsdf.outputs['BSDF'], mix.inputs[2])
    links.new(transparent.outputs['BSDF'], mix.inputs[1])
    links.new(mix.outputs['Shader'], output.inputs['Surface'])

    # Configure material for Eevee transparency
    mat.blend_method = 'BLEND'
    mat.shadow_method = 'HASHED'
    mat.use_screen_refraction = True

    ribbon_obj.data.materials.append(mat)
    return ribbon_obj

def addCamera():
    """
    Sets up a camera pointing at the center of the Hopf fiber structure with the entire structure in view.
    """
    camera_data = bpy.data.cameras.new(name="HopfCamera")
    camera_data.lens = 50  # Standard lens for a natural perspective
    camera_obj = bpy.data.objects.new(name="HopfCamera", object_data=camera_data)
    bpy.context.collection.objects.link(camera_obj)

    max_extent = 2.0  # Approximate maximum x or y coordinate
    z_extent = 20.5  # Approximate maximum z coordinate
    distance = max_extent * 12  # Increased distance to fit structure in view (with margin)

    angle = math.radians(20)
    x = distance * math.sin(angle)
    y = distance * math.cos(angle)
    z = z_extent / 2
    camera_obj.location = (x, y, z)

    target = mathutils.Vector((0, 0, 0))
    direction = target - mathutils.Vector((x, y, z))
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

# Horizontal path 1
paths.append([(theta_start + (theta_end - theta_start) * k / (num_points - 1), phis[0]) for k in range(num_points)])
path_colors.append((MAGENTA_START, MAGENTA_END))

# Vertical path 1
theta_fixed = (theta_start + theta_end) / 2
phi_start = 0.5 * math.pi
phi_end = 0.7 * math.pi
paths.append([(theta_fixed, phi_start + (phi_end - phi_start) * k / (num_points - 1)) for k in range(num_points)])
path_colors.append((PASTEL_ORANGE_START, PASTEL_ORANGE_END))

# Horizontal path 2
paths.append([(theta_start + (theta_end - theta_start) * k / (num_points - 1), phis[1]) for k in range(num_points)])
path_colors.append((PASTEL_ORANGE_START, PASTEL_ORANGE_END))

# Vertical path 2
theta_fixed = (theta_start + theta_end) / 2
phi_start = 0.7 * math.pi
phi_end = 0.8 * math.pi
paths.append([(theta_fixed, phi_start + (phi_end - phi_start) * k / (num_points - 1)) for k in range(num_points)])
path_colors.append((COLOR_START, COLOR_END))
#path_colors.append((MAGENTA_START, MAGENTA_END))

# Horizontal path 3
paths.append([(theta_start + (theta_end - theta_start) * k / (num_points - 1), phis[2]) for k in range(num_points)])
path_colors.append((COLOR_START, COLOR_END))

# Clear existing scene
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Set color management to Standard
bpy.context.scene.display_settings.display_device = 'sRGB'
bpy.context.scene.view_settings.view_transform = 'Standard'

addCamera()

# Set transparent background
setBackgroundColor()

lc = Vector((7.5244, 20.6732, 0.7500))

addLightSourcePoint(-0.15 * lc + Vector((+0.5, 0, -0.5)), 500)
addLightSourcePoint(-0.2 * lc + Vector((1.5, -0.0, 0)), 500)
addLightSourcePoint(+0.3 * lc + Vector((0.0, -0.0, -1)), 500)

## illuminate inner left parts
addLightSourcePoint(+0.1 * lc + Vector((+2.0, 0.0, -2)), 50)
addLightSourcePoint(+0.1 * lc + Vector((+3.5, 0.0, -2.5)), 50)

## illuminate outer parts
addLightSourcePoint(0.3 * lc + Vector((0, 0, +2)), 500)
addLightSourcePoint(Vector((+0.0, 0.0, +10)), 1000)
addLightSourcePoint(Vector((+0.0, 0.0, -10)), 1000)
addLightSourcePoint(Vector((+10.0, 0.0, 0)), 1000)
addLightSourcePoint(Vector((-10.0, 0.0, 0)), 1000)

# Create fibers with path-specific coloring
for path_idx, path in enumerate(paths):
    color_start, color_end = path_colors[path_idx]
    for k, (theta, phi) in enumerate(path):
        s = k / (num_points - 1) if num_points > 1 else 0
        createHopfFiberCircle(theta, phi, color_start, color_end, s, num_points=256, fiber_width=fiber_width, fiber_alpha=fiber_alpha)
        #createHopfFiberRibbon(theta, phi, color_start, color_end, s, num_points=128)

# Optimize rendering settings for Eevee with transparency
bpy.context.scene.render.engine = 'BLENDER_EEVEE'
bpy.context.scene.eevee.use_bloom = True
bpy.context.scene.eevee.bloom_intensity = 0.1
bpy.context.scene.eevee.use_soft_shadows = True
bpy.context.scene.eevee.use_ssr = True
bpy.context.scene.eevee.use_ssr_refraction = True
bpy.context.scene.render.film_transparent = True  # Enable transparent background

# Set render output settings
bpy.context.scene.render.image_settings.file_format = 'PNG'
bpy.context.scene.render.image_settings.color_mode = 'RGBA'  # Ensure alpha channel is included
bpy.context.scene.render.filepath = 'hopf_fibration.png'

# Render the scene
bpy.ops.render.render(write_still=True)
