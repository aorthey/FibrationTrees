import bpy
import bmesh
import mathutils
import math
import numpy as np
from colorsys import hsv_to_rgb
import yaml
import argparse
import sys
import re

n_points = 64

#edge_width = 0.015
edge_width = 0.5

# --- Smoothness parameter for edge interpolation ---
segment_smooth_stride = 10  # Increase for smoother, decrease for more accurate (1 = all points)

# Function to parse command-line arguments
def parse_arguments():
    parser = argparse.ArgumentParser(description="Visualize Hopf fibration data in Blender")
    parser.add_argument(
        "--dat_file",
        type=str,
        required=True,
        help="Path to the hopffibration_trees.dat file"
    )
    parser.add_argument(
        "--fiber_width",
        type=float,
        default=0.05,
        help="Width of the fibers (default: 0.001)"
    )
    parser.add_argument(
        "--alpha",
        type=float,
        default=0.5,
        help="Transparency of fibers and surface (0.0 to 1.0, default: 0.5)"
    )
    parser.add_argument(
        "--camera_distance",
        type=float,
        default=2.0,
        help="Camera distance multiplier relative to scene size (default: 2.0)"
    )
    parser.add_argument(
        "--camera_height",
        type=float,
        default=0.8,
        help="Camera height ratio (0.0-1.0) (default: 0.8)"
    )
    parser.add_argument(
        "--camera_lens",
        type=float,
        default=50.0,
        help="Camera lens length in mm (default: 50.0)"
    )
    parser.add_argument(
        "--main_light_energy",
        type=float,
        default=5.0,
        help="Main light energy (default: 5.0)"
    )
    parser.add_argument(
        "--fill_light_energy",
        type=float,
        default=3.0,
        help="Fill light energy (default: 3.0)"
    )
    return parser.parse_args(sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])

# Function to preprocess the .dat file to fix missing dashes
def preprocess_dat_file(content):
    lines = content.splitlines()
    corrected_lines = []
    in_fibers_section = False
    fibers_indent = None

    for i, line in enumerate(lines):
        # Replace tabs with 2 spaces for consistent indentation
        line = line.replace('\t', '  ')
        stripped_line = line.strip()

        # Detect start of fibers section
        if stripped_line == 'fibers:':
            in_fibers_section = True
            fibers_indent = len(line) - len(line.lstrip())
            corrected_lines.append(line)
            continue

        # Within fibers section, process fiber entries
        if in_fibers_section and re.match(r'^fiber\d+:', stripped_line):
            # Determine indentation for fiber entries
            if fibers_indent is None:
                fibers_indent = len(line) - len(line.lstrip())

            # Check if the line already has a dash
            if not stripped_line.startswith('-'):
                # Add dash with correct indentation
                corrected_line = ' ' * (fibers_indent - 2) + '- ' + stripped_line
            else:
                # Keep existing dash, ensure correct indentation
                corrected_line = ' ' * (fibers_indent - 2) + stripped_line.lstrip('- ')

            corrected_lines.append(corrected_line)
        else:
            corrected_lines.append(line)

        # Detect end of fibers section
        if in_fibers_section and stripped_line and (len(line) - len(line.lstrip()) <= fibers_indent or not stripped_line):
            in_fibers_section = False

    # Debug: Log first 10 lines of the fibers section
    fibers_start = next((i for i, line in enumerate(corrected_lines) if line.strip().startswith('fibers:')), -1)
    if fibers_start != -1:
        print("First 10 lines of corrected fibers section:")
        for i in range(fibers_start, min(fibers_start + 10, len(corrected_lines))):
            print(f"Line {i + 1}: {corrected_lines[i]}")

    # Save corrected content for inspection
    corrected_content = '\n'.join(corrected_lines)
    with open('hopffibration_trees.dat.corrected', 'w') as f:
        f.write(corrected_content)

    return corrected_content

# Function to parse the .dat file
def parse_dat_file(file_path):
    try:
        with open(file_path, 'r') as file:
            content = file.read()

        # Preprocess to fix missing dashes
        corrected_content = preprocess_dat_file(content)

        # Parse corrected YAML content
        data = yaml.safe_load(corrected_content)

        # Debug: Print available top-level keys
        print("Parsed data top-level keys:", list(data.keys()))
        print("Trees keys:", list(data.get('trees', {}).keys()))

        if data['trees']['so3_tree']:
          if data['trees']['so3_tree']['goal']:
              goal = data['trees']['so3_tree']['goal']
          else:
              goal = None
          # Extract so3_tree
          so3_tree = {
              'start': data['trees']['so3_tree']['start'],
              'goal': goal,
              'edges': [
                  {'source': edge['source'], 'target': edge['target']}
                  for edge in data['trees']['so3_tree']['edges']
              ]
          }

        # Extract fibers from 'fibers' list
        fibers = {}
        for fiber_dict in data.get('fibers', []):
            try:
                fiber_key = next(iter(fiber_dict))
                if fiber_key.startswith('fiber') and 'states' in fiber_dict[fiber_key]:
                    fibers[fiber_key] = fiber_dict[fiber_key]['states']
            except StopIteration:
                print(f"Warning: Empty fiber dictionary at index {len(fibers)}")
                continue

        # Debug: Print number of fibers detected
        print(f"Detected {len(fibers)} fibers: {list(fibers.keys())}")

        return so3_tree, fibers

    except FileNotFoundError:
        print(f"Error: File {file_path} not found.")
        sys.exit(1)
    except yaml.YAMLError as e:
        print(f"Error parsing YAML: {e}")
        print("Check hopffibration_trees.dat.corrected for preprocessed content.")
        sys.exit(1)
    except KeyError as e:
        print(f"Error: Missing key in data structure: {e}")
        sys.exit(1)

# Clear existing objects
def clear_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

# Create a sphere at given position with color and size
def create_sphere(location, color, size=0.05, name="Sphere"):
    bpy.ops.mesh.primitive_uv_sphere_add(radius=size, location=location)
    sphere = bpy.context.active_object
    sphere.name = name
    material = bpy.data.materials.new(name=f"Material_{name}")
    material.diffuse_color = color + (1.0,)  # Add alpha
    sphere.data.materials.append(material)
    return sphere

# Create a line between two points
def create_line(start, end, color=(0, 1, 0), thickness=0.01, name="Line"):
    bm = bmesh.new()
    v1 = bm.verts.new(start)
    v2 = bm.verts.new(end)
    bm.edges.new((v1, v2))
    mesh = bpy.data.meshes.new(name)
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    material = bpy.data.materials.new(name=f"Material_{name}")
    material.diffuse_color = color + (1.0,)
    obj.data.materials.append(material)
    # Add thickness
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_add(type='SKIN')
    obj.modifiers["Skin"].use_smooth_shade = True
    for vert in obj.data.skin_vertices[0].data:
        vert.radius = (thickness, thickness)
    bpy.ops.object.modifier_apply(modifier="Skin")
    return obj

# Fit a circle through 3D points and create a curve in Blender
def create_fitted_circle(points, color, name="Circle", width=0.01, alpha=0.5):
    points = np.array(points)
    # Ensure we have enough points
    if len(points) < 3:
        print(f"Warning: Fiber {name} has fewer than 3 points, skipping.")
        return None

    # Calculate center and normal using SVD
    mean = np.mean(points, axis=0)
    centered = points - mean
    u, s, vh = np.linalg.svd(centered)
    normal = vh[-1]

    # Project points to the plane
    u = vh[0]
    v = vh[1]
    proj = np.dot(centered, np.array([u, v]).T)

    # Fit circle in 2D
    x = proj[:, 0]
    y = proj[:, 1]
    A = np.vstack([x, y, np.ones(len(x))]).T
    b = x**2 + y**2
    coeffs = np.linalg.lstsq(A, b, rcond=None)[0]
    xc, yc = coeffs[0]/2, coeffs[1]/2
    radius = np.sqrt(coeffs[2] + xc**2 + yc**2)

    # Convert back to 3D
    center = mean + xc*u + yc*v

    # Create circle in Blender
    curve = bpy.data.curves.new(name, 'CURVE')
    curve.dimensions = '3D'
    spline = curve.splines.new('BEZIER')

    # Create points for circle
    spline.bezier_points.add(n_points - 1)
    for i in range(n_points):
        theta = 2 * math.pi * i / (n_points - 1)
        point_2d = np.array([radius * math.cos(theta), radius * math.sin(theta)])
        point_3d = center + point_2d[0] * u + point_2d[1] * v
        spline.bezier_points[i].co = point_3d
        spline.bezier_points[i].handle_left = point_3d
        spline.bezier_points[i].handle_right = point_3d

    obj = bpy.data.objects.new(name, curve)
    bpy.context.collection.objects.link(obj)
    material = bpy.data.materials.new(name=f"Material_{name}")
    material.diffuse_color = color + (alpha,)
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    nodes.clear()
    bsdf = nodes.new(type='ShaderNodeBsdfPrincipled')
    output = nodes.new(type='ShaderNodeOutputMaterial')
    bsdf.inputs['Base Color'].default_value = color + (alpha,)
    bsdf.inputs['Alpha'].default_value = alpha
    material.blend_method = 'BLEND'
    links.new(bsdf.outputs['BSDF'], output.inputs['Surface'])
    obj.data.materials.append(material)
    # Add thickness
    obj.data.bevel_depth = width
    return obj

# Create a single surface spanning all fibers
def create_spanning_surface(fibers, num_fibers, width=0.01, name="SpanningSurface", alpha=0.5):
    if num_fibers < 2:
        print(f"Warning: Fewer than 2 fibers, cannot create spanning surface.")
        return None

    # Determine the number of points per fiber (use minimum to ensure consistency)
    points_per_fiber = min(len(fiber_points) for fiber_points in fibers.values())
    if points_per_fiber < 3:
        print(f"Warning: Fibers have fewer than 3 points, cannot create surface.")
        return None

    # Resample points to ensure each fiber has the same number of points
    resampled_points = []
    for fiber_points in fibers.values():
        fiber_points = np.array(fiber_points)
        # If necessary, resample to points_per_fiber points
        if len(fiber_points) != points_per_fiber:
            # Interpolate points along the fiber
            t = np.linspace(0, 1, len(fiber_points))
            t_new = np.linspace(0, 1, points_per_fiber)
            x = np.interp(t_new, t, fiber_points[:, 0])
            y = np.interp(t_new, t, fiber_points[:, 1])
            z = np.interp(t_new, t, fiber_points[:, 2])
            fiber_points = np.vstack((x, y, z)).T
        resampled_points.append(fiber_points)

    # Create mesh
    mesh = bpy.data.meshes.new(name)
    bm = bmesh.new()

    # Add vertices (grid: num_fibers x points_per_fiber)
    vertices = []
    for i in range(num_fibers):
        fiber_points = resampled_points[i]
        for j in range(points_per_fiber):
            vert = bm.verts.new(fiber_points[j])
            vertices.append(vert)

    # Create faces (connect vertices in a grid)
    for i in range(num_fibers - 1):
        for j in range(points_per_fiber):
            j_next = (j + 1) % points_per_fiber  # Wrap around for closed fibers
            v1 = vertices[i * points_per_fiber + j]
            v2 = vertices[i * points_per_fiber + j_next]
            v3 = vertices[(i + 1) * points_per_fiber + j_next]
            v4 = vertices[(i + 1) * points_per_fiber + j]
            bm.faces.new((v1, v2, v3, v4))

    # Update bmesh and create object
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)

    # Add vertex colors for gradient
    color_layer = mesh.vertex_colors.new(name="Col")
    for poly in mesh.polygons:
        for loop_idx in poly.loop_indices:
            loop = mesh.loops[loop_idx]
            vert_idx = loop.vertex_index
            fiber_idx = vert_idx // points_per_fiber
            hue = 0.66 - (0.66 / max(num_fibers - 1, 1)) * fiber_idx
            rgb = hsv_to_rgb(hue, 0.5, 1.0)
            color_layer.data[loop_idx].color = rgb + (alpha,)

    # Add material
    material = bpy.data.materials.new(name=f"Material_{name}")
    material.diffuse_color = (1.0, 1.0, 1.0, alpha)  # Base color with alpha
    material.use_nodes = True
    nodes = material.node_tree.nodes
    links = material.node_tree.links
    nodes.clear()
    vertex_color = nodes.new(type='ShaderNodeVertexColor')
    vertex_color.layer_name = "Col"
    bsdf = nodes.new(type='ShaderNodeBsdfPrincipled')
    output = nodes.new(type='ShaderNodeOutputMaterial')
    bsdf.inputs['Alpha'].default_value = alpha
    material.blend_method = 'BLEND'
    links.new(vertex_color.outputs['Color'], bsdf.inputs['Base Color'])
    links.new(bsdf.outputs['BSDF'], output.inputs['Surface'])
    obj.data.materials.append(material)

    # Add subdivision for smoothness
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_add(type='SUBSURF')
    obj.modifiers["Subdivision"].levels = 2
    obj.modifiers["Subdivision"].render_levels = 2

    return obj

def is_branch_point(point, connections):
    """Check if a point is a branch point (has more than 2 connections)"""
    return len(connections.get(point, set())) > 2

def merge_non_branch_edges(connections):
    """Merge edges that aren't at branch points into longer segments"""
    # Identify all branch points
    branch_points = {point for point, conns in connections.items() if is_branch_point(point, connections)}
    print(f"Debug: Found {len(branch_points)} branch points")

    # Create segments between branch points
    segments = []
    visited = set()

    # Process each branch point
    for branch_point in branch_points:
        # For each connection from this branch point
        for neighbor in connections[branch_point]:
            if (branch_point, neighbor) in visited or (neighbor, branch_point) in visited:
                continue

            # Start a new segment
            current = neighbor
            prev = branch_point
            segment = [branch_point, current]

            # Follow the path until we hit another branch point or a leaf
            while current not in branch_points:
                next_points = [n for n in connections[current] if n != prev]
                if not next_points:  # Leaf node, end of segment
                    break
                next_point = next_points[0]
                segment.append(next_point)
                visited.add((prev, current))
                visited.add((current, prev))
                prev, current = current, next_point

            # Add the final point if it's a branch point
            if current != branch_point and current in branch_points:
                segment.append(current)
                visited.add((prev, current))
                visited.add((current, prev))

            segments.append(segment)

    print(f"Debug: Created {len(segments)} segments")
    return segments

def setup_lights(center, distance, args):
    """Setup multiple lights in a spherical arrangement around the scene"""
    # Create main lights at cardinal points
    light_positions = [
        (distance, 0, 0),          # Right
        (-distance, 0, 0),         # Left
        (0, distance, 0),          # Front
        (0, -distance, 0),         # Back
        (0, 0, distance),          # Top
        (0, 0, -distance),         # Bottom
        # Add diagonal lights for better coverage
        (distance, distance, distance),
        (-distance, -distance, -distance),
        (distance, -distance, distance),
        (-distance, distance, -distance)
    ]

    lights = []
    for i, pos in enumerate(light_positions):
        bpy.ops.object.light_add(type='SUN', radius=1, location=center + np.array(pos))
        light = bpy.context.active_object
        light.name = f"Light_{i}"
        # Adjust energy based on position
        if i < 6:  # Main cardinal lights
            light.data.energy = args.main_light_energy
        else:  # Diagonal fill lights
            light.data.energy = args.fill_light_energy
        lights.append(light)

    return lights

def setup_camera(points, args):
    """Setup camera to view the entire structure"""
    # Calculate bounding box
    all_points = np.array(list(points.values()))
    min_coords = np.min(all_points, axis=0)
    max_coords = np.max(all_points, axis=0)
    center = (min_coords + max_coords) / 2

    print(f"Debug: Scene bounds - min: {min_coords}, max: {max_coords}")
    print(f"Debug: Scene center: {center}")

    # Calculate the size of the bounding box
    size = max_coords - min_coords
    max_size = np.max(size)

    # Position camera at a distance proportional to the size
    distance = max_size * args.camera_distance

    # Create camera
    bpy.ops.object.camera_add(enter_editmode=False, align='VIEW', location=(0, 0, 0))
    camera = bpy.context.active_object
    camera.name = "MainCamera"

    # Position camera at an angle that shows the structure well
    camera.location = center + np.array([distance, -distance, distance * args.camera_height])
    print(f"Debug: Camera location: {camera.location}")

    # Point camera at center
    direction = center - camera.location
    # Convert numpy array to Blender Vector
    direction_vec = mathutils.Vector(direction)
    rot_quat = direction_vec.to_track_quat('-Z', 'Y')
    camera.rotation_euler = rot_quat.to_euler()
    print(f"Debug: Camera rotation: {camera.rotation_euler}")

    # Set as active camera
    bpy.context.scene.camera = camera

    # Adjust camera settings for better view
    camera.data.lens = args.camera_lens
    camera.data.clip_start = 0.1
    camera.data.clip_end = 1000.0

    # Setup lights
    setup_lights(center, distance, args)

    # Set render settings
    bpy.context.scene.render.engine = 'CYCLES'
    bpy.context.scene.cycles.device = 'GPU'
    # Set view transform to Standard to avoid Filmic warning
    bpy.context.scene.view_settings.view_transform = 'Standard'

    # Set viewport to use the camera
    for area in bpy.context.screen.areas:
        if area.type == 'VIEW_3D':
            area.spaces[0].region_3d.view_perspective = 'CAMERA'
            break

    # Add a constraint to keep the camera looking at the center
    constraint = camera.constraints.new(type='TRACK_TO')
    constraint.target = bpy.data.objects.new("CameraTarget", None)
    bpy.context.collection.objects.link(constraint.target)
    constraint.target.location = center
    constraint.track_axis = 'TRACK_NEGATIVE_Z'
    constraint.up_axis = 'UP_Y'

    return camera

# Main visualization function
def visualize_hopf_fibration(file_path):
    # Parse the .dat file
    so3_tree, fibers = parse_dat_file(file_path)

    # Get command-line arguments
    args = parse_arguments()

    # Clear the scene
    clear_scene()

    # Create start and goal spheres for so3_tree
    create_sphere(so3_tree['start'], (0, 1, 0), 0.02, "Start")
    if so3_tree['goal']:
        create_sphere(so3_tree['goal'], (1, 0, 0), 0.02, "Goal")

    # Collect all points and their connections from so3_tree
    points = {}
    connections = {}
    # Add start and goal points
    points['start'] = np.array(so3_tree['start'])
    points['goal'] = np.array(so3_tree['goal'])
    # Process edges to build point connections
    for edge in so3_tree['edges']:
        source_arr = np.array(edge['source'])
        target_arr = np.array(edge['target'])
        if np.allclose(source_arr, target_arr):
            continue  # Skip zero-length edge
        source = tuple(edge['source'])
        target = tuple(edge['target'])
        if source not in points:
            points[source] = source_arr
        if target not in points:
            points[target] = target_arr
        if source not in connections:
            connections[source] = set()
        if target not in connections:
            connections[target] = set()
        connections[source].add(target)
        connections[target].add(source)

    print(f"Debug: Original number of connections: {len(connections)}")

    # Merge non-branch edges into segments
    segments = merge_non_branch_edges(connections)
    print(f"Debug: Number of segments after merging: {len(segments)}")
    print(f"Debug: First few segments: {segments[:3]}")

    # Draw each segment as a single path
    for i, segment in enumerate(segments):
        # Skip segments that are too short
        if len(segment) < 2:
            continue

        # Subsample points for smoother curve
        if len(segment) > 2 * segment_smooth_stride:
            subsampled = [segment[0]]
            subsampled += [segment[j] for j in range(1, len(segment)-1) if j % segment_smooth_stride == 0]
            subsampled.append(segment[-1])
        else:
            subsampled = segment

        print(f"Debug: Drawing segment {i} with {len(subsampled)} points")

        # Compute local normal using PCA/SVD
        seg_points = np.array([points[pt] for pt in subsampled])
        mean = np.mean(seg_points, axis=0)
        centered = seg_points - mean
        u, s, vh = np.linalg.svd(centered)
        normal = vh[-1]

        # Draw only once (no duplication)
        curve = bpy.data.curves.new(f"EdgeSegment_{i}", 'CURVE')
        curve.dimensions = '3D'
        curve.resolution_u = 24
        spline = curve.splines.new('BEZIER')
        n_points = len(subsampled)
        spline.bezier_points.add(n_points - 1)
        for j, pt in enumerate(subsampled):
            pt_offset = points[pt].copy()
            spline.bezier_points[j].co = pt_offset
            spline.bezier_points[j].handle_left = points[pt]
            spline.bezier_points[j].handle_right = points[pt]
            spline.bezier_points[j].handle_left_type = 'AUTO'
            spline.bezier_points[j].handle_right_type = 'AUTO'
        obj = bpy.data.objects.new(f"EdgeSegment_{i}", curve)
        bpy.context.collection.objects.link(obj)
        material = bpy.data.materials.new(name=f"Material_EdgeSegment_{i}")
        material.diffuse_color = (0, 1, 0, 1.0)
        obj.data.materials.append(material)
        obj.data.bevel_depth = edge_width
        obj.data.bevel_resolution = 3

    # Create fibers with color gradient from light blue to light red
    num_fibers = len(fibers)
    print(f"Visualizing {num_fibers} fibers.")

    # Collect all points for camera setup
    all_points = points.copy()
    fiber_colors = {}
    for i, (fiber_name, fiber_points) in enumerate(fibers.items()):
        # Calculate color (hue from blue 0.66 to red 0.0)
        hue = 0.66 - (0.66 / max(num_fibers - 1, 1)) * i
        rgb = hsv_to_rgb(hue, 0.5, 1.0)  # Light colors with full value
        # Create fiber circle
        create_fitted_circle(fiber_points, rgb, fiber_name, width=args.fiber_width, alpha=args.alpha)
        fiber_colors[fiber_name] = rgb
        # Add fiber points to all_points for camera setup
        for j, pt in enumerate(fiber_points):
            all_points[f"fiber_{fiber_name}_{j}"] = np.array(pt)

    # Create single spanning surface
    #if fibers:
        #create_spanning_surface(fibers, num_fibers, width=args.fiber_width * 2, alpha=args.alpha)

    # Setup camera to view the entire structure
    camera = setup_camera(all_points, args)
    print("Debug: Camera setup complete")

# Run the visualization with command-line argument
if __name__ == "__main__":
    args = parse_arguments()
    print(f"Loading file: {args.dat_file}")
    visualize_hopf_fibration(args.dat_file)
