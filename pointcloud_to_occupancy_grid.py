"""
Point Cloud to 2D Occupancy Grid Converter

Optimized for uneven terrain, slopes, and ramps.

Supports:
- ROS 2 bag files (using rosbag2)
- Standalone point cloud files (PCD, PLY, LAS, LAZ)

Features:
- Ground/obstacle separation using surface normals + elevation
- Relative Z-slicing for terrain adaptation
- Parallel processing for performance
- Comprehensive error handling
"""

import argparse
import numpy as np
import pyoctomap
import yaml
import open3d as o3d
from PIL import Image
from scipy.interpolate import griddata
from concurrent.futures import ThreadPoolExecutor, as_completed
import sys
from pathlib import Path


def load_point_cloud_file(file_path):
    """
    Loads a point cloud from file formats: PCD, PLY, LAS, LAZ.

    Args:
        file_path (str): Path to the point cloud file.

    Returns:
        np.ndarray: Nx3 array of points, or None if loading fails.
    """
    file_path = Path(file_path)

    if not file_path.exists():
        print(f"❌ Error: File not found: {file_path}")
        return None

    try:
        if file_path.suffix.lower() in ['.pcd', '.ply']:
            print(f"📖 Loading {file_path.suffix} file with Open3D...")
            pcd = o3d.io.read_point_cloud(str(file_path))
            if len(pcd.points) == 0:
                print("❌ Error: Point cloud file is empty")
                return None
            return np.asarray(pcd.points)

        elif file_path.suffix.lower() in ['.las', '.laz']:
            print(f"📖 Loading {file_path.suffix} file with laspy...")
            import laspy
            with laspy.open(file_path) as f:
                points = np.vstack((f.x, f.y, f.z)).transpose()
            if len(points) == 0:
                print("❌ Error: Point cloud file is empty")
                return None
            return points

        else:
            print(f"❌ Error: Unsupported file format '{file_path.suffix}'")
            print("   Supported: .pcd, .ply, .las, .laz")
            return None

    except Exception as e:
        print(f"❌ Error loading point cloud: {e}")
        return None


def load_point_cloud_from_bag(bag_path, topic_name, world_frame, sensor_frame):
    """
    Loads point clouds from a ROS 2 bag file.

    Args:
        bag_path (str): Path to the ROS 2 bag directory.
        topic_name (str): Point cloud topic to extract.
        world_frame (str): Fixed frame (e.g., 'odom', 'map').
        sensor_frame (str): Sensor frame name.

    Returns:
        np.ndarray: Combined Nx3 array of all points from all messages.
    """
    try:
        import robotdatapy as rd
    except ImportError:
        print("❌ Error: 'robotdatapy' is not installed.")
        print("   To use ROS 2 bag files, install it:")
        print("   pip install robotdatapy")
        print("\n   Alternatively, use a standalone point cloud file (PCD, PLY, LAS, LAZ)")
        return None

    try:
        print(f"📖 Loading ROS 2 bag from: {bag_path}")
        robot_data = rd.RobotData(bag_path)

        aggregated_points = []
        point_cloud_iterator = robot_data.get_point_cloud_iterator(
            topic_name=topic_name,
            world_frame=world_frame,
            sensor_frame=sensor_frame
        )

        count = 0
        for timestamp, points in point_cloud_iterator:
            if points is None or len(points) == 0:
                continue
            aggregated_points.append(points)
            count += 1
            print(f" ✓ Cloud #{count}: {len(points)} points")

        if not aggregated_points:
            print(f"❌ Error: No valid point clouds found on topic '{topic_name}'")
            return None

        full_cloud = np.vstack(aggregated_points)
        print(f"✓ Total aggregated points: {len(full_cloud)}")
        return full_cloud

    except Exception as e:
        print(f"❌ Error loading ROS 2 bag: {e}")
        return None


def separate_ground_and_obstacles(points, slope_deg_threshold=10.0,
                                  normal_radius=0.2, downsample_voxel=0.05):
    """
    Separates point cloud into ground and obstacle points.
    """
    print("\n🔍 Separating ground from obstacles...")
    print(f"   Slope threshold: {slope_deg_threshold}°")
    print(f"   Normal estimation radius: {normal_radius}m")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if downsample_voxel > 0:
        print(f"   Downsampling to {downsample_voxel}m voxels for normal estimation...")
        pcd_down = pcd.voxel_down_sample(voxel_size=downsample_voxel)
    else:
        pcd_down = pcd

    print(f"   Estimating normals (working on {len(pcd_down.points)} points)...")

    try:
        pcd_down.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=normal_radius,
                max_nn=30
            )
        )
        pcd_down.orient_normals_to_align_with_direction(
            orientation_reference=np.array([0., 0., 1.])
        )
    except Exception as e:
        print(f"⚠️ Warning: Normal estimation failed: {e}")
        print("   Falling back to elevation-only classification")
        z_values = points[:, 2]
        z_threshold = np.percentile(z_values, 25)
        ground_mask = z_values <= z_threshold
        return points[ground_mask], points[~ground_mask]

    normals_down = np.asarray(pcd_down.normals)
    points_down = np.asarray(pcd_down.points)

    z_axis = np.array([0, 0, 1])
    dot_products = np.dot(normals_down, z_axis)
    dot_products = np.clip(np.abs(dot_products), -1, 1)
    angles = np.arccos(dot_products)
    slope_rad_threshold = np.deg2rad(slope_deg_threshold)

    angle_based_ground_mask = angles < slope_rad_threshold

    z_values_down = points_down[:, 2]
    z_ground_threshold = np.percentile(z_values_down, 10)
    z_tolerance = 0.3
    elevation_based_mask = z_values_down <= (z_ground_threshold + z_tolerance)

    ground_mask_down = angle_based_ground_mask | elevation_based_mask
    obstacle_mask_down = ~ground_mask_down

    ground_points_down = points_down[ground_mask_down]
    obstacle_points_down = points_down[obstacle_mask_down]

    print(f"   ✓ Ground points (downsampled): {len(ground_points_down)}")
    print(f"   ✓ Obstacle points (downsampled): {len(obstacle_points_down)}")

    # Return the original, full-resolution points
    # Find original points that are close to the downsampled ground/obstacle points
    # This is a simplified way; a more robust method would use KDTree lookups
    # For now, we return the downsampled points as they are sufficient for the next steps
    return ground_points_down, obstacle_points_down


def build_ground_height_map(ground_points, grid_resolution, bbx_min,
                            x_size, y_size):
    """
    Creates a 2D grid representing the Z-height of the ground.
    """
    print("\n📊 Building ground height map...")

    sum_z = np.zeros((y_size, x_size), dtype=np.float64)
    counts = np.zeros((y_size, x_size), dtype=int)

    for pt in ground_points:
        grid_x = int((pt[0] - bbx_min[0]) / grid_resolution)
        grid_y = int((pt[1] - bbx_min[1]) / grid_resolution)
        if 0 <= grid_x < x_size and 0 <= grid_y < y_size:
            sum_z[grid_y, grid_x] += pt[2]
            counts[grid_y, grid_x] += 1

    with np.errstate(divide='ignore', invalid='ignore'):
        avg_z = sum_z / counts

    cells_with_data = np.count_nonzero(counts)
    total_cells = x_size * y_size
    coverage = (cells_with_data / total_cells) * 100
    print(f"   Initial coverage: {coverage:.1f}% ({cells_with_data}/{total_cells} cells)")

    valid_points = np.where(counts > 0)

    if len(valid_points[0]) < 3:
        print(f"⚠️ Warning: Too few ground points ({len(valid_points[0])}) for interpolation")
        print("   Using global minimum as fallback")
        min_z = ground_points[:, 2].min()
        filled_ground_map = np.full((y_size, x_size), min_z, dtype=np.float64)
    else:
        valid_values = avg_z[valid_points]
        grid_x, grid_y = np.mgrid[0:y_size, 0:x_size]

        print(f"   Interpolating to fill {total_cells - cells_with_data} empty cells...")

        try:
            filled_ground_map = griddata(
                (valid_points[0], valid_points[1]),
                valid_values,
                (grid_x, grid_y),
                method='linear',
                fill_value=np.min(valid_values)
            )

            if np.any(np.isnan(filled_ground_map)):
                print("   Linear interpolation left NaNs, using nearest neighbor...")
                filled_ground_map = griddata(
                    (valid_points[0], valid_points[1]),
                    valid_values,
                    (grid_x, grid_y),
                    method='nearest'
                )

        except Exception as e:
            print(f"⚠️ Warning: Interpolation failed: {e}")
            print("   Using nearest neighbor method instead")
            filled_ground_map = griddata(
                (valid_points[0], valid_points[1]),
                valid_values,
                (grid_x, grid_y),
                method='nearest'
            )

    print("   ✓ Ground height map complete")
    return filled_ground_map

# --- FIX 2: REVISED OCCUPANCY GRID GENERATION LOGIC ---
# This function is updated to correctly handle three states:
# 0 (occupied), 254 (free), and 127 (unknown).
def create_occupancy_grid_parallel(obstacle_tree, ground_height_map,
                                   grid_resolution, bbx_min, relative_z_min,
                                   relative_z_max, num_workers=4):
    """
    Creates 2D occupancy grid using parallel processing for speed.
    """
    print("\n🔲 Generating 2D occupancy grid...")
    print(f"   Checking {relative_z_min}m to {relative_z_max}m above ground")
    print(f"   Using {num_workers} worker threads")

    y_size, x_size = ground_height_map.shape
    occupancy_grid = np.full((y_size, x_size), 127, dtype=np.uint8)

    def process_cell(j, i):
        """Process a single grid cell and returns its state."""
        local_ground_z = ground_height_map[j, i]
        if np.isnan(local_ground_z):
            return (j, i, 127)  # Unknown if ground height is unknown

        z_slice_min = local_ground_z + relative_z_min
        z_slice_max = local_ground_z + relative_z_max

        world_x = bbx_min[0] + (i + 0.5) * grid_resolution
        world_y = bbx_min[1] + (j + 0.5) * grid_resolution

        is_occupied = False
        is_known_free = False

        for z in np.linspace(z_slice_min, z_slice_max, 7):
            node = obstacle_tree.search(
                np.array([world_x, world_y, z], dtype=float)
            )

            if node is not None:
                occ = node.getOccupancy()
                thresh = obstacle_tree.getOccupancyThres()
                if occ >= thresh:
                    is_occupied = True
                    break  # Occupied takes precedence
                else:
                    is_known_free = True

        if is_occupied:
            return (j, i, 0)      # Occupied (black)
        elif is_known_free:
            return (j, i, 254)    # Free (white)
        else:
            return (j, i, 127)    # Unknown (gray)

    with ThreadPoolExecutor(max_workers=num_workers) as executor:
        futures = [executor.submit(process_cell, j, i) for j in range(y_size) for i in range(x_size)]

        completed = 0
        total = len(futures)
        for future in as_completed(futures):
            j, i, value = future.result()
            occupancy_grid[j, i] = value
            completed += 1
            if total > 0 and completed % max(1, total // 20) == 0:
                pct = (completed / total) * 100
                print(f"   Progress: {pct:.0f}%", end='\r')

    print("   ✓ Grid generation complete ")
    occupancy_grid = np.flipud(occupancy_grid)
    return occupancy_grid
# --- END FIX 2 ---


def save_map_and_yaml(grid, yaml_data, output_path):
    """
    Saves the occupancy grid as PGM image and YAML metadata.
    """
    output_path = Path(output_path)
    pgm_path = output_path.with_suffix('.pgm')
    yaml_path = output_path.with_suffix('.yaml')

    output_path.parent.mkdir(parents=True, exist_ok=True)

    print("\n💾 Saving output files...")

    try:
        print(f"   📄 Saving: {pgm_path}")
        img = Image.fromarray(grid, 'L')
        img.save(str(pgm_path))
    except Exception as e:
        print(f"❌ Error saving PGM: {e}")
        return False

    try:
        print(f"   📄 Saving: {yaml_path}")
        yaml_data['image'] = str(pgm_path.name) # Use relative path for portability
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, sort_keys=False, default_flow_style=False)
    except Exception as e:
        print(f"❌ Error saving YAML: {e}")
        return False

    print("✓ Files saved successfully!")
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Convert a 3D point cloud to 2D occupancy grid (optimized for uneven terrain)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
EXAMPLES:

Standalone point cloud file:
  python pointcloud_to_occupancy_grid.py data/my_cloud.ply output/my_map

High resolution on slopes:
  python pointcloud_to_occupancy_grid.py data/ramp.las output/ramp_map \
    --grid_res 0.05 --normal_radius 0.3 --slope_deg 25
"""
    )

    parser.add_argument(
        "input", type=str,
        help="Input file (PCD, PLY, LAS, LAZ) or ROS 2 bag directory"
    )
    parser.add_argument(
        "output", type=str,
        help="Output path (without extension)"
    )
    parser.add_argument(
        "--use_bag", action="store_true",
        help="Treat input as ROS 2 bag directory"
    )
    parser.add_argument(
        "--topic", type=str,
        default="/dlio/odom_node/pointcloud/deskewed",
        help="Point cloud topic name"
    )
    parser.add_argument(
        "--world_frame", type=str, default="odom",
        help="Fixed world frame"
    )
    parser.add_argument(
        "--sensor_frame", type=str, default="lidar",
        help="Sensor frame name"
    )
    parser.add_argument(
        "--octree_res", type=float, default=0.1,
        help="3D octree resolution"
    )
    parser.add_argument(
        "--grid_res", type=float, default=0.05,
        help="2D grid resolution"
    )
    parser.add_argument(
        "--slope_deg", type=float, default=10.0,
        help="Max slope angle for ground"
    )
    parser.add_argument(
        "--z_min", type=float, default=0.1,
        help="Minimum Z offset from ground"
    )
    parser.add_argument(
        "--z_max", type=float, default=1.5,
        help="Maximum Z offset from ground"
    )
    parser.add_argument(
        "--normal_radius", type=float, default=0.2,
        help="Radius for normal estimation"
    )
    parser.add_argument(
        "--downsample", type=float, default=0.05,
        help="Voxel size for downsampling"
    )
    parser.add_argument(
        "--workers", type=int, default=4,
        help="Number of worker threads"
    )

    args = parser.parse_args()

    if args.octree_res <= 0 or args.grid_res <= 0:
        print("❌ Error: Resolutions must be positive")
        return False

    if args.z_min >= args.z_max:
        print("❌ Error: z_min must be less than z_max")
        return False

    print("=" * 70)
    print(" Point Cloud to Occupancy Grid Converter (Improved v2)")
    print("=" * 70)

    print("\n📥 Loading input...")

    if args.use_bag:
        points = load_point_cloud_from_bag(
            args.input, args.topic, args.world_frame, args.sensor_frame
        )
    else:
        points = load_point_cloud_file(args.input)

    if points is None or len(points) == 0:
        print("❌ Failed to load point cloud")
        return False

    print(f"✓ Loaded {len(points):,} points")
    print(f" X range: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}]")
    print(f" Y range: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}]")
    print(f" Z range: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")

    ground_points, obstacle_points = separate_ground_and_obstacles(
        points,
        slope_deg_threshold=args.slope_deg,
        normal_radius=args.normal_radius,
        downsample_voxel=args.downsample
    )

    if len(ground_points) == 0:
        print("❌ No ground points detected - cannot create map")
        return False

    # --- FIX 1: CALCULATE A PLAUSIBLE SENSOR ORIGIN ---
    # Using a fixed (0,0,0) origin is incorrect if the point cloud is not
    # centered there. This heuristic estimates the origin as the centroid
    # of the point cloud, which is more robust for single static scans.
    # Note: This is still a simplification for aggregated clouds from a moving sensor.
    print("\n🤔 Estimating sensor origin for raycasting...")
    if points.shape[0] > 0:
        sensor_origin = np.mean(points, axis=0)
        sensor_origin[2] = points[:, 2].min()
        print(f"   ✓ Estimated sensor origin at: [{sensor_origin[0]:.2f}, {sensor_origin[1]:.2f}, {sensor_origin[2]:.2f}]")
    else:
        sensor_origin = np.array([0.0, 0.0, 0.0], dtype=float)
        print("   ⚠️ No points to estimate origin, using default (0,0,0).")
    # --- END FIX 1 ---

    print("\n🌳 Building 3D octree...")
    print(f"   Resolution: {args.octree_res}m")

    obstacle_tree = pyoctomap.OcTree(args.octree_res)

    if len(obstacle_points) > 0:
        # Use the estimated origin for more accurate raycasting.
        for pt in obstacle_points.astype(float):
            obstacle_tree.addPointWithRayCasting(
                sensor_origin,
                pt
            )

    print(f"✓ Octree created with {obstacle_tree.size():,} nodes")

    bbx_min = points.min(axis=0)
    bbx_max = points.max(axis=0)
    x_size = int(np.ceil((bbx_max[0] - bbx_min[0]) / args.grid_res))
    y_size = int(np.ceil((bbx_max[1] - bbx_min[1]) / args.grid_res))

    print(f"\n📐 Grid dimensions: {x_size} × {y_size} cells")

    ground_height_map = build_ground_height_map(
        ground_points, args.grid_res, bbx_min, x_size, y_size
    )

    occupancy_grid = create_occupancy_grid_parallel(
        obstacle_tree, ground_height_map, args.grid_res, bbx_min,
        args.z_min, args.z_max, num_workers=args.workers
    )

    yaml_data = {
        'image': '', # Will be filled in by save_map_and_yaml
        'resolution': args.grid_res,
        'origin': [float(bbx_min[0]), float(bbx_min[1]), 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25
    }

    success = save_map_and_yaml(occupancy_grid, yaml_data, args.output)

    if success:
        print("\n" + "=" * 70)
        print("✓ Conversion complete!")
        print("=" * 70)
        print("\n📍 Output files:")
        print(f" {Path(args.output).with_suffix('.pgm')}")
        print(f" {Path(args.output).with_suffix('.yaml')}")
        print("\n🤖 To use with ROS 2 Nav2:")
        print(f" ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={Path(args.output).with_suffix('.yaml')}")
        return True
    else:
        print("\n❌ Failed to save output files")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)

