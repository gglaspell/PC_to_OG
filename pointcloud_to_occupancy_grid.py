"""
Point Cloud to 2D Occupancy Grid Converter
Optimized for uneven terrain, slopes, and ramps.

HYBRID APPROACH:
- Uses ray-casting to generate FREE space (white).
- Uses direct insertion to guarantee OCCUPIED space (black).
- Leaves unobserved areas as UNKNOWN (gray).
"""
import argparse
import sys
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed

import numpy as np
import open3d as o3d
import pyoctomap
import yaml
from PIL import Image
from scipy.interpolate import griddata


def load_point_cloud_file(file_path):
    """
    Loads a point cloud from file formats: PCD, PLY, LAS, LAZ.
    """
    file_path = Path(file_path)
    if not file_path.exists():
        print(f"❌ Error: File not found: {file_path}")
        return None
    try:
        if file_path.suffix.lower() in ['.pcd', '.ply']:
            pcd = o3d.io.read_point_cloud(str(file_path))
            return np.asarray(pcd.points) if pcd.has_points() else None
        elif file_path.suffix.lower() in ['.las', '.laz']:
            import laspy
            with laspy.open(file_path) as f:
                points = np.vstack((f.x, f.y, f.z)).transpose()
            return points if len(points) > 0 else None
        else:
            print(f"❌ Error: Unsupported file format '{file_path.suffix}'")
            return None
    except Exception as e:
        print(f"❌ Error loading point cloud: {e}")
        return None


def load_point_cloud_from_bag(bag_path, topic_name, world_frame, sensor_frame):
    """
    Loads point clouds from a ROS 2 bag file.
    """
    try:
        import robotdatapy as rd
    except ImportError:
        print("❌ Error: 'robotdatapy' is not installed for ROS 2 bag support.")
        return None
    try:
        robot_data = rd.RobotData(bag_path)
        aggregated_points = []
        for _, points in robot_data.get_point_cloud_iterator(topic_name, world_frame, sensor_frame):
            if points is not None and len(points) > 0:
                aggregated_points.append(points)
        return np.vstack(aggregated_points) if aggregated_points else None
    except Exception as e:
        print(f"❌ Error loading ROS 2 bag: {e}")
        return None


def separate_ground_and_obstacles(points, slope_deg_threshold=10.0,
                                  normal_radius=0.2, downsample_voxel=0.05):
    """
    Separates point cloud into ground and obstacle points.
    """
    print("\n🔍 Separating ground from obstacles...")
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if downsample_voxel > 0:
        pcd_down = pcd.voxel_down_sample(voxel_size=downsample_voxel)
    else:
        pcd_down = pcd

    try:
        pcd_down.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=normal_radius, max_nn=30))
        pcd_down.orient_normals_to_align_with_direction([0., 0., 1.])
    except Exception as e:
        print(f"⚠️ Warning: Normal estimation failed: {e}. Falling back to elevation-only.")
        z_values = points[:, 2]
        z_threshold = np.percentile(z_values, 25)
        ground_mask = z_values <= z_threshold
        return points[ground_mask], points[~ground_mask]

    pcd_tree = o3d.geometry.KDTreeFlann(pcd_down)
    indices = np.array([pcd_tree.search_knn_vector_3d(pt, 1)[1][0] for pt in pcd.points])
    normals_full = np.asarray(pcd_down.normals)[indices]

    dot_products = np.abs(np.dot(normals_full, np.array([0, 0, 1])))
    angles = np.arccos(np.clip(dot_products, -1, 1))
    slope_rad_threshold = np.deg2rad(slope_deg_threshold)
    
    ground_mask = angles < slope_rad_threshold
    ground_points = np.asarray(pcd.points)[ground_mask]
    obstacle_points = np.asarray(pcd.points)[~ground_mask]

    print(f"   ✓ Ground: {len(ground_points)} points, Obstacles: {len(obstacle_points)} points")
    return ground_points, obstacle_points


def build_ground_height_map(ground_points, grid_resolution, bbx_min, x_size, y_size):
    """
    Creates a 2D grid representing the Z-height of the ground.
    """
    print("\n📊 Building ground height map...")
    if len(ground_points) == 0:
        return np.zeros((y_size, x_size), dtype=np.float64)

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
    
    valid_points = np.where(counts > 0)
    if len(valid_points[0]) < 3:
        return np.full((y_size, x_size), ground_points[:, 2].min(), dtype=np.float64)

    valid_values = avg_z[valid_points]
    grid_x, grid_y = np.mgrid[0:y_size, 0:x_size]

    filled_map = griddata((valid_points[0], valid_points[1]), valid_values, (grid_x, grid_y), method='nearest')
    print("   ✓ Ground height map complete.")
    return filled_map


def create_occupancy_grid_parallel(obstacle_tree, ground_height_map,
                                   grid_resolution, bbx_min, relative_z_min,
                                   relative_z_max, num_workers=4):
    """
    Creates 2D occupancy grid using the hybrid OcTree.
    """
    print("\n🔲 Generating 2D occupancy grid...")
    y_size, x_size = ground_height_map.shape
    occupancy_grid = np.full((y_size, x_size), 127, dtype=np.uint8)  # Unknown

    def process_cell(j, i):
        local_ground_z = ground_height_map[j, i]
        if np.isnan(local_ground_z): return (j, i, 127)

        world_x = bbx_min[0] + (i + 0.5) * grid_resolution
        world_y = bbx_min[1] + (j + 0.5) * grid_resolution
        z_slice = np.linspace(local_ground_z + relative_z_min, local_ground_z + relative_z_max, 7)

        is_occupied = False
        is_free = False

        for z in z_slice:
            node = obstacle_tree.search(np.array([world_x, world_y, z], dtype=float))
            if node is not None:
                if node.getOccupancy() >= obstacle_tree.getOccupancyThres():
                    is_occupied = True
                    break  # Occupied takes precedence
                else:
                    is_free = True

        if is_occupied:
            return (j, i, 0)      # Occupied (black)
        elif is_free:
            return (j, i, 254)    # Free (white)
        else:
            return (j, i, 127)    # Unknown (gray)

    with ThreadPoolExecutor(max_workers=num_workers) as executor:
        futures = [executor.submit(process_cell, j, i) for j in range(y_size) for i in range(x_size)]
        for i, future in enumerate(as_completed(futures)):
            j, i_cell, value = future.result()
            occupancy_grid[j, i_cell] = value
            if (i + 1) % 1000 == 0: print(f"   Progress: {((i + 1) / (x_size*y_size)) * 100:.1f}%", end='\r')
    
    print("\n   ✓ Grid generation complete!")
    return np.flipud(occupancy_grid)


def save_map_and_yaml(grid, yaml_data, output_path):
    """
    Saves the occupancy grid as PGM and YAML metadata.
    """
    output_path = Path(output_path)
    pgm_path = output_path.with_suffix('.pgm')
    yaml_path = output_path.with_suffix('.yaml')
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        Image.fromarray(grid, 'L').save(str(pgm_path))
        yaml_data['image'] = str(pgm_path.name)
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, sort_keys=False)
        print(f"\n💾 Saved map to {pgm_path} and {yaml_path}")
        return True
    except Exception as e:
        print(f"❌ Error saving files: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Convert 3D point cloud to a 2D occupancy grid using a hybrid method.")
    # ... (Argument parsing remains the same) ...
    parser.add_argument("input", type=str, help="Input file (PCD, PLY, LAS, LAZ) or ROS 2 bag directory")
    parser.add_argument("output", type=str, help="Output path (without extension)")
    parser.add_argument("--use_bag", action="store_true", help="Treat input as ROS 2 bag directory")
    parser.add_argument("--topic", type=str, default="/dlio/odom_node/pointcloud/deskewed", help="Point cloud topic name for ROS bag")
    parser.add_argument("--world_frame", type=str, default="odom", help="Fixed world frame for ROS bag")
    parser.add_argument("--sensor_frame", type=str, default="lidar", help="Sensor frame name for ROS bag")
    parser.add_argument("--octree_res", type=float, default=0.1, help="Resolution of the internal 3D octree")
    parser.add_argument("--grid_res", type=float, default=0.05, help="Resolution of the final 2D grid")
    parser.add_argument("--slope_deg", type=float, default=15.0, help="Maximum angle in degrees to be considered ground")
    parser.add_argument("--z_min", type=float, default=0.1, help="Minimum Z offset from the ground to check for obstacles")
    parser.add_argument("--z_max", type=float, default=2.0, help="Maximum Z offset from the ground to check for obstacles")
    parser.add_argument("--normal_radius", type=float, default=0.2, help="Radius for normal estimation during ground separation")
    parser.add_argument("--downsample", type=float, default=0.05, help="Voxel size for downsampling before normal estimation (0 to disable)")
    parser.add_argument("--workers", type=int, default=4, help="Number of parallel worker threads for grid generation")
    args = parser.parse_args()

    points = load_point_cloud_file(args.input) if not args.use_bag else \
             load_point_cloud_from_bag(args.input, args.topic, args.world_frame, args.sensor_frame)

    if points is None:
        print("❌ Failed to load point cloud. Aborting.")
        return False

    ground_points, obstacle_points = separate_ground_and_obstacles(
        points, args.slope_deg, args.normal_radius, args.downsample)

    # --- HYBRID OCTREE CONSTRUCTION ---
    print("\n🌳 Building Hybrid 3D OcTree...")
    obstacle_tree = pyoctomap.OcTree(args.octree_res)

    if len(obstacle_points) > 0:
        # Estimate a plausible sensor origin for ray-casting.
        # This is a heuristic and may not be perfect for all datasets.
        sensor_origin = np.mean(points, axis=0)
        sensor_origin[2] = points[:, 2].min()
        print(f"   Estimated sensor origin for ray-casting: {np.round(sensor_origin, 2)}")

        # Step 1: Perform ray-casting to create FREE space.
        print(f"   Step 1/2: Ray-casting {len(obstacle_points)} points to create free space...")
        for pt in obstacle_points:
            obstacle_tree.addPointWithRayCasting(sensor_origin.astype(float), pt.astype(float))

        # Step 2: Directly insert obstacle points to GUARANTEE occupied space.
        print(f"   Step 2/2: Reinforcing {len(obstacle_points)} obstacle locations...")
        for pt in obstacle_points:
            obstacle_tree.updateNode(pt.astype(float), True)
    
    print(f"✓ Hybrid OcTree created with {obstacle_tree.size()} nodes.")
    # --- END HYBRID CONSTRUCTION ---

    bbx_min = points.min(axis=0)
    bbx_max = points.max(axis=0)
    x_size = int(np.ceil((bbx_max[0] - bbx_min[0]) / args.grid_res))
    y_size = int(np.ceil((bbx_max[1] - bbx_min[1]) / args.grid_res))
    print(f"\n📐 Grid dimensions: {x_size} × {y_size} cells")

    ground_height_map = build_ground_height_map(
        ground_points, args.grid_res, bbx_min, x_size, y_size)

    occupancy_grid = create_occupancy_grid_parallel(
        obstacle_tree, ground_height_map, args.grid_res, bbx_min,
        args.z_min, args.z_max, args.workers)

    yaml_data = {
        'image': '', 'resolution': args.grid_res, 'negate': 0,
        'origin': [float(bbx_min[0]), float(bbx_min[1]), 0.0],
        'occupied_thresh': 0.65, 'free_thresh': 0.25
    }

    if save_map_and_yaml(occupancy_grid, yaml_data, args.output):
        print("\n✓ Conversion complete!")
        return True
    else:
        print("\n❌ Failed to save output files.")
        return False


if __name__ == "__main__":
    if main():
        sys.exit(0)
    else:
        sys.exit(1)
