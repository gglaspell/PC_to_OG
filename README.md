# PC_to_OG

<img width="278" height="267" alt="Screenshot from 2026-01-28 14-30-00" src="https://github.com/user-attachments/assets/547a2fb6-cb10-4885-9190-b58bef0272c1" />

<img width="278" height="267" alt="Screenshot from 2026-03-02 11-47-36" src="https://github.com/user-attachments/assets/d5528e0d-bf4a-46e9-9680-89e1274269f6" />


## ⚡ Fastest Way to Get Started

```bash
# 1. Create folder and files
mkdir PC_to_OG && cd PC_to_OG

# Copy these files into the folder:
# - pointcloud_to_occupancy_grid.py
# - Dockerfile

# Create subdirectories
mkdir data output

# 2. Put your .ply, .pcd, .las, or .laz file in data/
cp /path/to/your/cloud.ply data/

# 3. Build Docker image (takes ~5-10 min first time)
docker build -t pointcloud-converter:latest .

# 4. Run conversion
docker run --rm \
  -v "$(pwd)/data:/data" \
  -v "$(pwd)/output:/output" \
  pointcloud-converter:latest \
  /data/cloud.ply /output/my_map

# 5. Check results
ls output/
# → my_map.pgm (your occupancy grid image)
# → my_map.yaml (ROS 2 config file)
```

---

## 📂 FINAL PROJECT STRUCTURE

Organize files and folders like this:

```
my_pointcloud_project/
├── pointcloud_to_occupancy_grid.py     ← From PYTHON_AND_DOCKERFILE.md
├── Dockerfile                           ← From PYTHON_AND_DOCKERFILE.md
├── README.md                           ← Documentation
├── data/                               ← Create this folder (your point cloud files go here)
└── output/                             ← Create this folder (results will be saved here)
```

---

## ✅ Verification

After setup, you should be able to:

1. ✓ Build Docker image without errors
2. ✓ Run conversion on any `.ply`, `.pcd`, `.las`, or `.laz` file
3. ✓ Get `.pgm` (image) and `.yaml` (config) files

---

## 🎯 Common Use Cases

### Case 1: Simple Indoor Map
```bash
docker run --rm -v "$(pwd)/data:/data" -v "$(pwd)/output:/output" \
  pointcloud-converter:latest \
  /data/room.ply /output/room_map
```

### Case 2: Outdoor with Slopes
```bash
docker run --rm -v "$(pwd)/data:/data" -v "$(pwd)/output:/output" \
  pointcloud-converter:latest \
  /data/terrain.las /output/terrain_map \
  --slope_deg 15 --z_min 0.05 --z_max 2.0
```

### Case 3: Steep Ramp or Stairs
```bash
docker run --rm -v "$(pwd)/data:/data" -v "$(pwd)/output:/output" \
  pointcloud-converter:latest \
  /data/ramp.pcd /output/ramp_map \
  --slope_deg 25 --normal_radius 0.4 --downsample 0.1
```

### Case 4: Very Large Point Cloud
```bash
docker run --rm -v "$(pwd)/data:/data" -v "$(pwd)/output:/output" \
  pointcloud-converter:latest \
  /data/huge.laz /output/large_map \
  --octree_res 0.2 --workers 8
```

### Case 5: Noisy Scan (filter small / isolated obstacles)
```bash
docker run --rm -v "$(pwd)/data:/data" -v "$(pwd)/output:/output" \
  pointcloud-converter:latest \
  /data/noisy_scan.ply /output/clean_map \
  --min_cluster_size 50 --cluster_eps 0.25
```

---
## 🔧 All Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| --octree_res | 0.1 | Resolution of the internal 3D octree (metres)|
| --grid_res | 0.05 | Resolution of the final 2D grid (metres/cell) |
| --slope_deg | 15.0 | Max surface angle (°) to be classified as ground |
| --z_min | 0.1 | Min Z offset above ground to check for obstacles |
| --z_max | 2.0 | Max Z offset above ground to check for obstacles |
| --normal_radius | 0.2 | Radius for surface normal estimation |
| --downsample | 0.05 | Voxel size for downsampling (0 to disable) |
| --workers | 4 | Parallel worker threads for grid generation |
| --min_cluster_size | 30 | Min points per obstacle cluster; smaller clusters are removed as noise (0 to disable) |
| --cluster_eps | 0.2 | DBSCAN neighbourhood radius in metres for obstacle clustering |

---

## 🐛 Troubleshooting Quick Links

- **"Permission denied" when building** → Run with `sudo docker`
- **Docker image not found** → Make sure you built it first
- **Output is all black** → Increase `--z_max`
- **Output is all white** → Decrease `--slope_deg`
- **"robotdatapy not installed"** → Use standalone `.ply` file instead
- **Process runs out of memory** → Increase `--octree_res`

---

## 🆘 Need Help?

**Common Questions:**

**Q: Where do I put the Python script and Dockerfile?**  
A: Both go in the same folder (your project root).

**Q: What if the map looks wrong?**  
A: Most issues are parameter-related.

**Q: Which file do I edit?**  
A: Only `pointcloud_to_occupancy_grid.py`. `Dockerfile` should work as-is.

**Q: Can I use my own point cloud file?**  
A: Yes! Supports .pcd, .ply, .las, .laz. Put in `data/` folder.

**Q: Does it work with ROS 2?**  
A: Yes! Output is ROS 2 Nav2 compatible.

---


## 🛠️ System Requirements

**To Run:**
- Docker (any recent version)
- ~2GB disk space for image
- Point cloud file in PCD, PLY, LAS, or LAZ format

**To Develop:**
- Python 3.8+
- Same libraries as Dockerfile (numpy, pyoctomap, open3d, etc.)

---

## 📊 Performance Benchmarks

| Scenario | Typical Runtime |
|----------|-----------------|
| Small room (100K points) | ~30 seconds |
| Medium warehouse (1M points) | ~2 minutes |
| Large outdoor (10M points) | ~15 minutes |
| Dense building (100M points) | ~2 hours |

*Times assume 4-core CPU, 8GB RAM. Parallelization scales with CPU cores.*

---

## 🤖 ROS 2 Integration

After generating the map:

```bash
# Launch Nav2 with your new map
ros2 launch nav2_bringup navigation_launch.py \
  map:=$(pwd)/output/my_map.yaml
```

The `.yaml` file contains the path to `.pgm` image and metadata for ROS 2.

---

