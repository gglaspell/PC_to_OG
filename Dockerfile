# Use Ubuntu 24.04 LTS as the base image
FROM ubuntu:24.04

# Prevent interactive prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system-level dependencies
# Install system-level dependencies
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-venv \
    build-essential \
    cmake \
    liboctomap-dev \
    libgl1 \
    libglx-mesa0 \
    libxext6 \
    && rm -rf /var/lib/apt/lists/*

# Upgrade pip and install Python dependencies
RUN python3 -m venv /venv && \
    /venv/bin/pip install --no-cache-dir --upgrade pip && \
    /venv/bin/pip install --no-cache-dir \
        numpy \
        pyoctomap \
        pyyaml \
        open3d \
        "laspy[lazrs]" \
        scipy \
        Pillow

ENV PATH="/venv/bin:${PATH}"

# Set working directory
WORKDIR /app

# Copy the Python script into the container
COPY pointcloud_to_occupancy_grid.py .

# Set the entrypoint to execute the script
ENTRYPOINT ["python3", "pointcloud_to_occupancy_grid.py"]

