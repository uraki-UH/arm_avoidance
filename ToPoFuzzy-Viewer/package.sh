#!/bin/bash

# Configuration
APP_NAME="ToPoFuzzyViewer_Source"
DATE=$(date +%Y%m%d)
PACKAGE_NAME="${APP_NAME}_${DATE}"
OUTPUT_DIR="packaged_dist"
STAGING_DIR="${OUTPUT_DIR}/${PACKAGE_NAME}"

# Create staging directory
rm -rf "$OUTPUT_DIR"
mkdir -p "$STAGING_DIR"

echo "Package Name: $PACKAGE_NAME"
echo "Staging Directory: $STAGING_DIR"

# Copy Backend Source
echo "Copying Backend..."
mkdir -p "$STAGING_DIR/backend"
# Copy src, but exclude any potential build artifacts if they exist inside src (unlikely but safe)
cp -r backend/src "$STAGING_DIR/backend/"
# Copy CMakeLists.txt if it exists in backend root (it usually does for colcon workspace level, but often it's just src)
# Checking project structure: usually backend/src contains packages.
# Let's copy backend/src and any other top-level config files if necessary. 
# Based on earlier `ls backend`: src, build, install, log. We only want src.

# Copy Frontend Source
echo "Copying Frontend..."
mkdir -p "$STAGING_DIR/frontend"
# Exclude node_modules, dist, etc.
rsync -av --progress frontend/ "$STAGING_DIR/frontend/" \
    --exclude node_modules \
    --exclude dist \
    --exclude .git \
    --exclude .env \
    --exclude .DS_Store

# Copy Common
echo "Copying Common protocols..."
cp -r common "$STAGING_DIR/"

# Create new README.md
echo "Generating README.md..."
cat > "$STAGING_DIR/README.md" <<EOF
# ToPoFuzzy Viewer & Editor (Source Package)

This package contains the source code for the ToPoFuzzy Viewer.

## Prerequisites

- **Node.js** (v16+)
- **ROS2 Humble**
- **C++ Compiler** (C++17 support)
- **CMake** (v3.16+)

## 1. Backend Setup

The backend is a ROS2 workspace.

\`\`\`bash
cd backend
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
\`\`\`

## 2. Frontend Setup

\`\`\`bash
cd frontend
npm install
\`\`\`

## 3. Running the Application

### Start Backend
\`\`\`bash
cd backend
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run topo_fuzzy_viewer pc_server
\`\`\`

### Start Frontend
\`\`\`bash
cd frontend
npm run dev
\`\`\`

Then open [http://localhost:5173](http://localhost:5173) in your browser.
EOF

# Zip the package
echo "Zipping..."
cd "$OUTPUT_DIR"
zip -r "../${PACKAGE_NAME}.zip" "$PACKAGE_NAME"
cd ..

# Cleanup
echo "Cleaning up..."
rm -rf "$OUTPUT_DIR"

echo "Done! Created ${PACKAGE_NAME}.zip"
