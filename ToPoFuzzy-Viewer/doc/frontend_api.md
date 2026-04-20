# ToPoFuzzyViewer Frontend Architecture

This document provides an overview of the frontend architecture, including directory structure and the role of key files.

## Directory Structure

```
frontend/src/
├── components/         # Reusable UI components
│   └── ui/             # Core UI elements (Tabs, CollapsibleSection, etc.)
├── features/           # Feature-specific components
│   ├── analysis/       # Analysis tools (Zone monitoring)
│   ├── io/             # Input/Output (File loading, ROS integration)
│   ├── manipulation/   # Interaction tools (Clipping, Transformation, Selection)
│   └── visualization/  # Rendering components (Point clouds, GNG graph)
├── hooks/              # Custom React hooks (WebSocket, Selection, Clipping)
├── layout/             # Layout components (Sidebar, MainLayout)
├── types/              # TypeScript type definitions
├── utils/              # Utility functions and file loaders
├── App.tsx             # Main application entry point
└── main.tsx            # React DOM rendering
```

## Core Components

### App.tsx
The main application component that orchestrates:
- Global state management (point clouds, selected items)
- WebSocket connection initialization
- 3D Canvas setup with `@react-three/fiber`
- Integration of `Sidebar` and `MainLayout`
- Event handling for scene interactions

### Layout
- **SidebarContent.tsx**: Manages the content within the sidebar tabs. It orchestrates sub-components for layers, display settings, editing tools, and analysis features.
- **MainLayout.tsx**: Provides the structural frame for the application, handling the sidebar and main content area positioning.

## Features

### Visualization
- **PointCloudRenderer.tsx**: Renders point cloud data using Three.js points. Supports different visualization modes (RGB, Height Heatmap, etc.).
- **GraphRenderer.tsx**: Renders the GNG (Growing Neural Gas) topological graph, including nodes, edges, and clusters.
- **GngLayerControls.tsx**: Controls for the GNG layer, including visibility toggles for graph elements, clusters, normals, and label filtering.
- **HeatmapControls.tsx**: UI controls for configuring heatmap color schemes and value ranges.
- **NormalVectorRenderer.tsx**: Visualizes normal vectors for graph nodes/clusters.

### Manipulation
- **SelectionHandler.tsx**: Handles the logic for selecting points within the 3D scene using a selection rectangle.
- **TransformPanel.tsx**: Provides UI controls for translating, rotating, and scaling point clouds.
- **ClippingControls.tsx**: Manages clipping planes to slice the view of the point cloud.
- **SelectionToolbar.tsx**: Toolbar for selection-related actions (Select Rectangle, Delete, Clear).

### I/O (Input/Output)
- **SourceSelector.tsx**: Manages data sources (WebSocket connection, ROS topics).
- **ServerFileBrowser.tsx**: Interface for browsing and loading point cloud files from the server.
- **RosbagPlayer.tsx**: Controls for playing back ROS bag files.
- **ExportPanel.tsx**: Functionality to export point cloud data to various formats (PCD, PLY, LAS).

### Analysis
- **ZoneMonitorPanel.tsx**: UI for defining monitoring zones and displaying cluster counts within those zones.
- **ZoneVisualizer.tsx**: Renders the monitoring zone in the 3D scene.
- **useZoneMonitor.ts**: Hook managing the logic for zone creation and point containment checks.

## Key Hooks

### useWebSocket.ts
Manages the WebSocket connection to the backend. It provides:
- Real-time data streaming (PointCloud, Graph)
- RPC (Remote Procedure Call) methods for backend operations
- Connection state management

### usePointSelection.ts
Implements frustum-based point selection logic. It converts 2D screen coordinates from a selection rectangle into a 3D frustum to identify selected points.

### useClippingPlanes.ts
Manages the state and logic for global clipping planes used to inspect the interior of point clouds.

## Utilities

### Protocol (protocol.ts)
Handles the binary protocol for deserializing point cloud data transferred over WebSocket.

### File Loaders
- **lasLoader.ts**, **pcdLoader.ts**, **plyLoader.ts**, **landxmlLoader.ts**: Parsers for different point cloud file formats.

### Math & Helpers
- **bounds.ts**: Utility to calculate bounding boxes for point clouds.
- **transformUtils.ts**: Helper functions for 3D transformations.
- **heatmapShaders.ts**: Custom WebGL shaders for efficient heatmap rendering.

## Type Definitions

### types/index.ts
Centralized location for shared TypeScript interfaces and types, such as:
- `PointCloudData`: Structure for point cloud data.
- `GraphData`: Structure for GNG graph data.
- `DataSource`: Definition of available data sources.
- `GngStatus`: Status of the GNG process.
