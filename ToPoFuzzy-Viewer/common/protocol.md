# Point Cloud Binary Protocol Specification

## Overview
High-performance binary protocol for streaming point cloud data via WebSocket from C++ backend to JavaScript frontend.

## Design Goals
- Zero-copy data transfer where possible
- Minimal parsing overhead
- Support for partial/incremental updates
- GPU-friendly data layout (contiguous arrays)

## Message Format

### Header (20 bytes)
```
Offset | Size | Type   | Field        | Description
-------|------|--------|--------------|------------------
0      | 4    | char   | magic        | "PCDX" (0x50434458)
4      | 4    | uint32 | version      | Protocol version (1)
8      | 4    | uint32 | pointCount   | Number of points
12     | 1    | uint8  | dataMask     | Bitfield for data presence
13     | 3    | -      | reserved     | Reserved for future use
16     | 4    | uint32 | payloadSize  | Total payload size in bytes
```

### Data Mask Bits
```
Bit 0: XYZ coordinates (always 1)
Bit 1: RGB colors
Bit 2: Intensity
Bit 3: Normals
Bit 4-7: Reserved
```

### Payload
Contiguous blocks in order:
1. **Positions** (always present): `float32[pointCount * 3]` - (x, y, z) interleaved
2. **Colors** (if bit 1 set): `uint8[pointCount * 3]` - (r, g, b) in [0, 255]
3. **Intensities** (if bit 2 set): `float32[pointCount]` - normalized [0, 1]
4. **Normals** (if bit 3 set): `float32[pointCount * 3]` - (nx, ny, nz) interleaved

## Example
For 100 points with XYZ + RGB:
```
Header: 20 bytes
Positions: 100 * 3 * 4 = 1200 bytes
Colors: 100 * 3 * 1 = 300 bytes
Total: 1520 bytes
```

## Endianness
Little-endian for all multi-byte values.

## WebSocket Message Type
Binary frame (opcode 0x02)
