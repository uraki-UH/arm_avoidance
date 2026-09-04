#!/usr/bin/env python3
"""三角形meshの完全表面点群化、ROS 2配信、GNG保存結果の統合ツール。"""

import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import random
import re
import struct
import sys
from datetime import datetime, timezone


TEMPLATE_ID_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")
POINT_FIELDS = ("x", "y", "z")


def utc_timestamp():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def validate_template_id(template_id):
    if not TEMPLATE_ID_PATTERN.fullmatch(template_id):
        raise ValueError("template_idは英字開始の英数字または_だけで指定してください。")


def read_text_mesh_triangles(mesh_path):
    vertices = []
    triangles = []
    for line in mesh_path.read_text(encoding="utf-8", errors="ignore").splitlines():
        values = line.strip().split()
        if not values:
            continue
        if values[0] == "v" and len(values) >= 4:
            vertices.append((float(values[1]), float(values[2]), float(values[3])))
            continue
        if values[0] != "f" or len(values) < 4:
            continue
        indices = []
        for token in values[1:]:
            index_text = token.split("/", 1)[0]
            if not index_text:
                raise ValueError(f"OBJ faceの頂点指定が不正です: {line}")
            index = int(index_text)
            index = index - 1 if index > 0 else len(vertices) + index
            if index < 0 or index >= len(vertices):
                raise ValueError(f"OBJ faceの頂点番号が範囲外です: {line}")
            indices.append(index)
        for index in range(1, len(indices) - 1):
            triangles.append((vertices[indices[0]], vertices[indices[index]], vertices[indices[index + 1]]))
    if not vertices or not triangles:
        raise ValueError("OBJから三角形面を取得できません。")
    return triangles


def read_binary_stl_triangles(data):
    if len(data) < 84:
        return None
    triangle_num = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_num * 50
    if expected_size != len(data):
        return None
    triangles = []
    offset = 84
    for _ in range(triangle_num):
        values = struct.unpack_from("<12fH", data, offset)
        triangles.append((
            (values[3], values[4], values[5]),
            (values[6], values[7], values[8]),
            (values[9], values[10], values[11]),
        ))
        offset += 50
    return triangles


def read_ascii_stl_triangles(data):
    values = re.findall(
        rb"\bvertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)", data)
    if len(values) < 3 or len(values) % 3 != 0:
        raise ValueError("ASCII STLから三角形面を取得できません。")
    vertices = [tuple(float(value) for value in vertex) for vertex in values]
    return [tuple(vertices[index:index + 3]) for index in range(0, len(vertices), 3)]


def read_stl_triangles(mesh_path):
    data = mesh_path.read_bytes()
    triangles = read_binary_stl_triangles(data)
    return triangles if triangles is not None else read_ascii_stl_triangles(data)


def read_binary_ply_triangles(mesh_path):
    data = mesh_path.read_bytes()
    header_end = data.find(b"end_header\n")
    if header_end < 0:
        raise ValueError("PLY headerが不正です。")
    header_end += len(b"end_header\n")
    lines = data[:header_end].decode("ascii", errors="strict").splitlines()
    if "format binary_little_endian 1.0" not in lines:
        raise ValueError("対応PLY形式はbinary_little_endian 1.0です。")
    vertex_num = 0
    face_num = 0
    vertex_properties = []
    reading_vertex = False
    reading_face = False
    type_formats = {"float": "f", "float32": "f", "double": "d", "uchar": "B", "uint8": "B"}
    for line in lines:
        fields = line.split()
        if fields[:2] == ["element", "vertex"]:
            vertex_num = int(fields[2])
            reading_vertex, reading_face = True, False
        elif fields[:2] == ["element", "face"]:
            face_num = int(fields[2])
            reading_vertex, reading_face = False, True
        elif reading_vertex and fields[:1] == ["property"] and len(fields) == 3:
            if fields[1] not in type_formats:
                raise ValueError(f"未対応PLY vertex型です: {fields[1]}")
            vertex_properties.append((fields[2], type_formats[fields[1]]))
        elif reading_face and fields[:2] == ["property", "list"]:
            if fields[2:4] != ["uchar", "uint"]:
                raise ValueError("対応PLY face index型はuchar/uintです。")
    if vertex_num <= 0 or face_num <= 0:
        raise ValueError("PLYにvertexまたはfaceがありません。")
    vertex_format = "<" + "".join(item[1] for item in vertex_properties)
    vertex_size = struct.calcsize(vertex_format)
    property_indices = {name: index for index, (name, _) in enumerate(vertex_properties)}
    if not all(name in property_indices for name in POINT_FIELDS):
        raise ValueError("PLY vertexにx/y/zがありません。")
    offset = header_end
    vertices = []
    for _ in range(vertex_num):
        values = struct.unpack_from(vertex_format, data, offset)
        vertices.append(tuple(values[property_indices[name]] for name in POINT_FIELDS))
        offset += vertex_size
    triangles = []
    for _ in range(face_num):
        count = data[offset]
        offset += 1
        indices = struct.unpack_from("<" + "I" * count, data, offset)
        offset += count * 4
        for index in range(1, count - 1):
            triangles.append((vertices[indices[0]], vertices[indices[index]], vertices[indices[index + 1]]))
    return triangles


def read_mesh_triangles(mesh_path):
    suffix = mesh_path.suffix.lower()
    if suffix == ".obj":
        return read_text_mesh_triangles(mesh_path)
    if suffix == ".stl":
        return read_stl_triangles(mesh_path)
    if suffix == ".ply":
        return read_binary_ply_triangles(mesh_path)
    raise ValueError("対応mesh形式はOBJ、STL、binary PLYです。")


def subtract(first, second):
    return (first[0] - second[0], first[1] - second[1], first[2] - second[2])


def cross(first, second):
    return (
        first[1] * second[2] - first[2] * second[1],
        first[2] * second[0] - first[0] * second[2],
        first[0] * second[1] - first[1] * second[0],
    )


def triangle_area(triangle):
    first = subtract(triangle[1], triangle[0])
    second = subtract(triangle[2], triangle[0])
    vector = cross(first, second)
    return 0.5 * math.sqrt(sum(value * value for value in vector))


def transform_triangles(triangles, unit_scale, keep_mesh_pose):
    if unit_scale <= 0.0:
        raise ValueError("unit_scaleは正数で指定してください。")
    scaled = [
        tuple(tuple(value * unit_scale for value in vertex) for vertex in triangle)
        for triangle in triangles
    ]
    coordinates = [coordinate for triangle in scaled for vertex in triangle for coordinate in vertex]
    xs = coordinates[0::3]
    ys = coordinates[1::3]
    zs = coordinates[2::3]
    if keep_mesh_pose:
        translation = (0.0, 0.0, 0.0)
    else:
        translation = (-0.5 * (min(xs) + max(xs)), -0.5 * (min(ys) + max(ys)), -min(zs))
    transformed = [
        tuple(
            tuple(vertex[axis] + translation[axis] for axis in range(3))
            for vertex in triangle
        )
        for triangle in scaled
    ]
    return transformed, translation


def sample_surface_points(triangles, point_count, seed):
    if point_count < 3:
        raise ValueError("point_countは3以上で指定してください。")
    weighted = []
    total_area = 0.0
    for triangle in triangles:
        area = triangle_area(triangle)
        if area <= 0.0:
            continue
        total_area += area
        weighted.append((total_area, triangle))
    if not weighted:
        raise ValueError("面積を持つ三角形がありません。")
    randomizer = random.Random(seed)
    points = []
    for point_id in range(point_count):
        selection = randomizer.uniform(0.0, total_area)
        low = 0
        high = len(weighted) - 1
        while low < high:
            middle = (low + high) // 2
            if selection <= weighted[middle][0]:
                high = middle
            else:
                low = middle + 1
        _, triangle = weighted[low]
        radial = math.sqrt(randomizer.random())
        first_weight = 1.0 - radial
        second_weight = radial * (1.0 - randomizer.random())
        third_weight = radial - second_weight
        point = tuple(
            first_weight * triangle[0][axis] +
            second_weight * triangle[1][axis] +
            third_weight * triangle[2][axis]
            for axis in range(3)
        )
        points.append({"id": point_id, "x": point[0], "y": point[1], "z": point[2], "label": "surface"})
    return points


def write_pcd(path, points):
    header = "\n".join((
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS x y z",
        "SIZE 4 4 4",
        "TYPE F F F",
        "COUNT 1 1 1",
        f"WIDTH {len(points)}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {len(points)}",
        "DATA ascii",
    ))
    with path.open("w", encoding="utf-8") as stream:
        stream.write(header)
        stream.write("\n")
        for point in points:
            stream.write(f"{point['x']:.9g} {point['y']:.9g} {point['z']:.9g}\n")


def write_json(path, value):
    with path.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write("\n")


def read_json(path):
    opener = gzip.open if path.suffix == ".gz" else open
    with opener(path, "rt", encoding="utf-8") as stream:
        return json.load(stream)


def command_prepare(arguments):
    mesh_paths = [Path(value).resolve() for value in arguments.mesh]
    for mesh_path in mesh_paths:
        if not mesh_path.is_file():
            raise ValueError(f"meshがありません: {mesh_path}")
    validate_template_id(arguments.template_id)
    triangles = [triangle for mesh_path in mesh_paths for triangle in read_mesh_triangles(mesh_path)]
    transformed, translation = transform_triangles(
        triangles, arguments.unit_scale, arguments.keep_mesh_pose)
    points = sample_surface_points(transformed, arguments.point_count, arguments.seed)
    output_dir = Path(arguments.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    dataset_path = output_dir / f"{arguments.template_id}_object_surface_dataset_v1.json"
    pcd_path = output_dir / f"{arguments.template_id}_surface.pcd"
    if (dataset_path.exists() or pcd_path.exists()) and not arguments.replace:
        raise ValueError("出力済みです。置換する場合は--replaceを指定してください。")
    mesh_hash = hashlib.sha256()
    for mesh_path in mesh_paths:
        mesh_hash.update(mesh_path.name.encode("utf-8"))
        mesh_hash.update(mesh_path.read_bytes())
    dataset = {
        "schema_version": 1,
        "kind": "object_surface_dataset",
        "dataset_id": arguments.template_id,
        "display_name": arguments.display_name,
        "canonical_yaw_deg": 0.0,
        "created_at": utc_timestamp(),
        "source": {
            "sample": "mesh_surface",
            "point_cloud_source": "mesh_surface",
            "frame_id": arguments.frame_id,
            "source_mesh": {
                "files": [mesh_path.name for mesh_path in mesh_paths],
                "sha256": mesh_hash.hexdigest(),
                "unit_scale": arguments.unit_scale,
                "translation": list(translation),
                "keep_mesh_pose": arguments.keep_mesh_pose,
            },
            "surface_sampling": {
                "method": "triangle_area_uniform",
                "point_count": arguments.point_count,
                "seed": arguments.seed,
            },
        },
        "surface_points": points,
    }
    write_json(dataset_path, dataset)
    write_pcd(pcd_path, points)
    print(f"surface_dataset={dataset_path}")
    print(f"surface_pcd={pcd_path}")
    print(f"surface_points={len(points)} triangles={len(triangles)}")


def load_surface_dataset(dataset_path):
    dataset = read_json(dataset_path)
    if dataset.get("kind") != "object_surface_dataset":
        raise ValueError("object_surface_datasetを指定してください。")
    points = dataset.get("surface_points")
    if not isinstance(points, list) or not points:
        raise ValueError("surface_pointsがありません。")
    for point in points:
        if not all(isinstance(point.get(field), (int, float)) for field in POINT_FIELDS):
            raise ValueError("surface_pointsのx/y/zが不正です。")
    return dataset, points


def command_publish(arguments):
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2, PointField
    except ImportError as error:
        raise RuntimeError("ROS 2環境をsourceしてからpublishを実行してください。") from error

    dataset_path = Path(arguments.dataset).resolve()
    dataset, points = load_surface_dataset(dataset_path)
    frame_id = arguments.frame_id or dataset.get("source", {}).get("frame_id", "object_template")
    if arguments.rate_hz <= 0.0:
        raise ValueError("rate_hzは正数で指定してください。")
    payload = b"".join(struct.pack("<fff", point["x"], point["y"], point["z"]) for point in points)

    class SurfacePublisher(Node):
        def __init__(self):
            super().__init__("mesh_surface_template_publisher")
            self.publisher = self.create_publisher(PointCloud2, arguments.topic, 10)
            self.message = PointCloud2()
            self.message.header.frame_id = frame_id
            self.message.height = 1
            self.message.width = len(points)
            self.message.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            self.message.is_bigendian = False
            self.message.point_step = 12
            self.message.row_step = self.message.point_step * self.message.width
            self.message.is_dense = True
            self.message.data = payload
            self.timer = self.create_timer(1.0 / arguments.rate_hz, self.publish)
            self.get_logger().info(
                f"完全表面点群配信開始: topic={arguments.topic} points={len(points)} rate_hz={arguments.rate_hz:g}")

        def publish(self):
            self.message.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.message)

    rclpy.init()
    node = SurfacePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def command_merge_gng(arguments):
    surface_path = Path(arguments.surface_dataset).resolve()
    gng_path = Path(arguments.gng_dataset).resolve()
    dataset, _ = load_surface_dataset(surface_path)
    template = read_json(gng_path)
    if template.get("kind") != "object_template":
        raise ValueError("ais_gngが保存したobject_templateを指定してください。")
    template_id = template.get("template_id")
    if template_id != dataset.get("dataset_id"):
        raise ValueError("表面データセットとGNGデータセットのtemplate_idが一致しません。")
    graph = template.get("gng")
    if not isinstance(graph, dict) or not isinstance(graph.get("nodes"), list) or not graph["nodes"]:
        raise ValueError("GNG nodeがありません。学習後のデータセットを指定してください。")
    dataset["display_name"] = template.get("display_name", dataset.get("display_name", template_id))
    dataset["gng_template"] = template
    dataset["updated_at"] = utc_timestamp()
    output_path = Path(arguments.output).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists() and not arguments.replace:
        raise ValueError("出力済みです。置換する場合は--replaceを指定してください。")
    write_json(output_path, dataset)
    print(f"merged_dataset={output_path}")
    print(f"template_id={template_id} nodes={len(graph['nodes'])} edges={len(graph.get('edges', []))}")


def create_parser():
    parser = argparse.ArgumentParser(description="完全表面mesh GNGテンプレート補助ツール")
    commands = parser.add_subparsers(dest="command", required=True)

    prepare = commands.add_parser("prepare", help="OBJ/STLを完全表面点群JSONとPCDへ変換")
    prepare.add_argument("--mesh", required=True, action="append")
    prepare.add_argument("--template-id", required=True)
    prepare.add_argument("--display-name", required=True)
    prepare.add_argument("--output-dir", required=True)
    prepare.add_argument("--point-count", type=int, default=50000)
    prepare.add_argument("--unit-scale", type=float, default=1.0)
    prepare.add_argument("--frame-id", default="object_template")
    prepare.add_argument("--seed", type=int, default=42)
    prepare.add_argument("--keep-mesh-pose", action="store_true")
    prepare.add_argument("--replace", action="store_true")
    prepare.set_defaults(handler=command_prepare)

    publish = commands.add_parser("publish", help="完全表面点群JSONをPointCloud2として配信")
    publish.add_argument("--dataset", required=True)
    publish.add_argument("--topic", default="/mesh_template_surface")
    publish.add_argument("--frame-id", default="")
    publish.add_argument("--rate-hz", type=float, default=10.0)
    publish.set_defaults(handler=command_publish)

    merge = commands.add_parser("merge-gng", help="ais_gng保存GNGを完全表面点群JSONへ統合")
    merge.add_argument("--surface-dataset", required=True)
    merge.add_argument("--gng-dataset", required=True)
    merge.add_argument("--output", required=True)
    merge.add_argument("--replace", action="store_true")
    merge.set_defaults(handler=command_merge_gng)
    return parser


def main():
    parser = create_parser()
    arguments = parser.parse_args()
    try:
        arguments.handler(arguments)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
