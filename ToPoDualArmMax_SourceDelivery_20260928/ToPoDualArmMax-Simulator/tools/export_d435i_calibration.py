"""Optional: read an attached D435i using pyrealsense2; no device settings are written.
Run explicitly on a PC with the SDK and device. Not used by the simulator runtime.
"""
import json
from pathlib import Path
import pyrealsense2 as rs


def intrinsics(profile):
    k = profile.as_video_stream_profile().get_intrinsics()
    if any(abs(v) > 1e-12 for v in k.coeffs):
        raise ValueError("Non-zero distortion: rectify the stream and use its rectified intrinsics first.")
    return dict(width=k.width, height=k.height, fx=k.fx, fy=k.fy,
                ppx=k.ppx, ppy=k.ppy, model="none", coeffs=[0] * 5)


def main():
    pipeline, config = rs.pipeline(), rs.config()
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 30)
    profile = pipeline.start(config)
    try:
        depth, color = profile.get_stream(rs.stream.depth), profile.get_stream(rs.stream.color)
        ex = depth.get_extrinsics_to(color)
        # SDK rotation is column-major; simulator calibration is row-major.
        rotation = [ex.rotation[col * 3 + row] for row in range(3) for col in range(3)]
        sensor = profile.get_device().first_depth_sensor()
        result = dict(version=1, label="Device intrinsics/extrinsics; URDF mount remains unmeasured",
                      depth=intrinsics(depth), color=intrinsics(color),
                      depth_to_color=dict(rotation=rotation, translation=list(ex.translation)),
                      mount=dict(translation=[0, 0, 0], rpy=[0, 0, 0]), baseline_m=.05,
                      depth_scale=sensor.get_depth_scale(), min_depth_m=.195, max_depth_m=3.)
        destination = Path("d435i-calibration.json")
        with destination.open("x", encoding="utf-8") as file:
            json.dump(result, file, indent=2, ensure_ascii=False)
        print(destination.resolve())
        print("Paste this JSON into the simulator calibration editor; separately measure URDF mount correction.")
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
