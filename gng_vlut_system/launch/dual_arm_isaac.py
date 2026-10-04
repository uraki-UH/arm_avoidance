#!/usr/bin/env python3
"""Isaac Sim 6.1の標準ros2_controlによる固定基台ロボットの起動。"""
import argparse
import math
from pathlib import Path
import signal
import time
import tempfile
import xml.etree.ElementTree as et

import yaml

from dual_arm_effort_config import load_model, effort_joints, controller_parameters, validate_namespace


package_dir = Path(__file__).resolve().parents[1]


def prepare_config(urdf_path, output_dir, tuning):
    """元URDFを保持したメッシュ解決と共通effort設定の生成。"""
    root = load_model(urdf_path)
    gains = effort_joints(root, tuning)
    output_dir.mkdir(parents=True, exist_ok=True)
    urdf = output_dir/'robot.urdf'
    urdf.write_text(et.tostring(root, encoding='unicode'))
    config = output_dir/'controllers.yaml'
    # Isaac標準拡張での名前空間適用。YAMLは機体名に依存しない形式
    config.write_text(yaml.safe_dump(controller_parameters(gains)))
    return root, gains, urdf, config


def configure_drives(stage, root_path, model, gains):
    """USDの関節対応検査と力制御設定。元USDの上書きなし。"""
    from pxr import Usd, UsdPhysics
    prim = stage.GetPrimAtPath(root_path)
    if not prim.IsValid():
        raise ValueError(f'ロボットprimが見つかりません: {root_path}')
    joints = {}
    roots = []
    for item in Usd.PrimRange(prim):
        if item.HasAPI(UsdPhysics.ArticulationRootAPI):
            roots.append(item)
        if item.IsA(UsdPhysics.RevoluteJoint) or item.IsA(UsdPhysics.PrismaticJoint):
            name = item.GetName()
            if name in joints:
                raise ValueError(f'USD内の関節名重複: {name}')
            joints[name] = item
    expected = {j.get('name') for j in model.findall('joint') if j.get('type') != 'fixed'}
    if set(joints) != expected or len(roots) != 1:
        raise ValueError(f'URDFとUSDの関節またはarticulation不一致: missing={expected-set(joints)}, extra={set(joints)-expected}, roots={len(roots)}')
    for name, item in joints.items():
        axis = 'angular' if item.IsA(UsdPhysics.RevoluteJoint) else 'linear'
        if name not in gains:
            # mimicへの独立した駆動力の禁止
            item.RemoveAPI(UsdPhysics.DriveAPI, axis)
            continue
        drive = UsdPhysics.DriveAPI.Apply(item, axis)
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.0)
        drive.CreateMaxForceAttr(gains[name]['u_clamp_max'])
    # 関節primがarticulation基台の兄弟にあるUSDにも対応する探索範囲
    return root_path


def run(args):
    # Isaacランタイムの初期化前に可能な入力検査
    validate_namespace(args.namespace)
    if not math.isfinite(args.max_run_sec) or args.max_run_sec < 0:
        raise ValueError('max_run_secには有限の非負値が必要です')
    output = args.output_dir.resolve() if args.output_dir else Path(tempfile.mkdtemp(prefix="dual_arm_isaac_"))
    if args.output_dir and output.exists():
        raise FileExistsError(f'生成物を保護するため未使用のoutput_dirが必要です: {output}')
    root, gains, urdf, config = prepare_config(args.urdf, output, yaml.safe_load(args.control_config.read_text()))
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': not args.enable_gui, 'renderer': 'RayTracedLighting'})
    is_running = True
    manager = None
    has_manager = False
    articulation_path = None
    app_utils = None

    def stop(signum, frame):
        nonlocal is_running
        is_running = False

    previous_handlers = {sig: signal.signal(sig, stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        import omni.graph.core as og
        import omni.kit.app
        import omni.usd
        import isaacsim.core.experimental.utils.app as app_utils
        import isaacsim.core.experimental.utils.stage as stage_utils
        from isaacsim.core.simulation_manager import SimulationManager

        for extension in ('isaacsim.ros2.core', 'isaacsim.ros2.bridge', 'isaacsim.ros2.control',
                          'isaacsim.robot.schema', 'isaacsim.asset.importer.urdf', 'omni.scene.optimizer.core'):
            app_utils.enable_extension(extension)
        app.update()
        from isaacsim.ros2.control import Ros2ControlManager
        from isaacsim.asset.importer.urdf.impl import URDFImporter, URDFImporterConfig
        manager = Ros2ControlManager
        stage_utils.create_new_stage()
        stage_utils.set_stage_units(meters_per_unit=1.0)
        settings = URDFImporterConfig()
        settings.urdf_path = str(urdf)
        settings.usd_path = str(output/'usd')
        settings.fix_base = True
        settings.merge_fixed_joints = False
        settings.allow_self_collision = True
        settings.joint_drive_type = 'force'
        settings.joint_target_type = 'none'
        settings.override_joint_stiffness = 0.0
        settings.override_joint_damping = 0.0
        usd_path = URDFImporter(settings).import_urdf()
        if not usd_path:
            raise RuntimeError('URDFからUSDへの変換に失敗しました')
        stage_utils.add_reference_to_stage(usd_path=str(usd_path), path='/World/Robot')
        app.update()
        stage = omni.usd.get_context().get_stage()
        articulation_path = configure_drives(stage, '/World/Robot', root, gains)
        stage.GetRootLayer().Export(str(output/'scene.usda'))
        keys = og.Controller.Keys
        og.Controller.edit({'graph_path': '/SimulationClock', 'evaluator_name': 'execution'}, {
            keys.CREATE_NODES: [('tick', 'omni.graph.action.OnPlaybackTick'),
                                ('time', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                                ('clock', 'isaacsim.ros2.bridge.ROS2PublishClock')],
            keys.CONNECT: [('tick.outputs:tick', 'clock.inputs:execIn'),
                           ('time.outputs:simulationTime', 'clock.inputs:timeStamp')]})
        SimulationManager.setup_simulation(dt=0.001, device='cpu')
        app_utils.play()
        for _ in range(5):
            app.update()
        # 物理初期化後の停止中に標準Controller Managerを構成
        app_utils.pause()
        for _ in range(2):
            app.update()
        if manager.setup(articulation_path, str(config), namespace=args.namespace) != 0:
            raise RuntimeError('IsaacのController Manager起動失敗')
        has_manager = True
        app_utils.play()
        begin = time.monotonic()
        while is_running and app.is_running():
            if args.max_run_sec and time.monotonic()-begin >= args.max_run_sec:
                break
            app.update()
    finally:
        try:
            if has_manager:
                manager.teardown(articulation_path)
        finally:
            if app_utils is not None:
                app_utils.stop()
            app.close()
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, default=package_dir.parent/'urdf/topo_dual_arm_max/topo_dual_arm_max.urdf')
    parser.add_argument('--control-config', type=Path, default=package_dir/'config/dual_arm_effort.yaml')
    parser.add_argument('--namespace', default='sim_topo_dual_arm_max')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--enable-gui', action='store_true')
    parser.add_argument('--max-run-sec', type=float, default=0.0)
    run(parser.parse_args())


if __name__ == '__main__':
    main()
