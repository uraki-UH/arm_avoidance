"""環境シナリオの共通設定とHarmonic SDF・Isaac USDへの変換。"""
import math
from pathlib import Path
import re
import xml.etree.ElementTree as et

import yaml


scenario_dir = Path(__file__).resolve().parents[1] / 'config/simulation/scenarios'


def _mapping(value, allowed, required, label):
    if not isinstance(value, dict) or set(value) - set(allowed) or set(required) - set(value):
        raise ValueError(f'{label}: 必須キーの欠損または未対応キーです')
    return value


def _name(value):
    if not isinstance(value, str) or re.fullmatch(r'[a-z][a-z0-9_]*', value) is None:
        raise ValueError(f'物体名にはlower_snake_caseが必要です: {value}')
    return value


def _number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{label}: 有限の数値が必要です')
    return float(value)


def _vector(value, label):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f'{label}: 3要素の配列が必要です')
    return [_number(item, label) for item in value]


def _asset(value):
    shapes = {'box': ('size',), 'sphere': ('radius',), 'cylinder': ('radius', 'length')}
    if not isinstance(value, dict) or not isinstance(value.get('shape'), str) or value['shape'] not in shapes:
        raise ValueError('shapeにはbox・sphere・cylinderのいずれかが必要です')
    dimensions = shapes[value['shape']]
    _mapping(value, ('shape', 'color', 'is_static', 'mass', 'friction', *dimensions),
             ('shape', 'is_static', *dimensions), '物体定義')
    if type(value['is_static']) is not bool:
        raise ValueError('is_staticには真偽値が必要です')
    result = {'shape': value['shape'], 'is_static': value['is_static'],
              'color': _vector(value.get('color', [0.6, 0.6, 0.6]), 'color'),
              'friction': _number(value.get('friction', 0.8), 'friction')}
    if any(item < 0 or item > 1 for item in result['color']) or result['friction'] < 0:
        raise ValueError('色または摩擦係数の範囲が不正です')
    for key in dimensions:
        result[key] = _vector(value[key], key) if key == 'size' else _number(value[key], key)
        if any(item <= 0 for item in (result[key] if key == 'size' else [result[key]])):
            raise ValueError('物体寸法には正の値が必要です')
    if result['is_static']:
        if 'mass' in value:
            raise ValueError('固定物体にはmassの指定が不要です')
    else:
        result['mass'] = _number(value.get('mass'), 'mass')
        if result['mass'] <= 0:
            raise ValueError('可動物体には正の質量が必要です')
    return result


def load_scenario(selection='empty'):
    """組込み名またはYAMLパスからの物体参照解決。未知設定の黙示的な無視なし。"""
    path = Path(selection).expanduser()
    if path.suffix not in ('.yaml', '.yml'):
        _name(str(selection))
        path = scenario_dir / (str(selection) + '.yaml')
    data = _mapping(yaml.safe_load(path.read_text()), ('description', 'objects_file', 'objects'),
                    ('description', 'objects_file', 'objects'), 'シナリオ')
    if not isinstance(data['description'], str) or not isinstance(data['objects_file'], str):
        raise ValueError('descriptionとobjects_fileには文字列が必要です')
    if not isinstance(data['objects'], list):
        raise ValueError('objectsには配列が必要です')
    library_path = path.parent / data['objects_file']
    library = yaml.safe_load(library_path.read_text())
    if not isinstance(library, dict):
        raise ValueError('物体定義には名前をキーとする辞書が必要です')
    assets = {_name(name): _asset(item) for name, item in library.items()}
    result = {'description': data['description'], 'objects': []}
    names = set()
    for item in data['objects']:
        _mapping(item, ('name', 'asset', 'position', 'rpy'), ('name', 'asset', 'position'), '物体配置')
        name, asset = _name(item['name']), _name(item['asset'])
        if name in names or asset not in assets:
            raise ValueError(f'物体名重複または未知のassetです: {name}, {asset}')
        names.add(name)
        result['objects'].append({**assets[asset], 'name': name,
            'position': _vector(item['position'], 'position'),
            'rpy': _vector(item.get('rpy', [0.0, 0.0, 0.0]), 'rpy')})
    return result


def save_scenario(scenario, output_dir):
    """再起動に利用可能な解決済みシナリオと物体定義の保存。"""
    assets, objects = {}, []
    for item in scenario['objects']:
        assets[item['name']] = {key: value for key, value in item.items() if key not in ('name', 'position', 'rpy')}
        objects.append({key: item[key] for key in ('name', 'position', 'rpy')})
        objects[-1]['asset'] = item['name']
    (output_dir / 'scenario_objects.yaml').write_text(yaml.safe_dump(assets, sort_keys=False))
    (output_dir / 'scenario.yaml').write_text(yaml.safe_dump({
        'description': scenario['description'], 'objects_file': 'scenario_objects.yaml', 'objects': objects},
        allow_unicode=True, sort_keys=False))


def diagonal_inertia(item):
    """中心原点・一様密度の基本形状の主慣性。単位はkg m²。"""
    mass = item['mass']
    if item['shape'] == 'box':
        x, y, z = item['size']
        return [mass * (y*y + z*z) / 12, mass * (x*x + z*z) / 12, mass * (x*x + y*y) / 12]
    radius = item['radius']
    if item['shape'] == 'sphere':
        return [2 * mass * radius**2 / 5] * 3
    axial = mass * radius**2 / 2
    lateral = mass * (3 * radius**2 + item['length']**2) / 12
    return [lateral, lateral, axial]


def _text(parent, tag, value):
    element = et.SubElement(parent, tag)
    element.text = ' '.join(map(str, value)) if isinstance(value, (list, tuple)) else str(value)
    return element


def gazebo_world(scenario):
    """Harmonicの既存物理周期・制御world名を維持した環境生成。"""
    root = et.Element('sdf', version='1.9')
    world = et.SubElement(root, 'world', name='motor_test')
    _text(world, 'gravity', [0, 0, -9.81])
    physics = et.SubElement(world, 'physics', name='physics', type='ignored')
    _text(physics, 'max_step_size', 0.001)
    _text(physics, 'real_time_factor', 1)
    for filename, name in (('physics', 'Physics'), ('user-commands', 'UserCommands'), ('scene-broadcaster', 'SceneBroadcaster')):
        et.SubElement(world, 'plugin', filename='gz-sim-' + filename + '-system', name='gz::sim::systems::' + name)
    light = et.SubElement(world, 'light', name='sun', type='directional')
    _text(light, 'pose', [0, 0, 10, 0, 0, 0])
    _text(light, 'direction', [-0.5, 0.1, -0.9])
    for item in scenario['objects']:
        model = et.SubElement(world, 'model', name='environment_' + item['name'])
        _text(model, 'static', str(item['is_static']).lower())
        _text(model, 'pose', item['position'] + item['rpy'])
        link = et.SubElement(model, 'link', name='body')
        if not item['is_static']:
            inertial = et.SubElement(link, 'inertial')
            _text(inertial, 'mass', item['mass'])
            inertia = et.SubElement(inertial, 'inertia')
            for key, value in zip(('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'), [*diagonal_inertia(item), 0, 0, 0]):
                _text(inertia, key, value)
        for kind in ('collision', 'visual'):
            entry = et.SubElement(link, kind, name=kind)
            geometry = et.SubElement(et.SubElement(entry, 'geometry'), item['shape'])
            for key in {'box': ('size',), 'sphere': ('radius',), 'cylinder': ('radius', 'length')}[item['shape']]:
                _text(geometry, key, item[key])
            if kind == 'visual':
                material = et.SubElement(entry, 'material')
                for key in ('ambient', 'diffuse'):
                    _text(material, key, item['color'] + [1.0])
            else:
                surface = et.SubElement(entry, 'surface')
                friction = et.SubElement(et.SubElement(surface, 'friction'), 'ode')
                _text(friction, 'mu', item['friction'])
                _text(friction, 'mu2', item['friction'])
    return root


def add_isaac_environment(stage, scenario):
    """標準USD物理スキーマによる環境追加。固定衝突体と可動剛体の区別。"""
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics, UsdShade

    root_path = '/World/Environment'
    if stage.GetPrimAtPath(root_path).IsValid():
        raise ValueError('環境primが既に存在します')
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, root_path)
    UsdLux.DomeLight.Define(stage, '/World/EnvironmentLight').CreateIntensityAttr(500.0)
    for item in scenario['objects']:
        path = root_path + '/' + item['name']
        body = UsdGeom.Xform.Define(stage, path)
        body.AddTranslateOp().Set(Gf.Vec3d(*item['position']))
        roll, pitch, yaw = [value / 2 for value in item['rpy']]
        cos_roll, cos_pitch, cos_yaw = math.cos(roll), math.cos(pitch), math.cos(yaw)
        sin_roll, sin_pitch, sin_yaw = math.sin(roll), math.sin(pitch), math.sin(yaw)
        body.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(
            cos_roll*cos_pitch*cos_yaw + sin_roll*sin_pitch*sin_yaw,
            Gf.Vec3d(sin_roll*cos_pitch*cos_yaw - cos_roll*sin_pitch*sin_yaw,
                     cos_roll*sin_pitch*cos_yaw + sin_roll*cos_pitch*sin_yaw,
                     cos_roll*cos_pitch*sin_yaw - sin_roll*sin_pitch*cos_yaw)))
        shape_path = path + '/shape'
        if item['shape'] == 'box':
            shape = UsdGeom.Cube.Define(stage, shape_path)
            shape.CreateSizeAttr(1.0)
            shape.AddScaleOp().Set(Gf.Vec3f(*item['size']))
        elif item['shape'] == 'sphere':
            shape = UsdGeom.Sphere.Define(stage, shape_path)
            shape.CreateRadiusAttr(item['radius'])
        else:
            shape = UsdGeom.Cylinder.Define(stage, shape_path)
            shape.CreateAxisAttr(UsdGeom.Tokens.z)
            shape.CreateRadiusAttr(item['radius'])
            shape.CreateHeightAttr(item['length'])
        shape.CreateDisplayColorAttr([Gf.Vec3f(*item['color'])])
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim()).CreateCollisionEnabledAttr(True)
        material = UsdShade.Material.Define(stage, path + '/material')
        surface = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        surface.CreateStaticFrictionAttr(item['friction'])
        surface.CreateDynamicFrictionAttr(item['friction'])
        surface.CreateRestitutionAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(material, materialPurpose='physics')
        if not item['is_static']:
            UsdPhysics.RigidBodyAPI.Apply(body.GetPrim()).CreateRigidBodyEnabledAttr(True)
            mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
            mass.CreateMassAttr(item['mass'])
            mass.CreateCenterOfMassAttr(Gf.Vec3f(0.0))
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*diagonal_inertia(item)))
            mass.CreatePrincipalAxesAttr(Gf.Quatf(1.0))
    return root_path
