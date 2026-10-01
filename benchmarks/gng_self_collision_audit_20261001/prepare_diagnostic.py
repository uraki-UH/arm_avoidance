"""表示専用の胴体外装を衝突形状へ追加する診断用コピーの生成。"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as element_tree

parser = argparse.ArgumentParser()
parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
body_names = {'waist_cover_link', 'L_shoulder_cover_link', 'R_shoulder_cover_link',
              'neck_tilt_cover_link', 'realsense_mount_link'}
reports = []
for name, source_name in [('max', 'topo_dual_arm_max'), ('long', 'topo_dual_arm_max_long')]:
    source = args.repo / 'urdf' / source_name / 'topo_dual_arm_max.urdf'
    tree = element_tree.parse(source)
    added = []
    for link in tree.getroot().findall('link'):
        if link.get('name') not in body_names:
            continue
        assert not link.findall('collision'), link.get('name')
        for visual in link.findall('visual'):
            collision = element_tree.SubElement(link, 'collision', {'name': 'audit_visual_body'})
            for tag in ('origin', 'geometry'):
                child = visual.find(tag)
                if child is not None:
                    collision.append(copy.deepcopy(child))
        added.append(link.get('name'))
    assert set(added) == body_names
    for mesh in tree.getroot().iter('mesh'):
        filename = mesh.get('filename')
        assert not filename.startswith(('package:', 'file:'))
        mesh.set('filename', str(Path('/ros2_ws/src/urdf') / source_name / filename))
        assert (source.parent / filename).is_file()
    target = args.output / name / 'body_visual_collision.urdf'
    target.parent.mkdir(parents=True, exist_ok=True)
    assert not target.exists(), target
    tree.write(target, encoding='utf-8', xml_declaration=True)
    reports.append({'model': name, 'source': str(source), 'diagnostic_urdf': str(target),
                    'added_body_links': added,
                    'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                    'diagnostic_sha256': hashlib.sha256(target.read_bytes()).hexdigest()})
(args.output / 'diagnostic_urdf_manifest.json').write_text(json.dumps(reports, indent=2) + '\n')
print(json.dumps(reports, indent=2))
