"""公式OSCBFを用いたMuJoCo用速度フィルタ。"""
from pathlib import Path
import sys
import json


def create_filter(scene, settings):
    root = Path(__file__).parent
    if not (root / '.deps/cbfpy').exists():
        raise ImportError('OSCBFの依存が未導入です。oscbf_avoidance/install.shを実行してください')
    for path in (root / '.deps', root / 'vendor'):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))
    from .filter import CollisionFilter
    defaults = json.loads((root / 'defaults.json').read_text())
    return CollisionFilter(scene, dict(defaults, **settings))
