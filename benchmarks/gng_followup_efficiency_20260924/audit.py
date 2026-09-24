"""完了条件の明示的な全条件照合。部分集計だけでの成功判定を禁止。"""
import argparse
import hashlib
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('--methods', nargs='+', required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
output = root/'artifacts/gng_followup_efficiency_20260924'
expected = {f'voxel_{voxel}_{trial}': (300 if trial == 1 else 100, 50, False)
            for voxel in ('0.1','0.5') for trial in (1,2,3)}
expected.update({f'features_{voxel}': (30,5,True) for voxel in ('0','0.1','0.5')})
required_hashes = ('graph_sha256','cluster_sha256','label_sha256','normal_sha256','rho_sha256',
                   'positions_sha256','voxel_sha256','node_members_sha256','mapping_sha256')
required_times = ('matching_ms','candidates_ms','mapping_sort_ms','total_ms','learn_ms','cluster_ms')
checked = []
for method in ['before'] + args.methods:
    cache = (output/method/'CMakeCache.txt').read_text()
    assert 'CMAKE_BUILD_TYPE:STRING=Release' in cache
    commands = json.loads((output/method/'compile_commands.json').read_text())
    core = [row['command'] for row in commands if row['file'].endswith('/src/cpu/cugng.cpp')]
    assert core and all('-O3' in cmd and '-flto' in cmd for cmd in core), method
    for name,(frames,warmup,enable_features) in expected.items():
        path = output/method/(name+'.json')
        run = json.loads(path.read_text())
        before = json.loads((output/'before'/(name+'.json')).read_text())
        assert len(run['records']) == frames and run['warmup'] == warmup, (method,name)
        assert run['has_features'] == enable_features and run['accepted'] == before['accepted']
        keys = [key for key in before['records'][0] if key.endswith('_sha256') or key in
                ('nodes','edges','clusters','voxel_num','kept_input_num','attention_num','learning_num','events_num')]
        for idx,(first,second) in enumerate(zip(before['records'],run['records'])):
            assert all(key in second for key in required_hashes+required_times), (method,name,idx,second.keys())
            assert second['learning_num'] == 4000
            assert all(first[key] == second[key] for key in keys), (method,name,idx)
            if enable_features:
                assert all(key in second for key in ('events_sha256','delta_sha256','statistics_sha256','observation_sha256'))
        checked.append(dict(method=method,name=name,frames=frames,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
result = dict(methods=args.methods,cases=checked,executed_frames=sum(row['frames'] for row in checked),
              compared_frames=sum(row['frames'] for row in checked if row['method']!='before'))
(output/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({key:value for key,value in result.items() if key!='cases'}))
