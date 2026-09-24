"""試行内平均の3試行中央値と初期構築費の集計。"""
from pathlib import Path
import json
import statistics
root=Path(__file__).resolve().parents[2]
out=root/'artifacts/goal_selection_efficiency_20260924'
rows=json.loads((out/'results.json').read_text())
initial=json.loads((out/'initial.json').read_text())
summary={}
for fixture in ['points','planar','robot']:
    summary[fixture]={'initial_ms':{},'scenarios':{}}
    for method in dict.fromkeys(x['method'] for x in rows):
        values=[]
        for trial in range(3):
            values.append(statistics.median(float(x['ms']) for x in initial if x['fixture']==fixture and x['method']==method and x['trial']==trial))
        summary[fixture]['initial_ms'][method]=statistics.median(values)
    for scenario in dict.fromkeys(x['scenario'] for x in rows):
        grouped={}
        for count in ['1','20','100']:
            grouped[count]={}
            for method in dict.fromkeys(x['method'] for x in rows):
                part=[x for x in rows if x['fixture']==fixture and x['scenario']==scenario and x['candidates']==count and x['method']==method]
                if len(part)!=3: raise RuntimeError('試行数の不一致')
                grouped[count][method]={key:statistics.median(float(x[key]) for x in part) for key in ['map_ms','features_ms','select_ms','total_ms','median_select_ms','median_total_ms']}
        summary[fixture]['scenarios'][scenario]=grouped
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
for fixture,values in summary.items():
    print(fixture, 'initial_ms', values['initial_ms'])
    for scenario,counts in values['scenarios'].items():
        print(scenario, {name:round(values['total_ms'],5) for name,values in counts['20'].items()})
