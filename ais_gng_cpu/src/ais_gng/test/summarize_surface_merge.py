"""曲面統合ベンチマークの集計。反復を標本数に含めない採否集計。"""
import argparse
import json
import statistics
from collections import defaultdict
from pathlib import Path


def summarize(rows):
    groups = defaultdict(list)
    for row in rows:
        groups[(row['suite'], row['mode'])].append(row)
    summary = []
    for (suite, mode), samples in sorted(groups.items()):
        cases = [row for row in samples if row['iter'] == 0]
        positive = [row for row in cases if row.get('is_expected_merge') is True]
        negative = [row for row in cases if row.get('is_expected_merge') is False]
        elapsed = sorted(row['elapsed_ms'] for row in samples)
        summary.append({
            'suite': suite, 'mode': mode, 'cases': len(cases),
            'positive': len(positive), 'positive_merged': sum(row['is_merged'] for row in positive),
            'negative': len(negative), 'negative_merged': sum(row['is_merged'] for row in negative),
            'merged': sum(row['is_merged'] for row in cases),
            'median_ms': statistics.median(elapsed),
            'quantile_95_ms': elapsed[int(0.95 * (len(elapsed)-1))],
        })
    repeated = defaultdict(dict)
    for row in rows:
        if row['suite'] == 'repeat_support' and row['iter'] == 0:
            key = (row['mode'], row['kind'], row['angle_deg'], row['has_bridge'])
            repeated[key][row['copies']] = row['is_merged']
    density = defaultdict(lambda: {'groups': 0, 'changed': 0})
    for key, decisions in repeated.items():
        density[key[0]]['groups'] += 1
        density[key[0]]['changed'] += len(set(decisions.values())) > 1
    by_kind = defaultdict(lambda: {'cases': 0, 'merged': 0})
    for row in rows:
        if row['suite'] != 'synthetic' or row['iter'] != 0:
            continue
        key = row['mode'] + '/' + row['kind']
        by_kind[key]['cases'] += 1
        by_kind[key]['merged'] += row['is_merged']
    return {'summary': summary, 'repeat_support': dict(density), 'by_kind': dict(by_kind)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    args = parser.parse_args()
    with args.input.open() as stream:
        rows = [json.loads(line) for line in stream]
    # 決定論的な採否・幾何条件の反復間整合性。
    decisions = defaultdict(list)
    for row in rows:
        decisions[(row['case_idx'], row['mode'])].append(row)
        assert row['elapsed_ms'] >= 0
        assert 0 <= row['coverage'] <= 1
    for samples in decisions.values():
        assert {row['iter'] for row in samples} == {0, 1, 2}
        assert len(samples) == 3
        assert len({row['is_merged'] for row in samples}) == 1
    print(json.dumps(summarize(rows), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
