"""反復測定用の圧縮器起動と数値指標への変換。"""
import argparse
import json
import subprocess
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('executable', type=Path)
    parser.add_argument('input_path', type=Path)
    parser.add_argument('radius_cells', type=int)
    parser.add_argument('seed', type=int)
    parser.add_argument('output_prefix', type=Path)
    args = parser.parse_args()
    subprocess.run([str(args.executable), str(args.input_path), str(args.radius_cells),
                    str(args.seed), str(args.output_prefix)], check=True, timeout=80)
    metrics = json.loads(Path(str(args.output_prefix) + '.metrics.json').read_text())
    # runnerの有限数値辞書制約に対応する真偽値の0/1表現
    numeric = {key: int(value) if isinstance(value, bool) else value for key, value in metrics.items()}
    Path(str(args.output_prefix) + '.numeric.json').write_text(json.dumps(numeric, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
