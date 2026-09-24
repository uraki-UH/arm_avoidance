"""実在エッジ単一走査版の、固定基準からの独立ソース準備。"""
from pathlib import Path
import shutil
import subprocess
import sys
trial = Path(__file__).resolve().parent
output = trial.parents[1]/'artifacts'/trial.name
source = output/'pooled_source'
shutil.copytree(output/'before_source', source)
subprocess.run([sys.executable, str(trial/'optimize.py'), str(source), '--method', 'combined'], check=True)
subprocess.run([sys.executable, str(trial/'optimize_pool.py'), str(source)], check=True)
subprocess.run([sys.executable, str(trial/'register_test.py'), str(source)], check=True)
