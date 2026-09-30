"""元姿勢と補完姿勢のTCP分布の比較図。"""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument('folder', type=Path)
args = parser.parse_args()
figure, axes = plt.subplots(2, 2, figsize=(10, 9), subplot_kw={'projection': '3d'})
for row, model in enumerate(('max', 'long')):
    folder = args.folder / model / 'selection'
    tcp = np.load(folder / 'candidates/all_tcp.npy')
    selected = np.load(folder / 'candidates/selected_candidate_idx.npy')
    for column, title in enumerate(('Original', 'Original + coverage witnesses')):
        axis = axes[row, column]
        points = tcp[:10000].reshape(-1, 3)
        axis.scatter(*points.T, s=.4, alpha=.12, color='#2478b5', rasterized=True)
        if column:
            points = tcp[selected].reshape(-1, 3)
            axis.scatter(*points.T, s=1.0, alpha=.2, color='#e37d26', rasterized=True)
        axis.set_title(f'{model}: {title}', fontsize=10)
        axis.set(xlim=(-.5, .5), ylim=(-.65, .65), zlim=(-.1, .9), xlabel='X [m]', ylabel='Y [m]', zlabel='Z [m]')
        axis.view_init(elev=18, azim=135)
        axis.set_box_aspect((1.0, 1.3, 1.0))
figure.suptitle('TCP positions in the URDF root frame\nBlue: original 10,000 poses; orange: selected additions', fontsize=12)
figure.tight_layout()
figure.savefig(args.folder / 'tcp_coverage.png', dpi=160)
plt.close(figure)
