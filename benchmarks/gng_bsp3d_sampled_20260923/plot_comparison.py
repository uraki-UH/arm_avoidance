from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

workspace = Path(__file__).resolve().parents[2]
artifacts = workspace / "artifacts/gng_bsp3d_sampled_20260923"
fig, axes = plt.subplots(2,2,figsize=(13,10),constrained_layout=True)
for column,(variant,title) in enumerate((("grid","CPU grid"),("sampled","BSP + bounded probes"))):
    graph = np.load(artifacts/f"{variant}_0_1_1.graph.npz")
    points,nodes,edges = graph["input_points"],graph["nodes"],graph["edges"]
    for row,(first,second) in enumerate(((0,1),(0,2))):
        ax = axes[row,column]
        # 図の描画密度だけの調整。計算・評価に使う入力点の削減なし。
        ax.scatter(points[::4,first],points[::4,second],s=0.4,c="#999999",alpha=0.35,rasterized=True,label="Input")
        segments = nodes[edges][:,:,[first,second]]
        ax.add_collection(LineCollection(segments,colors="#2674aa",linewidths=0.15,alpha=0.25,rasterized=True))
        ax.scatter(nodes[:,first],nodes[:,second],s=0.6,c="#d94b30",alpha=0.7,rasterized=True,label="Nodes")
        ax.set_title(title+" / "+("XY" if row==0 else "XZ"))
        ax.set_xlabel("X [m]")
        ax.set_ylabel(("Y" if row==0 else "Z")+" [m]")
        ax.set_xlim(-2,85)
        ax.set_ylim((-65,70) if row==0 else (-6,11))
        ax.grid(alpha=0.2)
        if row==0:
            ax.set_aspect("equal",adjustable="box")
            ax.legend(markerscale=5,loc="lower right")
fig.suptitle("Same bag frame / input voxel 0.1 m / 20,000-node limit")
fig.savefig(artifacts/"graph_comparison.png",dpi=180)
print(artifacts/"graph_comparison.png")
