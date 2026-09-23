#!/usr/bin/env python3
"""SpatialTree のヘッダ群を 1ファイルに結合して bsp3d/include/bsp3d/bsp3d.hpp を作る。

元の実装（SpatialTree サブモジュール）を変更せず、そこから生成する。
追従するときは、このスクリプトを再実行するだけでよい。
"""
import re, pathlib, datetime, sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
SRC = ROOT / "SpatialTree" / "include" / "SpatialTree"
OUT = ROOT / "bsp3d" / "include" / "bsp3d" / "bsp3d.hpp"

# 依存順に並べる
FILES = [
    "Config.hpp", "Point.hpp", "Policy.hpp", "Traits.hpp",
    "SpatialTree.hpp",          # SearchResult / SpatialTreeParams / AdaptiveTree
    "MovingBSPTree.hpp",
    "GNG/Node.hpp", "GNG/GNGPolicy.hpp", "GNG/GNG.hpp",
]

STD_INCLUDES = set()
body = []
for f in FILES:
    text = (SRC / f).read_text()
    # インクルードガードとローカル include を取り除き、標準ヘッダは集約する
    text = re.sub(r"#ifndef\s+\w+_HPP\s*\n#define\s+\w+_HPP\s*\n", "", text, count=1)
    text = re.sub(r"#endif\s*//[^\n]*\n?\s*$", "", text)
    def collect(m):
        inc = m.group(0)
        if '"' in inc or "SpatialTree/" in inc:
            return ""
        STD_INCLUDES.add(inc.strip())
        return ""
    text = re.sub(r"#include\s+[<\"][^>\"]+[>\"]", collect, text)
    body.append(f"// ===== {f} " + "=" * max(0, 60 - len(f)) + "\n" + text.strip() + "\n")

header = f"""// bsp3d: 動く点群向け空間索引 + Growing Neural Gas（単一ヘッダ版）
//
// SpatialTree の実装から {pathlib.Path(__file__).name} で生成したものです。
// 生成元: SpatialTree/include/SpatialTree/{{{", ".join(FILES)}}}
// 生成日: {datetime.date.today()}
//
// 直接編集しないでください。元のヘッダを直してから再生成します。
//   python3 bsp3d/tools/amalgamate.py
#ifndef BSP3D_SINGLE_HEADER_HPP
#define BSP3D_SINGLE_HEADER_HPP

"""
tail = """

// ===== 3次元向けの型エイリアスと既定値 =======================
namespace bsp3d {

using Scalar = float;
inline constexpr int kDim = 3;
using Point3 = SpatialTree::Point<Scalar, kDim>;

/// 索引に入れる要素が満たすべき形（position / spatial_handle / index_in_cell）
/// 例:
///   struct MyNode { bsp3d::Point3 position; void* spatial_handle = nullptr; int index_in_cell = -1; };
template <class T>
using Index = SpatialTree::MovingBSPTree<T, Scalar, kDim>;

template <class T>
using SearchResult = SpatialTree::SearchResult<T, Scalar, kDim>;

using Params = SpatialTree::MovingBSPParams<Scalar>;

/// GNG（索引に Index を使う）。Node は GNGNode を使う
using GNGNode = SpatialTree::Node<Scalar, kDim>;
using GNG = SpatialTree::GrowingNeuralGas<Scalar, kDim, SpatialTree::NoHysteresis, true,
                                          SpatialTree::MovingBSPTree>;
using GNGParams = SpatialTree::GNGParams<Scalar>;

} // namespace bsp3d

#endif // BSP3D_SINGLE_HEADER_HPP
"""
OUT.parent.mkdir(parents=True, exist_ok=True)
OUT.write_text(header + "\n".join(sorted(STD_INCLUDES)) + "\n\n" + "\n".join(body) + tail)
print(f"wrote {OUT} ({len(OUT.read_text().splitlines())} 行)")
