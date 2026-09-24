"""重点候補の座標コピーを元点番号・区間参照へ置換する比較用変換。"""
import argparse
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('source', type=Path)
parser.add_argument('--variant', choices=('ids', 'spans'), default='spans')
args = parser.parse_args()
path = args.source / 'src/cpu/gng.cpp'
text = path.read_text()
old = 'n1.observation_angle_table && !enable_observation_attention_compact ? &observation_attention_raw_ids : nullptr'
assert old in text
text = text.replace(old, '!enable_observation_attention_compact ? &observation_attention_raw_ids : nullptr')
old = 'if (n1.observation_angle_table && !enable_observation_attention_compact) {observation_attention_raw_ids.resize(input_pcl_num);}'
assert old in text
text = text.replace(old, 'if (!enable_observation_attention_compact) {observation_attention_raw_ids.resize(input_pcl_num);}')
if args.variant == 'spans':
    old = 'enable_observation_attention_compact = n1.observation_angle_table && input_pcl_num > max_observation_direct_point_num;'
    assert old in text
    text = text.replace(old, 'enable_observation_attention_compact = input_pcl_num > max_observation_direct_point_num;')
old = '''        } else if (n1.observation_angle_table) {
            for (j = vg.voxel_range[i].start; j < vg.voxel_range[i].end; ++j) {
                const auto raw_idx = vg.voxel_index[j].raw_index;
                map.inpcl_labels[raw_idx] = voxel_labels[i];
                observation_attention_raw_ids[attention_pcl_num++] = raw_idx;
            }
        } else {
            for (j = vg.voxel_range[i].start; j < vg.voxel_range[i].end; ++j) {
                map.inpcl_labels[vg.voxel_index[j].raw_index] = voxel_labels[i];
                attention_pcl[attention_pcl_num] = map.input_pcl[vg.voxel_index[j].raw_index];
                attention_pcl_num++;
            }
        }'''
new = '''        } else {
            // 全候補の座標複製を省いた、選択順を保つ元点番号の保存。
            for (j = vg.voxel_range[i].start; j < vg.voxel_range[i].end; ++j) {
                const auto raw_idx = vg.voxel_index[j].raw_index;
                map.inpcl_labels[raw_idx] = voxel_labels[i];
                observation_attention_raw_ids[attention_pcl_num++] = raw_idx;
            }
        }'''
assert old in text
text = text.replace(old, new)
text = text.replace('    attention_pcl.resize(param.config.point_cloud_num);',
                    '    // 重点座標は学習時の元点参照。互換引数用の空配列。\n    attention_pcl.clear();')
path.write_text(text)
