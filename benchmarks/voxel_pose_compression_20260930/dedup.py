"""リンク別の同一占有辞書による、実関節姿勢を保持した可逆圧縮の検証。"""
import argparse
import json
import struct
import time
from pathlib import Path


def read_dataset(path):
    raw = Path(path).read_bytes()
    magic, num_nodes, num_links, angle_dim, coord_layers, resolution = struct.unpack_from('<8sIIIIf', raw)
    assert magic == b'VOXPOSE1'
    offset = 28
    pose_size = 4 + 4 * (angle_dim + 3 * coord_layers)
    poses, masks = [], []
    for node_idx in range(num_nodes):
        poses.append(raw[offset:offset + pose_size])
        offset += pose_size
        links = []
        for link_idx in range(num_links):
            num_cells = struct.unpack_from('<I', raw, offset)[0]
            offset += 4
            links.append(raw[offset:offset + num_cells * 12])
            offset += num_cells * 12
        masks.append(links)
    assert offset == len(raw)
    return (num_nodes, num_links, angle_dim, coord_layers, resolution), poses, masks, len(raw)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input')
    parser.add_argument('output_prefix')
    args = parser.parse_args()
    config, poses, masks, original_bytes = read_dataset(args.input)
    num_nodes, num_links, angle_dim, coord_layers, resolution = config
    start = time.perf_counter()
    dictionaries = [{} for _ in range(num_links)]
    unique_masks = [[] for _ in range(num_links)]
    references = []
    for links in masks:
        refs = []
        for link_idx, payload in enumerate(links):
            entries = dictionaries[link_idx]
            mask_idx = entries.get(payload)
            if mask_idx is None:
                mask_idx = len(entries)
                entries[payload] = mask_idx
                unique_masks[link_idx].append(payload)
            refs.append(mask_idx)
        references.append(refs)
    dedup_ms = (time.perf_counter() - start) * 1000
    output = Path(args.output_prefix + '.voxshared')
    with output.open('wb') as stream:
        stream.write(struct.pack('<8sIIIIf', b'VOXSHAR1', *config))
        for entries in unique_masks:
            stream.write(struct.pack('<I', len(entries)))
            for payload in entries:
                stream.write(struct.pack('<I', len(payload) // 12))
                stream.write(payload)
        for pose, refs in zip(poses, references):
            stream.write(pose)
            stream.write(struct.pack('<' + 'I' * num_links, *refs))
    # 保存物の再読込みによる、全姿勢・全リンク占有のバイト一致確認
    restored = output.read_bytes()
    assert struct.unpack_from('<8sIIIIf', restored) == (b'VOXSHAR1', *config)
    offset = 28
    restored_masks = []
    for link_idx in range(num_links):
        num_masks = struct.unpack_from('<I', restored, offset)[0]
        offset += 4
        entries = []
        for mask_idx in range(num_masks):
            num_cells = struct.unpack_from('<I', restored, offset)[0]
            offset += 4
            entries.append(restored[offset:offset + num_cells * 12])
            offset += num_cells * 12
        restored_masks.append(entries)
    pose_size = 4 + 4 * (angle_dim + 3 * coord_layers)
    for node_idx in range(num_nodes):
        assert restored[offset:offset + pose_size] == poses[node_idx]
        offset += pose_size
        refs = struct.unpack_from('<' + 'I' * num_links, restored, offset)
        offset += num_links * 4
        for link_idx, mask_idx in enumerate(refs):
            assert restored_masks[link_idx][mask_idx] == masks[node_idx][link_idx]
    assert offset == len(restored)
    original_refs = sum(len(payload) // 12 for links in masks for payload in links)
    shared_refs = sum(len(payload) // 12 for entries in unique_masks for payload in entries)
    metrics = dict(num_nodes=num_nodes, num_links=num_links,
                   num_original_masks=num_nodes * num_links,
                   num_unique_masks=sum(map(len, unique_masks)),
                   num_original_cell_refs=original_refs, num_shared_cell_refs=shared_refs,
                   original_bytes=original_bytes, shared_bytes=len(restored),
                   shared_byte_ratio=len(restored) / original_bytes, dedup_ms=dedup_ms,
                   num_mask_mismatches=0, num_pose_mismatches=0)
    Path(args.output_prefix + '.metrics.json').write_text(json.dumps(metrics, indent=2))
    per_link = [dict(link_idx=link_idx, num_unique_masks=len(entries),
                     original_cells=sum(len(links[link_idx]) // 12 for links in masks),
                     shared_cells=sum(len(payload) // 12 for payload in entries))
                for link_idx, entries in enumerate(unique_masks)]
    Path(args.output_prefix + '.links.json').write_text(json.dumps(per_link, indent=2))
    print(json.dumps(metrics), flush=True)


if __name__ == '__main__':
    main()
