import { useLayoutEffect, useMemo, useRef } from 'react';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';
import { GraphCluster, GraphData, LayerSettings, LAYER_COLORS } from '../../types';
import { ArrowBatch, use_click_pick } from './SharedRenderers';
import { arrow_sample, velocity_arrow_style } from './arrows';
import { get_active_node_labels, resolve_node_label } from './graphLayerSettings';

interface cluster_instance {
    cluster: GraphCluster;
    color: string;
}

// 形状・不透明度ごとの一括描画。個数の揺れに対して既存の確保容量を再利用
function ClusterMesh({ instances, is_human, opacity, graph, source_id, on_pick }: {
    instances: cluster_instance[];
    is_human: boolean;
    opacity: number;
    graph: GraphData;
    source_id: string;
    on_pick?: (id: number) => void;
}) {
    const mesh_ref = useRef<THREE.InstancedMesh>(null);
    const capacity_ref = useRef(1);
    const capacity = Math.max(capacity_ref.current, 2 ** Math.ceil(Math.log2(Math.max(1, instances.length))));
    capacity_ref.current = capacity;
    const { invalidate } = useThree();
    const pick = use_click_pick(on_pick ? event => {
        const instance = instances[event.instanceId ?? -1];
        if (instance) on_pick(instance.cluster.id);
    } : undefined);
    useLayoutEffect(() => {
        const mesh = mesh_ref.current;
        if (!mesh) return;
        const position = new THREE.Vector3(), scale = new THREE.Vector3();
        const rotation = new THREE.Quaternion(), matrix = new THREE.Matrix4(), color = new THREE.Color();
        const human_rotation = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2);
        for (let idx = 0; idx < instances.length; ++idx) {
            const { cluster, color: instance_color } = instances[idx];
            position.fromArray(cluster.pos);
            rotation.fromArray(cluster.quat);
            if (is_human) {
                rotation.multiply(human_rotation);
                scale.set(cluster.scale[0], cluster.scale[2], cluster.scale[1]);
            } else scale.fromArray(cluster.scale);
            mesh.setMatrixAt(idx, matrix.compose(position, rotation, scale));
            mesh.setColorAt(idx, color.set(instance_color));
        }
        mesh.count = instances.length;
        mesh.instanceMatrix.needsUpdate = true;
        if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
        mesh.computeBoundingSphere();
        invalidate();
    }, [instances, is_human, capacity, invalidate]);
    return <instancedMesh key={capacity} ref={mesh_ref} args={[undefined, undefined, capacity]}
        count={0} visible={instances.length > 0} frustumCulled={false} {...pick}
        userData={{ inspection_source: on_pick ? source_id : undefined, inspection_revision: graph,
            pick_clusters: instances.map(instance => instance.cluster) }}>
        {is_human ? <cylinderGeometry args={[0.5, 0.5, 1, 16]} /> : <boxGeometry args={[1, 1, 1]} />}
        <meshBasicMaterial transparent opacity={opacity} depthWrite={false} side={THREE.DoubleSide} />
    </instancedMesh>;
}

const velocity_style = velocity_arrow_style(1);

export function ClusterBatch({ graph, source_id, visible_labels, active_labels, node_opacity,
    selected_cluster_id, enable_velocity, on_select }: {
    graph: GraphData;
    source_id: string;
    visible_labels: LayerSettings['visibleLabels'];
    active_labels: ReturnType<typeof get_active_node_labels>;
    node_opacity: number;
    selected_cluster_id: number | null;
    enable_velocity: boolean;
    on_select?: (id: number | null) => void;
}) {
    const { batches, velocity_samples } = useMemo(() => {
        const batches: cluster_instance[][] = [[], [], [], []];
        const velocity_samples: arrow_sample[] = [];
        for (const cluster of graph.clusters) {
            if (visible_labels && !visible_labels[cluster.label as 0 | 1 | 2 | 3 | 4 | 5]) continue;
            const is_selected = selected_cluster_id === cluster.id;
            const is_human = cluster.label === 4;
            const color = is_selected ? '#FFFFFF' :
                (resolve_node_label({ semanticLabel: cluster.semanticLabel }, active_labels)?.color ||
                    LAYER_COLORS[cluster.label % LAYER_COLORS.length]);
            batches[(is_selected ? 2 : 0) + (is_human ? 1 : 0)].push({ cluster, color });
            if (enable_velocity) {
                const style = velocity_arrow_style(Math.hypot(...cluster.velocity));
                velocity_samples.push({ position: cluster.pos, direction: cluster.velocity,
                    length: style.length, head_length: style.head_length });
            }
        }
        return { batches, velocity_samples };
    }, [graph.clusters, visible_labels, active_labels, selected_cluster_id, enable_velocity]);
    return <group name="cluster-batch">
        {batches.map((instances, idx) => <ClusterMesh key={idx} instances={instances} is_human={idx % 2 === 1}
            opacity={idx >= 2 ? 0.1 : 0.3 * node_opacity} graph={graph} source_id={source_id}
            on_pick={on_select ? id => on_select(id === selected_cluster_id ? null : id) : undefined} />)}
        {enable_velocity && <ArrowBatch samples={velocity_samples} style={velocity_style} />}
    </group>;
}
