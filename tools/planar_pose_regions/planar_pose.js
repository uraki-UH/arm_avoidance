/* 平面アームの占有差分・姿勢領域の独立検証基盤 */
"use strict";

const planar_pose = (() => {
  const degree_scale = Math.PI / 180;
  const link_lengths = {
    3: [0.43, 0.36, 0.28],
    4: [0.36, 0.31, 0.25, 0.20],
    5: [0.31, 0.28, 0.24, 0.20, 0.16],
  };

  function create_config(options = {}) {
    const num_joints = options.num_joints ?? 3;
    if (![3, 4, 5].includes(num_joints)) throw new Error("軸数は3・4・5軸");
    const axis_x = options.axis_x ?? 0;
    const axis_y = options.axis_y ?? num_joints - 1;
    if (![axis_x, axis_y].every((axis) => Number.isInteger(axis) && axis >= 0 && axis < num_joints) || axis_x === axis_y) {
      throw new Error("表示軸は異なる2関節");
    }
    const resolution = options.resolution ?? 0.04;
    const max_num_nodes = options.max_num_nodes ?? 256;
    const max_cell_change_th = options.max_cell_change_th ?? 60;
    const split_method = options.split_method ?? "envelope";
    if (!["envelope", "local_sweep"].includes(split_method)) throw new Error("分割方式は全候補包絡・局所掃引モデル");
    if (!Number.isFinite(resolution) || resolution < 0.015 || resolution > 0.1) throw new Error("セル幅の範囲は0.015～0.1 m");
    if (!Number.isInteger(max_num_nodes) || max_num_nodes < 4 || max_num_nodes > 1024) throw new Error("ノード数の範囲は4～1024");
    if (!Number.isFinite(max_cell_change_th) || max_cell_change_th < 0) throw new Error("占有差の目安は非負数");
    const base_angles_deg = (options.base_angles_deg ?? [20, -40, 35, -25, 20]).slice(0, num_joints);
    if (base_angles_deg.length !== num_joints || !base_angles_deg.every(Number.isFinite)) throw new Error("関節角度の個数・数値");
    const lengths = [...link_lengths[num_joints]];
    const radii = lengths.map(() => 0.035);
    const has_round_tool = options.has_round_tool ?? false;
    if (has_round_tool) {
      lengths[num_joints - 1] = 0;
      radii[num_joints - 1] = 0.085;
    }
    const min_angles_deg = options.min_angles_deg ?? new Array(num_joints).fill(options.min_angle_deg ?? -90);
    const max_angles_deg = options.max_angles_deg ?? new Array(num_joints).fill(options.max_angle_deg ?? 90);
    if (min_angles_deg.length !== num_joints || max_angles_deg.length !== num_joints ||
        !min_angles_deg.every((value, idx) => Number.isFinite(value) && Number.isFinite(max_angles_deg[idx]) &&
          value >= -180 && max_angles_deg[idx] <= 180 && value < max_angles_deg[idx])) {
      throw new Error("各関節の可動域は−180～180°の非空区間");
    }
    const world_half_size = 1.4;
    const grid_width = Math.ceil(2 * world_half_size / resolution);
    return {
      num_joints, axis_x, axis_y, resolution, max_num_nodes, max_cell_change_th, split_method,
      base_angles_deg, lengths, radii, has_round_tool, world_half_size, grid_width,
      min_angles_deg: [...min_angles_deg], max_angles_deg: [...max_angles_deg], region_scope: "all_joints",
    };
  }

  function forward_kinematics(config, angles_deg) {
    const points = [[0, 0]];
    let angle = 0;
    for (let idx = 0; idx < config.num_joints; idx += 1) {
      angle += angles_deg[idx] * degree_scale;
      const previous = points[points.length - 1];
      points.push([
        previous[0] + config.lengths[idx] * Math.cos(angle),
        previous[1] + config.lengths[idx] * Math.sin(angle),
      ]);
    }
    return { points, tool_angle: angle };
  }

  function point_segment_dist_squared(x, y, first, second) {
    const delta_x = second[0] - first[0];
    const delta_y = second[1] - first[1];
    const squared_length = delta_x * delta_x + delta_y * delta_y;
    const fraction = squared_length === 0 ? 0 : Math.max(0, Math.min(1,
      ((x - first[0]) * delta_x + (y - first[1]) * delta_y) / squared_length));
    const diff_x = x - first[0] - fraction * delta_x;
    const diff_y = y - first[1] - fraction * delta_y;
    return diff_x * diff_x + diff_y * diff_y;
  }

  /* セル中心とリンクのカプセル形状による占有判定 */
  function rasterize_pose(config, angles_deg) {
    const kinematics = forward_kinematics(config, angles_deg);
    const ids = new Set();
    const { resolution, world_half_size, grid_width } = config;
    for (let idx = 0; idx < config.num_joints; idx += 1) {
      const first = kinematics.points[idx];
      const second = kinematics.points[idx + 1];
      const radius = config.radii[idx];
      const min_x = Math.max(0, Math.floor((Math.min(first[0], second[0]) - radius + world_half_size) / resolution));
      const max_x = Math.min(grid_width - 1, Math.floor((Math.max(first[0], second[0]) + radius + world_half_size) / resolution));
      const min_y = Math.max(0, Math.floor((Math.min(first[1], second[1]) - radius + world_half_size) / resolution));
      const max_y = Math.min(grid_width - 1, Math.floor((Math.max(first[1], second[1]) + radius + world_half_size) / resolution));
      for (let y_idx = min_y; y_idx <= max_y; y_idx += 1) {
        const y = (y_idx + 0.5) * resolution - world_half_size;
        for (let x_idx = min_x; x_idx <= max_x; x_idx += 1) {
          const x = (x_idx + 0.5) * resolution - world_half_size;
          if (point_segment_dist_squared(x, y, first, second) <= radius * radius) ids.add(y_idx * grid_width + x_idx);
        }
      }
    }
    return { angles_deg: [...angles_deg], ...kinematics, ids: [...ids].sort((first, second) => first - second) };
  }

  function union_ids(collections) {
    const result = new Set();
    for (const ids of collections) for (const id of ids) result.add(id);
    return [...result].sort((first, second) => first - second);
  }

  function intersect_ids(first, second) {
    const result = [];
    let first_idx = 0;
    let second_idx = 0;
    while (first_idx < first.length && second_idx < second.length) {
      if (first[first_idx] === second[second_idx]) {
        result.push(first[first_idx]);
        first_idx += 1;
        second_idx += 1;
      } else if (first[first_idx] < second[second_idx]) first_idx += 1;
      else second_idx += 1;
    }
    return result;
  }

  function subtract_ids(first, second) {
    const result = [];
    let second_idx = 0;
    for (const id of first) {
      while (second_idx < second.length && second[second_idx] < id) second_idx += 1;
      if (second[second_idx] !== id) result.push(id);
    }
    return result;
  }

  function symmetric_diff_count(first, second) {
    return first.length + second.length - 2 * intersect_ids(first, second).length;
  }

  /* リンク別輪郭の法線移動量による局所掃引近似。内部境界も加算、有限変位の誤差あり */
  function build_local_model(config, angles_deg, enable_boundary_rows = true) {
    const pose = forward_kinematics(config, angles_deg);
    const orientations = cumulative_angles(angles_deg).map((value) => value * degree_scale);
    const orientation_velocities = config.lengths.map((length, idx) => [-length * Math.sin(orientations[idx]), length * Math.cos(orientations[idx])]);
    /* 単独リンク回転・下流リンク並進の閉形式による累積角方向の面積係数 [m²/deg] */
    const orientation_weights = config.lengths.map((length, axis) => {
      let area = length * length / 2 + length * config.radii[axis];
      for (let idx = axis + 1; idx < config.num_joints; idx += 1) {
        area += length * (config.lengths[idx] * Math.abs(Math.cos(orientations[idx] - orientations[axis])) + 2 * config.radii[idx]);
      }
      return area * degree_scale;
    });
    const model = { reference_angles_deg: [...angles_deg], points: pose.points, orientations, orientation_velocities, orientation_weights };
    if (!enable_boundary_rows) return { ...model, rows: null };
    const rows = [];
    for (let link_idx = 0; link_idx < config.num_joints; link_idx += 1) {
      const angle = orientations[link_idx];
      const direction = [Math.cos(angle), Math.sin(angle)];
      const length = config.lengths[link_idx];
      const radius = config.radii[link_idx];
      function append_row(position, normal, weight) {
        const orientation_coefficients = new Array(config.num_joints).fill(0);
        for (let axis = 0; axis < link_idx; axis += 1) {
          orientation_coefficients[axis] = normal[0] * orientation_velocities[axis][0] + normal[1] * orientation_velocities[axis][1];
        }
        const offset = position.map((value, axis) => value - pose.points[link_idx][axis]);
        const rotation_value = -normal[0] * offset[1] + normal[1] * offset[0];
        orientation_coefficients[link_idx] = Math.abs(rotation_value) < 1e-12 ? 0 : rotation_value;
        const coefficients = orientation_coefficients.map((_, axis) =>
          orientation_coefficients.slice(axis).reduce((total, value) => total + value, 0));
        rows.push({ coefficients, orientation_coefficients, weight });
      }
      for (const side of [-1, 1]) {
        const normal = [-direction[1] * side, direction[0] * side];
        for (let idx = 0; idx < 4; idx += 1) {
          const position = pose.points[link_idx].map((value, axis) =>
            value + direction[axis] * length * (idx + 0.5) / 4 + normal[axis] * radius);
          if (length > 0) append_row(position, normal, length / 4);
        }
      }
      for (const endpoint of [0, 1]) {
        for (let idx = 0; idx < 8; idx += 1) {
          const normal_angle = angle + (endpoint === 0 ? Math.PI / 2 : -Math.PI / 2) + Math.PI * (idx + 0.5) / 8;
          const normal = [Math.cos(normal_angle), Math.sin(normal_angle)];
          const position = pose.points[link_idx].map((value, axis) => value + direction[axis] * length * endpoint + normal[axis] * radius);
          append_row(position, normal, Math.PI * radius / 8);
        }
      }
    }
    return { ...model, rows };
  }

  /* 輪郭法線の離散積分による局所掃引面積 [m²]。衝突判定・包絡保証とは別用途 */
  function estimate_local_sweep(model, delta_angles_deg) {
    if (!model.rows) throw new Error("輪郭点のない局所モデル");
    return model.rows.reduce((total, row) => total + 0.5 * row.weight * Math.abs(
      row.coefficients.reduce((value, coefficient, idx) => value + coefficient * delta_angles_deg[idx] * degree_scale, 0)), 0);
  }

  function model_span_area(model, bounds) {
    return model.orientation_weights.reduce((total, weight, idx) =>
      total + weight * (bounds.max_orientations_deg[idx] - bounds.min_orientations_deg[idx]), 0);
  }

  /* 表示用の極値姿勢と一次運動モデルの端点誤差 [m] */
  function local_linear_error(config, cell, model) {
    let max_error = 0;
    for (const pose of cell.sample_poses) {
      const values = cumulative_angles(pose.angles_deg);
      const displacement = [0, 0];
      for (let idx = 0; idx < config.num_joints; idx += 1) {
        const delta = values[idx] * degree_scale - model.orientations[idx];
        displacement[0] += model.orientation_velocities[idx][0] * delta;
        displacement[1] += model.orientation_velocities[idx][1] * delta;
        max_error = Math.max(max_error, Math.hypot(
          pose.points[idx + 1][0] - model.points[idx + 1][0] - displacement[0],
          pose.points[idx + 1][1] - model.points[idx + 1][1] - displacement[1]));
      }
    }
    return max_error;
  }

  function angles_at(config, x, y) {
    const result = [...config.base_angles_deg];
    result[config.axis_x] = x;
    result[config.axis_y] = y;
    return result;
  }

  function cumulative_angles(angles_deg) {
    let total = 0;
    return angles_deg.map((value) => (total += value));
  }

  function joint_angles(orientations_deg) {
    return orientations_deg.map((value, idx) => value - (idx === 0 ? 0 : orientations_deg[idx - 1]));
  }

  /* θ0=0、θi=累積関節角。matrix[i][j] は θj−θi の上限 [deg] */
  function close_constraints(matrix) {
    const result = matrix.map((row) => [...row]);
    for (let middle = 0; middle < result.length; middle += 1) {
      for (let first = 0; first < result.length; first += 1) {
        for (let second = 0; second < result.length; second += 1) {
          result[first][second] = Math.min(result[first][second], result[first][middle] + result[middle][second]);
        }
      }
    }
    return result.some((row, idx) => row[idx] < -1e-8) ? null : result;
  }

  function constrain_difference(matrix, first, second, min_value, max_value) {
    matrix[first][second] = Math.min(matrix[first][second], max_value);
    matrix[second][first] = Math.min(matrix[second][first], -min_value);
  }

  /* 各差分方向の極値姿勢の平均。実行可能領域の相対内部に位置する基準姿勢 */
  function reference_orientations(matrix) {
    const result = new Array(matrix.length - 1).fill(0);
    for (let anchor = 0; anchor < matrix.length; anchor += 1) {
      for (let idx = 1; idx < matrix.length; idx += 1) {
        result[idx - 1] += matrix[anchor][idx] - matrix[anchor][0] - matrix[idx][anchor] + matrix[0][anchor];
      }
    }
    return result.map((value) => value / (2 * matrix.length));
  }

  function bounds_from_matrix(matrix, coordinate_kind) {
    if (!matrix) return null;
    const num_joints = matrix.length - 1;
    return {
      constraint_matrix: matrix, coordinate_kind,
      min_angles_deg: Array.from({ length: num_joints }, (_, idx) => -matrix[idx + 1][idx]),
      max_angles_deg: Array.from({ length: num_joints }, (_, idx) => matrix[idx][idx + 1]),
      min_orientations_deg: matrix[0].slice(1).map((_, idx) => -matrix[idx + 1][0]),
      max_orientations_deg: matrix[0].slice(1),
      center_angles_deg: joint_angles(reference_orientations(matrix)),
    };
  }

  function region_bounds(config, bounds) {
    if (bounds.constraint_matrix) return bounds;
    const matrix = Array.from({ length: config.num_joints + 1 }, (_, idx) =>
      Array.from({ length: config.num_joints + 1 }, (_, other) => idx === other ? 0 : Infinity));
    for (let idx = 0; idx < config.num_joints; idx += 1) {
      constrain_difference(matrix, idx, idx + 1, bounds.min_angles_deg[idx], bounds.max_angles_deg[idx]);
    }
    return bounds_from_matrix(close_constraints(matrix), bounds.coordinate_kind ?? "joint_angle");
  }

  function constrain_orientation(bounds, axis, min_value, max_value) {
    const matrix = bounds.constraint_matrix.map((row) => [...row]);
    constrain_difference(matrix, 0, axis + 1, min_value, max_value);
    return bounds_from_matrix(close_constraints(matrix), bounds.coordinate_kind);
  }

  function constrain_joint(bounds, axis, min_value, max_value) {
    const matrix = bounds.constraint_matrix.map((row) => [...row]);
    constrain_difference(matrix, axis, axis + 1, min_value, max_value);
    return bounds_from_matrix(close_constraints(matrix), bounds.coordinate_kind);
  }

  function max_interval_cos(min_angle, max_angle) {
    if (Math.ceil(min_angle / (2 * Math.PI)) <= Math.floor(max_angle / (2 * Math.PI))) return 1;
    return Math.max(Math.cos(min_angle), Math.cos(max_angle));
  }

  const support_directions = Array.from({ length: 64 }, (_, idx) => {
    const angle = idx * 2 * Math.PI / 64;
    return { angle, x: Math.cos(angle), y: Math.sin(angle) };
  });

  /* |Σ Li ui|² の内積展開による端点到達距離の上界。全角度組み合わせの列挙なし */
  function endpoint_reach_bounds(config, bounds) {
    const result = [0];
    let squared_reach = 0;
    for (let idx = 0; idx < config.num_joints; idx += 1) {
      const length = config.lengths[idx];
      squared_reach += length * length;
      for (let previous = 0; previous < idx; previous += 1) {
        const min_angle = -bounds.constraint_matrix[idx + 1][previous + 1] * degree_scale;
        const max_angle = bounds.constraint_matrix[previous + 1][idx + 1] * degree_scale;
        squared_reach += 2 * length * config.lengths[previous] * max_interval_cos(min_angle, max_angle);
      }
      result.push(Math.sqrt(Math.max(0, squared_reach)) + 1e-10);
    }
    return result;
  }

  /* 共通回転と相対角区間による端点支持の上界。基準ベクトル＋角度変動残差 */
  function correlated_endpoint_supports(config, bounds, last_idx, supports) {
    if (last_idx === 0) return supports;
    let selected = null;
    for (let anchor = 0; anchor <= last_idx; anchor += 1) {
      let x = 0;
      let y = 0;
      let residual = 0;
      for (let idx = 0; idx <= last_idx; idx += 1) {
        const min_angle = -bounds.constraint_matrix[idx + 1][anchor + 1] * degree_scale;
        const max_angle = bounds.constraint_matrix[anchor + 1][idx + 1] * degree_scale;
        const middle = (min_angle + max_angle) / 2;
        x += config.lengths[idx] * Math.cos(middle);
        y += config.lengths[idx] * Math.sin(middle);
        residual += 2 * config.lengths[idx] * Math.sin(Math.min(Math.PI / 2, (max_angle - min_angle) / 4));
      }
      if (!selected || residual < selected.residual) selected = { anchor, x, y, residual };
    }
    const { anchor, x, y, residual } = selected;
    const length = Math.hypot(x, y);
    const offset = Math.atan2(y, x);
    const min_angle = bounds.min_orientations_deg[anchor] * degree_scale + offset;
    const max_angle = bounds.max_orientations_deg[anchor] * degree_scale + offset;
    return supports.map((value, idx) => Math.min(value, length * max_interval_cos(
      min_angle - support_directions[idx].angle, max_angle - support_directions[idx].angle) + residual + 1e-10));
  }

  /* リンク向きの連動制約からの支持半平面包絡。起点の円膨張・全姿勢列挙なし */
  function build_region_envelope(config, input_bounds) {
    const bounds = region_bounds(config, input_bounds);
    if (!bounds) throw new Error("空の姿勢領域");
    const { resolution, world_half_size, grid_width } = config;
    const occupied = new Uint8Array(grid_width * grid_width);
    const origin_supports = new Array(support_directions.length).fill(0);
    const primitives = [];
    const reach_bounds = endpoint_reach_bounds(config, bounds);
    let straight_reach = 0;
    for (let link_idx = 0; link_idx < config.num_joints; link_idx += 1) {
      const length = config.lengths[link_idx];
      const radius = config.radii[link_idx];
      const min_orientation = bounds.min_orientations_deg[link_idx] * degree_scale;
      const max_orientation = bounds.max_orientations_deg[link_idx] * degree_scale;
      const endpoint_supports = support_directions.map((direction) =>
        length * max_interval_cos(min_orientation - direction.angle, max_orientation - direction.angle));
      straight_reach += length;
      const independent_supports = endpoint_supports.map((value, idx) => Math.min(origin_supports[idx] + value, reach_bounds[link_idx + 1]));
      /* 到達距離の縮小がセル幅相当の領域のみ、連動支持の追加計算 */
      const tight_endpoint_supports = reach_bounds[link_idx + 1] + resolution < straight_reach ?
        correlated_endpoint_supports(config, bounds, link_idx, independent_supports) : independent_supports;
      /* 始点・終点の支持上界の凸結合。折りたたみの連動角による過大包絡の抑制 */
      const supports = tight_endpoint_supports.map((value, idx) => Math.max(
        Math.min(origin_supports[idx], reach_bounds[link_idx]),
        value) + radius + 1e-10);
      const max_reach = Math.max(reach_bounds[link_idx], reach_bounds[link_idx + 1]) + radius;
      const min_y = Math.max(0, Math.ceil((-supports[48] + world_half_size) / resolution - 0.5));
      const max_y = Math.min(grid_width - 1, Math.floor((supports[16] + world_half_size) / resolution - 0.5));
      for (let y_idx = min_y; y_idx <= max_y; y_idx += 1) {
        const y = (y_idx + 0.5) * resolution - world_half_size;
        if (Math.abs(y) > max_reach + 1e-10) continue;
        const radial_extent = Math.sqrt(Math.max(0, max_reach * max_reach - y * y));
        let min_x = -radial_extent;
        let max_x = radial_extent;
        for (let idx = 0; idx < support_directions.length; idx += 1) {
          const direction = support_directions[idx];
          if (direction.x > 1e-10) max_x = Math.min(max_x, (supports[idx] - direction.y * y) / direction.x);
          else if (direction.x < -1e-10) min_x = Math.max(min_x, (supports[idx] - direction.y * y) / direction.x);
        }
        const first = Math.max(0, Math.ceil((min_x + world_half_size) / resolution - 0.5 - 1e-9));
        const last = Math.min(grid_width - 1, Math.floor((max_x + world_half_size) / resolution - 0.5 + 1e-9));
        if (first <= last) occupied.fill(1, y_idx * grid_width + first, y_idx * grid_width + last + 1);
      }
      primitives.push({
        type: "support_halfplanes", supports, max_reach,
        min_orientation_deg: bounds.min_orientations_deg[link_idx],
        max_orientation_deg: bounds.max_orientations_deg[link_idx],
      });
      for (let idx = 0; idx < origin_supports.length; idx += 1) origin_supports[idx] = tight_endpoint_supports[idx];
    }
    const ids = [];
    for (let id = 0; id < occupied.length; id += 1) if (occupied[id]) ids.push(id);
    return { ids, primitives };
  }

  /* 全関節姿勢・同一格子の共通キャッシュ。円形ツールの占有不変回転も共有対象 */
  function create_registry(config) {
    const poses = new Map();
    const geometries = new Map();
    const envelopes = new Map();
    let num_cache_hits = 0;
    let num_envelope_cache_hits = 0;
    let num_registered_envelopes = 0;
    function register(angles_deg) {
      const key = angles_deg.join(",");
      if (poses.has(key)) {
        num_cache_hits += 1;
        return poses.get(key);
      }
      const geometry_key = (config.has_round_tool ? angles_deg.slice(0, -1) : angles_deg).join(",");
      if (!geometries.has(geometry_key)) geometries.set(geometry_key, rasterize_pose(config, angles_deg));
      const geometry = geometries.get(geometry_key);
      const pose = config.has_round_tool ? {
        angles_deg: [...angles_deg], ids: geometry.ids, points: geometry.points,
        tool_angle: angles_deg.reduce((total, value) => total + value, 0) * degree_scale,
      } : geometry;
      poses.set(key, pose);
      return pose;
    }
    function envelope_key(bounds) { return bounds.constraint_matrix.map((row) => row.join(",")).join("/"); }
    function register_envelope(input_bounds) {
      const bounds = region_bounds(config, input_bounds);
      const key = envelope_key(bounds);
      if (envelopes.has(key)) num_envelope_cache_hits += 1;
      else {
        envelopes.set(key, build_region_envelope(config, bounds));
        num_registered_envelopes += 1;
      }
      return envelopes.get(key);
    }
    function retain_envelopes(cells) {
      const retained = new Set(cells.map(envelope_key));
      for (const key of envelopes.keys()) if (!retained.has(key)) envelopes.delete(key);
    }
    return {
      config, register, register_envelope, retain_envelopes,
      get num_registered_poses() { return poses.size; },
      get num_pose_rasterizations() { return geometries.size; },
      get num_cache_hits() { return num_cache_hits; },
      get num_registered_envelopes() { return num_registered_envelopes; },
      get num_envelope_cache_hits() { return num_envelope_cache_hits; },
    };
  }

  /* 一次制約 min_dev <= A(q−q0) <= max_dev。累積角の交差係数による関節連動 */
  function evaluate_cell(registry, input_bounds) {
    const config = registry.config;
    const bounds = region_bounds(config, input_bounds);
    if (!bounds) throw new Error("空の姿勢領域");
    const center_angles_deg = bounds.center_angles_deg;
    const sample_angles = [center_angles_deg];
    for (let idx = 0; idx < config.num_joints; idx += 1) {
      for (const value of [bounds.min_orientations_deg[idx], bounds.max_orientations_deg[idx]]) {
        sample_angles.push(constrain_orientation(bounds, idx, value, value).center_angles_deg);
      }
    }
    sample_angles.push(joint_angles(bounds.min_orientations_deg), joint_angles(bounds.max_orientations_deg));
    const sample_poses = sample_angles.map(registry.register);
    const sampled_occupied_ids = union_ids(sample_poses.map((pose) => pose.ids));
    const envelope = registry.register_envelope(bounds);
    const extent = Math.max(...sample_poses[0].points.map((point) => Math.hypot(...point)));
    const total_length = config.lengths.reduce((total, length) => total + length, 0);
    const change_cells = envelope.ids.length - sample_poses[0].ids.length;
    const is_coupled = bounds.coordinate_kind === "link_orientation";
    const constraints = center_angles_deg.map((value, idx) => ({
      coefficients: center_angles_deg.map((_, axis) => axis === idx ? 1 : 0),
      min_dev_deg: bounds.min_angles_deg[idx] - value, max_dev_deg: bounds.max_angles_deg[idx] - value,
    }));
    const reference_angles = cumulative_angles(center_angles_deg);
    const orientation_constraints = reference_angles.map((value, idx) => ({
      coefficients: center_angles_deg.map((_, axis) => axis <= idx ? 1 : 0),
      min_dev_deg: bounds.min_orientations_deg[idx] - value, max_dev_deg: bounds.max_orientations_deg[idx] - value,
    }));
    return {
      ...bounds,
      center_angles_deg, sample_poses, representative: sample_poses[0],
      region: {
        type: is_coupled ? "coupled_linear_joint_region" : "linear_joint_box",
        reference_angles_deg: center_angles_deg,
        min_dev_angles_deg: bounds.min_angles_deg.map((value, idx) => value - center_angles_deg[idx]),
        max_dev_angles_deg: bounds.max_angles_deg.map((value, idx) => value - center_angles_deg[idx]),
        constraints: is_coupled ? [...constraints, ...orientation_constraints] : constraints,
        coordinate_constraints: is_coupled ? orientation_constraints : constraints,
      },
      occupied_ids: envelope.ids, envelope_primitives: envelope.primitives, sampled_occupied_ids,
      change_cells,
      split_priority: Math.round(change_cells * Math.sqrt(total_length / Math.max(config.resolution, extent)) /
        Math.sqrt(Math.max(1, sample_poses[0].ids.length)) * 1e8) / 1e8,
      fold_cut: fold_partition_cut(config, bounds),
    };
  }

  function contains_pose(cell, angles_deg) {
    if (angles_deg.length !== cell.min_angles_deg.length || !angles_deg.every(Number.isFinite)) return false;
    const values = [0, ...cumulative_angles(angles_deg)];
    return cell.constraint_matrix.every((row, first) => row.every((bound, second) => values[second] - values[first] <= bound + 1e-8));
  }

  function project_cell(config, cell) {
    return {
      min_x: cell.min_angles_deg[config.axis_x], max_x: cell.max_angles_deg[config.axis_x],
      min_y: cell.min_angles_deg[config.axis_y], max_y: cell.max_angles_deg[config.axis_y],
      center_x: cell.center_angles_deg[config.axis_x], center_y: cell.center_angles_deg[config.axis_y],
    };
  }

  function matches_slice(config, cell, angles_deg) {
    const matrix = cell.constraint_matrix.map((row) => [...row]);
    angles_deg.forEach((value, idx) => {
      if (idx !== config.axis_x && idx !== config.axis_y) constrain_difference(matrix, idx, idx + 1, value, value);
    });
    return Boolean(close_constraints(matrix));
  }

  function projected_pose(config, cell, x, y, fixed_angles = null) {
    const matrix = cell.constraint_matrix.map((row) => [...row]);
    if (fixed_angles) fixed_angles.forEach((value, idx) => {
      if (idx !== config.axis_x && idx !== config.axis_y) constrain_difference(matrix, idx, idx + 1, value, value);
    });
    constrain_difference(matrix, config.axis_x, config.axis_x + 1, x, x);
    constrain_difference(matrix, config.axis_y, config.axis_y + 1, y, y);
    return bounds_from_matrix(close_constraints(matrix), cell.coordinate_kind)?.center_angles_deg ?? null;
  }

  /* 他関節固定時の正確な2軸断面。外接枠による偽の領域表示の回避 */
  function slice_polygon(config, cell, angles_deg) {
    const min_x = config.min_angles_deg[config.axis_x];
    const max_x = config.max_angles_deg[config.axis_x];
    const min_y = config.min_angles_deg[config.axis_y];
    const max_y = config.max_angles_deg[config.axis_y];
    let polygon = [[min_x, min_y], [max_x, min_y], [max_x, max_y], [min_x, max_y]];
    function clip(normal_x, normal_y, bound) {
      const next = [];
      for (let idx = 0; idx < polygon.length; idx += 1) {
        const first = polygon[idx];
        const second = polygon[(idx + 1) % polygon.length];
        const first_value = normal_x * first[0] + normal_y * first[1] - bound;
        const second_value = normal_x * second[0] + normal_y * second[1] - bound;
        const is_first_inside = first_value <= 1e-8;
        const is_second_inside = second_value <= 1e-8;
        if (is_first_inside) next.push(first);
        if (is_first_inside !== is_second_inside) {
          const fraction = first_value / (first_value - second_value);
          next.push([first[0] + (second[0] - first[0]) * fraction, first[1] + (second[1] - first[1]) * fraction]);
        }
      }
      polygon = next;
    }
    for (const constraint of cell.region.constraints) {
      const coefficients = constraint.coefficients;
      const reference = coefficients.reduce((total, value, idx) => total + value * cell.center_angles_deg[idx], 0);
      const hidden = coefficients.reduce((total, value, idx) =>
        idx === config.axis_x || idx === config.axis_y ? total : total + value * angles_deg[idx], 0);
      clip(coefficients[config.axis_x], coefficients[config.axis_y], constraint.max_dev_deg + reference - hidden);
      clip(-coefficients[config.axis_x], -coefficients[config.axis_y], -constraint.min_dev_deg - reference + hidden);
      if (!polygon.length) break;
    }
    return polygon;
  }

  function can_split(config, cell) {
    return config.lengths.some((length, idx) => length > 0 && cell.max_orientations_deg[idx] - cell.min_orientations_deg[idx] > 1);
  }

  /* 隣接2リンクの伸展長に対する到達距離80%の境界角 [deg] */
  function fold_boundary_angle(first, second) {
    if (first === 0 || second === 0) return null;
    const cos = ((0.8 * (first + second)) ** 2 - first * first - second * second) / (2 * first * second);
    return cos > -1 && cos < 1 ? Math.acos(cos) / degree_scale : null;
  }

  /* 近位2関節の同方向折りたたみの初期層別。全関節の直積分割なし */
  function fold_partition_cut(config, cell) {
    const first_angle = fold_boundary_angle(config.lengths[0], config.lengths[1]);
    for (let axis = 1; axis < Math.min(3, config.num_joints); axis += 1) {
      const first = config.lengths[axis - 1];
      const second = config.lengths[axis];
      const angle = fold_boundary_angle(first, second);
      if (angle === null) continue;
      for (const value of [-angle, angle]) {
        if (axis === 2 && (first_angle === null || (value < 0 ?
          cell.max_angles_deg[1] > -first_angle + 1e-8 : cell.min_angles_deg[1] < first_angle - 1e-8))) continue;
        if (value - cell.min_angles_deg[axis] > 1 && cell.max_angles_deg[axis] - value > 1) return { axis, value };
      }
    }
    return null;
  }

  /* 各候補の実包絡による分割後平均セル数の最小化。端点占有差による方向選択なし */
  function divide_cell(registry, cell) {
    if (cell.fold_cut) {
      const { axis, value } = cell.fold_cut;
      const children = [constrain_joint(cell, axis, cell.min_angles_deg[axis], value),
        constrain_joint(cell, axis, value, cell.max_angles_deg[axis])];
      const split = { axis, coordinate_kind: "joint", method: "fold_partition", value, candidates: [] };
      return children.map((child) => ({ ...evaluate_cell(registry, child), parent_split: split }));
    }
    let selected = null;
    const candidates = [];
    const config = registry.config;
    const num_active_axes = config.lengths.filter((length, idx) => length > 0 && cell.max_orientations_deg[idx] - cell.min_orientations_deg[idx] > 1).length;
    const has_local_model = config.split_method === "local_sweep" && num_active_axes > 2;
    const model = build_local_model(config, cell.center_angles_deg, false);
    const parent_area = model_span_area(model, cell);
    for (let axis = 0; axis < registry.config.num_joints; axis += 1) {
      const span = cell.max_orientations_deg[axis] - cell.min_orientations_deg[axis];
      if (span <= 1 || registry.config.lengths[axis] === 0) continue;
      const middle = (cell.min_orientations_deg[axis] + cell.max_orientations_deg[axis]) / 2;
      const children = [
        constrain_orientation(cell, axis, cell.min_orientations_deg[axis], middle),
        constrain_orientation(cell, axis, middle, cell.max_orientations_deg[axis]),
      ];
      if (children.some((child) => !child)) continue;
      candidates.push({ axis, coordinate_kind: "orientation", children, span, motion: span * config.lengths[axis],
        estimated_reduction_area: parent_area - children.reduce((total, child) => total + model_span_area(model, child), 0) / 2 });
    }
    /* 折りたたみ領域の相対関節角。閉形式による上位1方向のみ実包絡の追加評価 */
    for (let axis = 1; axis < config.num_joints; axis += 1) {
      const span = cell.max_angles_deg[axis] - cell.min_angles_deg[axis];
      if (span <= 1 || config.lengths[axis] === 0) continue;
      const fold_angle = fold_boundary_angle(config.lengths[axis - 1], config.lengths[axis]);
      if (fold_angle === null || (cell.min_angles_deg[axis] < fold_angle - 1e-8 && cell.max_angles_deg[axis] > -fold_angle + 1e-8)) continue;
      const middle = (cell.min_angles_deg[axis] + cell.max_angles_deg[axis]) / 2;
      const children = [constrain_joint(cell, axis, cell.min_angles_deg[axis], middle),
        constrain_joint(cell, axis, middle, cell.max_angles_deg[axis])];
      if (children.some((child) => !child)) continue;
      candidates.push({ axis, coordinate_kind: "joint", children, span, motion: span * config.lengths[axis],
        estimated_reduction_area: parent_area - children.reduce((total, child) => total + model_span_area(model, child), 0) / 2 });
    }
    const orientation_candidates = candidates.filter((item) => item.coordinate_kind === "orientation");
    const joint_candidate = candidates.filter((item) => item.coordinate_kind === "joint").sort((a, b) =>
      b.estimated_reduction_area - a.estimated_reduction_area || b.motion - a.motion)[0];
    let verified = [...orientation_candidates];
    const max_linear_error = has_local_model ? local_linear_error(config, cell, model) : 0;
    if (has_local_model && orientation_candidates.length > 2) {
      verified = [...orientation_candidates].sort((first, second) => second.estimated_reduction_area - first.estimated_reduction_area).slice(0, 2);
      if (max_linear_error > config.resolution) {
        const widest = orientation_candidates.reduce((first, second) => first.span >= second.span ? first : second);
        if (!verified.includes(widest)) verified.push(widest);
      }
    }
    if (joint_candidate && joint_candidate.estimated_reduction_area > config.resolution * config.resolution) verified.push(joint_candidate);
    for (const candidate of verified) {
      const sizes = candidate.children.map((child) => registry.register_envelope(child).ids.length);
      const mean_cells = (sizes[0] + sizes[1]) / 2;
      candidate.mean_cells = mean_cells;
      candidate.max_cells = Math.max(...sizes);
      if (!selected || mean_cells < selected.mean_cells ||
          mean_cells === selected.mean_cells && candidate.max_cells < selected.max_cells ||
          mean_cells === selected.mean_cells && candidate.max_cells === selected.max_cells && candidate.motion > selected.motion + 1e-8) {
        selected = candidate;
      }
    }
    if (!selected) return null;
    const split = { axis: selected.axis, coordinate_kind: selected.coordinate_kind, method: config.split_method, max_linear_error,
      reduction_cells: cell.occupied_ids.length - selected.mean_cells,
      candidates: candidates.map((item) => {
        const { children, ...candidate } = item;
        return { ...candidate, is_verified: verified.includes(item) };
      }) };
    return selected.children.map((child) => ({ ...evaluate_cell(registry, child), parent_split: split }));
  }

  /* 全関節の制約交差・共有面・実行可能な通過姿勢による接続。角点接触は対象外 */
  function connect_cells(cells) {
    const edges = [];
    const neighbors = cells.map(() => []);
    const tolerance = 1e-8;
    const num_joints = cells[0].min_angles_deg.length;
    const is_coupled = cells[0].coordinate_kind === "link_orientation";
    for (let first_idx = 0; first_idx < cells.length; first_idx += 1) {
      const first = cells[first_idx];
      first.id = first_idx;
      for (let second_idx = first_idx + 1; second_idx < cells.length; second_idx += 1) {
        const second = cells[second_idx];
        const port_angles_deg = [];
        let num_touching_axes = 0;
        let has_shared_face = true;
        for (let axis = 0; axis < num_joints; axis += 1) {
          const min_angle = is_coupled ? Math.max(first.min_orientations_deg[axis], second.min_orientations_deg[axis]) :
            Math.max(first.min_angles_deg[axis], second.min_angles_deg[axis]);
          const max_angle = is_coupled ? Math.min(first.max_orientations_deg[axis], second.max_orientations_deg[axis]) :
            Math.min(first.max_angles_deg[axis], second.max_angles_deg[axis]);
          if (max_angle - min_angle > tolerance) port_angles_deg.push((min_angle + max_angle) / 2);
          else if (Math.abs(max_angle - min_angle) <= tolerance) {
            num_touching_axes += 1;
            port_angles_deg.push((min_angle + max_angle) / 2);
          } else { has_shared_face = false; break; }
        }
        if (!has_shared_face || (!is_coupled && num_touching_axes !== 1)) continue;
        if (is_coupled) {
          /* 累積角・相対関節角の共有面候補。角点のみの接触は制約行列の次元で除外 */
          for (let axis = 0; axis < num_joints; axis += 1) {
            const span = Math.min(first.max_angles_deg[axis], second.max_angles_deg[axis]) -
              Math.max(first.min_angles_deg[axis], second.min_angles_deg[axis]);
            if (span < -tolerance) { has_shared_face = false; break; }
            if (Math.abs(span) <= tolerance) num_touching_axes += 1;
          }
          if (!has_shared_face || num_touching_axes === 0) continue;
          const intersection = close_constraints(first.constraint_matrix.map((row, idx) =>
            row.map((value, other) => Math.min(value, second.constraint_matrix[idx][other]))));
          if (!intersection) continue;
          const groups = Array.from({ length: num_joints + 1 }, (_, idx) => idx);
          function group(idx) { while (groups[idx] !== idx) idx = groups[idx]; return idx; }
          for (let idx = 0; idx <= num_joints; idx += 1) {
            for (let other = idx + 1; other <= num_joints; other += 1) {
              if (intersection[idx][other] + intersection[other][idx] <= tolerance) groups[group(other)] = group(idx);
            }
          }
          if (new Set(groups.map((_, idx) => group(idx))).size !== num_joints) continue;
          port_angles_deg.splice(0, port_angles_deg.length, ...joint_angles(reference_orientations(intersection)));
        }
        const edge_idx = edges.length;
        edges.push({ first_idx, second_idx, port_angles_deg });
        neighbors[first_idx].push(edge_idx);
        neighbors[second_idx].push(edge_idx);
      }
    }
    return { cells, edges, neighbors };
  }

  async function build_comparison(config, hooks = {}) {
    const registry = create_registry(config);
    const start_ms = performance.now();
    const yield_work = hooks.yield_work ?? (() => new Promise((resolve) => setTimeout(resolve, 0)));
    async function checkpoint(stage, num_nodes) {
      if (hooks.can_continue && !hooks.can_continue()) throw new Error("構築の中断");
      if (hooks.on_progress) hooks.on_progress(stage, num_nodes);
      await yield_work();
      if (hooks.can_continue && !hooks.can_continue()) throw new Error("構築の中断");
    }
    const adaptive_cells = [evaluate_cell(registry, {
      min_angles_deg: [...config.min_angles_deg],
      max_angles_deg: [...config.max_angles_deg],
      coordinate_kind: "link_orientation",
    })];
    while (adaptive_cells.length < config.max_num_nodes) {
      let selected_idx = -1;
      let max_change = -Infinity;
      const has_fold_partition = adaptive_cells.some((cell) => cell.fold_cut);
      for (let idx = 0; idx < adaptive_cells.length; idx += 1) {
        const cell = adaptive_cells[idx];
        /* 占有セル数・姿勢サイズで正規化した包絡追加。折りたたみへの分割予算の配分 */
        const relative_change = cell.split_priority;
        if ((has_fold_partition ? cell.fold_cut : cell.change_cells > config.max_cell_change_th && can_split(config, cell)) && relative_change > max_change) {
          max_change = relative_change;
          selected_idx = idx;
        }
      }
      if (selected_idx === -1) break;
      const cell = adaptive_cells[selected_idx];
      const children = divide_cell(registry, cell);
      if (!children) break;
      adaptive_cells.splice(selected_idx, 1, ...children);
      if (adaptive_cells.length % 16 === 0) await checkpoint("全関節・占有適応", adaptive_cells.length);
    }
    await checkpoint("全関節・角度等間隔", 0);
    let num_divisions = Math.floor(config.max_num_nodes ** (1 / config.num_joints) + 1e-8);
    while (num_divisions ** config.num_joints > config.max_num_nodes) num_divisions -= 1;
    const num_uniform_nodes = num_divisions ** config.num_joints;
    const uniform_cells = [];
    for (let flat_idx = 0; flat_idx < num_uniform_nodes; flat_idx += 1) {
      let remainder = flat_idx;
      const min_angles_deg = [];
      const max_angles_deg = [];
      for (let axis = 0; axis < config.num_joints; axis += 1) {
        const coordinate = remainder % num_divisions;
        remainder = Math.floor(remainder / num_divisions);
        const step = (config.max_angles_deg[axis] - config.min_angles_deg[axis]) / num_divisions;
        min_angles_deg.push(config.min_angles_deg[axis] + coordinate * step);
        max_angles_deg.push(config.min_angles_deg[axis] + (coordinate + 1) * step);
      }
      uniform_cells.push(evaluate_cell(registry, { min_angles_deg, max_angles_deg }));
      if (uniform_cells.length % 16 === 0) await checkpoint("全関節・角度等間隔", uniform_cells.length);
    }
    const uniform = connect_cells(uniform_cells);
    const adaptive = connect_cells(adaptive_cells);
    registry.retain_envelopes([...uniform_cells, ...adaptive_cells]);
    return {
      config, registry, uniform, adaptive,
      stats: {
        elapsed_ms: performance.now() - start_ms,
        num_registered_poses: registry.num_registered_poses, num_pose_rasterizations: registry.num_pose_rasterizations,
        num_cache_hits: registry.num_cache_hits, num_registered_envelopes: registry.num_registered_envelopes,
        num_envelope_cache_hits: registry.num_envelope_cache_hits,
        is_budget_limited: adaptive_cells.some((cell) => cell.change_cells > config.max_cell_change_th && can_split(config, cell)),
      },
    };
  }

  /* 下流リンク長による運動量上界を用いた軌道サンプリング。連続衝突保証なし */
  function motion_bound(config, first_angles, second_angles) {
    let remaining_length = 0;
    let bound = 0;
    for (let idx = config.num_joints - 1; idx >= 0; idx -= 1) {
      remaining_length += config.lengths[idx];
      bound += remaining_length * Math.abs(second_angles[idx] - first_angles[idx]) * degree_scale;
    }
    return bound;
  }

  function interpolate_angles(first, second, fraction) {
    if (fraction === 0) return [...first];
    if (fraction === 1) return [...second];
    return first.map((value, idx) => value + (second[idx] - value) * fraction);
  }

  /* 有限軌道の占有サンプリング。登録先の差し替えによる動的比較と構築キャッシュの分離 */
  function sample_path(config, coordinates, register_pose) {
    const samples = [];
    const joint_travel_deg = new Array(config.num_joints).fill(0);
    for (let segment_idx = 0; segment_idx < coordinates.length - 1; segment_idx += 1) {
      const start = coordinates[segment_idx];
      const finish = coordinates[segment_idx + 1];
      finish.forEach((value, idx) => { joint_travel_deg[idx] += Math.abs(value - start[idx]); });
      const num_steps = Math.max(1, Math.ceil(motion_bound(config, start, finish) / (config.resolution * 0.5)));
      for (let step_idx = segment_idx === 0 ? 0 : 1; step_idx <= num_steps; step_idx += 1) {
        samples.push(register_pose(interpolate_angles(start, finish, step_idx / num_steps)));
      }
    }
    const occupied_ids = union_ids(samples.map((sample) => sample.ids));
    const first = samples[0];
    const last = samples[samples.length - 1];
    const endpoint_ids = union_ids([first.ids, last.ids]);
    return {
      coordinates, samples, occupied_ids, endpoint_ids,
      extra_ids: subtract_ids(occupied_ids, endpoint_ids),
      added_cells: subtract_ids(occupied_ids, first.ids).length,
      max_joint_travel_deg: Math.max(...joint_travel_deg),
      max_joint_change_deg: Math.max(...first.angles_deg.map((value, idx) => Math.abs(value - last.angles_deg[idx]))),
      endpoint_change_cells: symmetric_diff_count(first.ids, last.ids),
    };
  }

  function sample_edge(result, graph, edge_idx) {
    const edge = graph.edges[edge_idx];
    if (!edge) throw new Error("接続辺の指定");
    const coordinates = [graph.cells[edge.first_idx].center_angles_deg, edge.port_angles_deg, graph.cells[edge.second_idx].center_angles_deg];
    return { ...sample_path(result.config, coordinates, result.registry.register), edge };
  }

  function shared_bounds(first, second) {
    return bounds_from_matrix(close_constraints(first.constraint_matrix.map((row, idx) =>
      row.map((value, other) => Math.min(value, second.constraint_matrix[idx][other])))), first.coordinate_kind);
  }

  /* 差分制約内での累積角の逐次固定。最短距離の厳密解ではない実行可能な境界候補 */
  function preferred_pose(bounds, angles_deg, is_reverse = false) {
    const preferred = cumulative_angles(angles_deg);
    const order = preferred.map((_, idx) => idx);
    if (is_reverse) order.reverse();
    let current = bounds;
    for (const axis of order) {
      const span = current.max_orientations_deg[axis] - current.min_orientations_deg[axis];
      if (span < 1e-8) continue;
      /* 境界の丸め誤差による負閉路の回避用内側余白 [deg] */
      const margin = Math.min(1e-7, span / 4);
      const value = Math.max(current.min_orientations_deg[axis] + margin, Math.min(current.max_orientations_deg[axis] - margin, preferred[axis]));
      const next = constrain_orientation(current, axis, value, value);
      if (!next) return current.center_angles_deg;
      current = next;
    }
    return current.center_angles_deg;
  }

  function transition_coordinates(config, first, second, edge, start, path_method, local_model) {
    if (path_method === "representative") return [[...start], edge.port_angles_deg, second.center_angles_deg];
    if (path_method !== "boundary") throw new Error("移動方式は境界経由・代表姿勢間");
    const intersection = shared_bounds(first, second);
    if (!intersection) throw new Error("共有境界の欠落");
    const model = local_model;
    const ports = [preferred_pose(intersection, start), preferred_pose(intersection, start, true), edge.port_angles_deg]
      .filter((pose) => contains_pose(first, pose) && contains_pose(second, pose));
    if (!ports.length) throw new Error("実行可能な共有境界姿勢の欠落");
    ports.sort((a, b) => {
      const first_area = estimate_local_sweep(model, a.map((value, idx) => value - start[idx]));
      const second_area = estimate_local_sweep(model, b.map((value, idx) => value - start[idx]));
      return first_area - second_area || motion_bound(config, start, a) - motion_bound(config, start, b);
    });
    const port = ports[0];
    const bound = motion_bound(config, port, second.center_angles_deg);
    const fraction = bound === 0 ? 1 : Math.min(1, config.resolution * 0.5 / bound);
    const target = interpolate_angles(port, second.center_angles_deg, fraction);
    return [[...start], port, target];
  }

  function mark_pareto(candidates) {
    for (const candidate of candidates) {
      candidate.is_pareto = !candidates.some((other) => other !== candidate &&
        other.added_cells <= candidate.added_cells && other.max_joint_travel_deg <= candidate.max_joint_travel_deg + 1e-9 &&
        (other.added_cells < candidate.added_cells || other.max_joint_travel_deg < candidate.max_joint_travel_deg - 1e-9));
    }
    return candidates;
  }

  /* 一グラフにつき最後の比較だけの保持。スライダー操作・再生履歴の無制限蓄積なし */
  const neighbor_cache = new WeakMap();
  function evaluate_neighbors(result, graph, node_idx, start_angles_deg, path_method = "boundary") {
    const first = graph.cells[node_idx];
    if (!first || !contains_pose(first, start_angles_deg)) throw new Error("現在姿勢が選択領域外");
    const key = node_idx + "/" + start_angles_deg.join(",") + "/" + path_method;
    if (neighbor_cache.get(graph)?.key === key) return neighbor_cache.get(graph).value;
    const start_ms = performance.now();
    const registry = create_registry(result.config);
    const local_model = path_method === "boundary" ? build_local_model(result.config, start_angles_deg) : null;
    const candidates = graph.neighbors[node_idx].map((edge_idx) => {
      const edge = graph.edges[edge_idx];
      const target_idx = edge.first_idx === node_idx ? edge.second_idx : edge.first_idx;
      const coordinates = transition_coordinates(result.config, first, graph.cells[target_idx], edge, start_angles_deg, path_method, local_model);
      const sweep = { ...sample_path(result.config, coordinates, registry.register), edge, path_method };
      return { edge_idx, target_idx, sweep, added_cells: sweep.added_cells, max_joint_travel_deg: sweep.max_joint_travel_deg };
    });
    mark_pareto(candidates);
    const value = { candidates, elapsed_ms: performance.now() - start_ms, num_pose_rasterizations: registry.num_pose_rasterizations };
    neighbor_cache.set(graph, { key, value });
    return value;
  }

  /* 非劣候補内での正規化二目的による選択。目標到達・衝突安全性の保証なし */
  function select_neighbor(candidates, priority = "balanced") {
    if (!["balanced", "sweep", "motion"].includes(priority)) throw new Error("候補優先度の指定");
    const front = candidates.filter((candidate) => candidate.is_pareto);
    if (!front.length) return null;
    const min_cells = Math.min(...front.map((candidate) => candidate.added_cells));
    const max_cells = Math.max(...front.map((candidate) => candidate.added_cells));
    const min_travel = Math.min(...front.map((candidate) => candidate.max_joint_travel_deg));
    const max_travel = Math.max(...front.map((candidate) => candidate.max_joint_travel_deg));
    function score(candidate) {
      if (priority === "sweep") return candidate.added_cells;
      if (priority === "motion") return candidate.max_joint_travel_deg;
      const area = max_cells === min_cells ? 0 : (candidate.added_cells - min_cells) / (max_cells - min_cells);
      const travel = max_travel === min_travel ? 0 : (candidate.max_joint_travel_deg - min_travel) / (max_travel - min_travel);
      return area * area + travel * travel;
    }
    return [...front].sort((a, b) => score(a) - score(b) ||
      a.max_joint_travel_deg - b.max_joint_travel_deg || a.added_cells - b.added_cells || a.target_idx - b.target_idx)[0];
  }

  function compare_occupancy(first, second) {
    return { common: intersect_ids(first, second), first_only: subtract_ids(first, second), second_only: subtract_ids(second, first) };
  }

  function graph_stats(graph) {
    const changes = graph.cells.map((cell) => cell.change_cells).sort((first, second) => first - second);
    return {
      num_nodes: graph.cells.length, num_edges: graph.edges.length,
      mean_change_cells: changes.reduce((total, value) => total + value, 0) / changes.length,
      max_change_cells: changes[changes.length - 1],
    };
  }

  /* 選択ノードから指定辺数以内の部分グラフ */
  function graph_neighborhood(graph, node_idx, max_num_hops = 2) {
    if (!graph.cells[node_idx]) throw new Error("ノードの指定");
    const hops = new Array(graph.cells.length).fill(-1);
    const node_ids = [node_idx];
    hops[node_idx] = 0;
    for (let idx = 0; idx < node_ids.length; idx += 1) {
      const current_idx = node_ids[idx];
      if (hops[current_idx] >= max_num_hops) continue;
      for (const edge_idx of graph.neighbors[current_idx]) {
        const edge = graph.edges[edge_idx];
        const other_idx = edge.first_idx === current_idx ? edge.second_idx : edge.first_idx;
        if (hops[other_idx] !== -1) continue;
        hops[other_idx] = hops[current_idx] + 1;
        node_ids.push(other_idx);
      }
    }
    const edge_ids = [];
    for (let idx = 0; idx < graph.edges.length; idx += 1) {
      const edge = graph.edges[idx];
      if (hops[edge.first_idx] !== -1 && hops[edge.second_idx] !== -1) edge_ids.push(idx);
    }
    return { node_ids, edge_ids, hops };
  }

  function export_graph(result, graph_name) {
    const graph = result[graph_name];
    if (!graph) throw new Error("グラフ種別の指定");
    return {
      format: "planar_pose_regions_v5",
      description: "連動一次制約 A(q−q0) の全関節領域と支持半平面包絡。2軸は表示専用。実機制御・衝突安全保証の対象外",
      support_directions: support_directions.map((direction) => [direction.x, direction.y]),
      config: result.config, graph_name, stats: result.stats,
      nodes: graph.cells.map((cell) => ({
        id: cell.id, region: cell.region,
        min_angles_deg: cell.min_angles_deg, max_angles_deg: cell.max_angles_deg,
        min_orientations_deg: cell.min_orientations_deg, max_orientations_deg: cell.max_orientations_deg,
        representative_angles_deg: cell.center_angles_deg,
        conservative_occupied_ids: cell.occupied_ids, envelope_primitives: cell.envelope_primitives,
        sampled_occupied_ids: cell.sampled_occupied_ids, change_cells: cell.change_cells,
        parent_split: cell.parent_split,
      })),
      edges: graph.edges.map((edge) => ({
        first_idx: edge.first_idx, second_idx: edge.second_idx, port_angles_deg: edge.port_angles_deg,
      })),
    };
  }

  return {
    create_config, forward_kinematics, rasterize_pose, create_registry, evaluate_cell,
    union_ids, intersect_ids, subtract_ids, symmetric_diff_count, angles_at,
    build_region_envelope, contains_pose, project_cell, matches_slice, slice_polygon, projected_pose, interpolate_angles,
    build_comparison, sample_edge, motion_bound, graph_stats, graph_neighborhood, export_graph,
    build_local_model, estimate_local_sweep, evaluate_neighbors, select_neighbor, mark_pareto, compare_occupancy,
  };
})();

if (typeof module === "object" && module.exports) module.exports = planar_pose;
else globalThis.planar_pose = planar_pose;
