/* ARMTD型の関節到達集合・依存多項式・制動付き占有の独立比較基盤 */
"use strict";

const armtd_reachability = (() => {
  const angle_scale = Math.PI / 180;
  const source_url = "https://arxiv.org/abs/2002.01591";

  function finite_vector(values, size, name) {
    if (!Array.isArray(values) || values.length !== size || !values.every(Number.isFinite)) throw new Error(name + "は有限値" + size + "個");
    return [...values];
  }

  /* 加速度パラメータ付き軌道と終端制動。角度[deg]・速度[deg/s]・加速度[deg/s²] */
  function create_motion(config, options = {}) {
    const size = config.num_joints;
    if (![3, 4, 5].includes(size)) throw new Error("平面比較の対象は3～5軸");
    const start_angles_deg = finite_vector(options.start_angles_deg ?? config.base_angles_deg, size, "開始角度");
    const start_velocities_deg_s = finite_vector(options.start_velocities_deg_s ?? new Array(size).fill(0), size, "開始速度");
    const plan_time_sec = options.plan_time_sec ?? 0.5;
    const brake_time_sec = options.brake_time_sec ?? 0.5;
    const time_step_sec = options.time_step_sec ?? 0.05;
    const max_degree = options.max_degree ?? 3;
    const max_num_terms = options.max_num_terms ?? 48;
    if (![plan_time_sec, brake_time_sec, time_step_sec].every(value => Number.isFinite(value) && value >= 1e-4) ||
        plan_time_sec + brake_time_sec > 10 || Math.ceil(plan_time_sec / time_step_sec) + Math.ceil(brake_time_sec / time_step_sec) > 400) {
      throw new Error("時間設定0.0001秒以上・10秒以内・400区間以内");
    }
    if (!Number.isInteger(max_degree) || max_degree < 2 || max_degree > 6 || !Number.isInteger(max_num_terms) || max_num_terms < 8 || max_num_terms > 192) throw new Error("多項式次数2～6・項数8～192");
    const min_accelerations_deg_s2 = finite_vector(options.min_accelerations_deg_s2 ?? new Array(size).fill(-40), size, "加速度下限");
    const max_accelerations_deg_s2 = finite_vector(options.max_accelerations_deg_s2 ?? new Array(size).fill(40), size, "加速度上限");
    if (min_accelerations_deg_s2.some((value, idx) => value > max_accelerations_deg_s2[idx])) throw new Error("加速度の下限・上限の順序");
    const min_initial_velocities_deg_s = finite_vector(options.min_initial_velocities_deg_s ?? start_velocities_deg_s, size, "事前速度下限");
    const max_initial_velocities_deg_s = finite_vector(options.max_initial_velocities_deg_s ?? start_velocities_deg_s, size, "事前速度上限");
    if (start_velocities_deg_s.some((value, idx) => value < min_initial_velocities_deg_s[idx] || value > max_initial_velocities_deg_s[idx])) throw new Error("開始速度は事前計算の速度区間内");
    const max_speeds_deg_s = finite_vector(options.max_speeds_deg_s ?? new Array(size).fill(180), size, "速度上限");
    const max_accelerations_limit_deg_s2 = finite_vector(options.max_accelerations_limit_deg_s2 ?? new Array(size).fill(360), size, "駆動加速度上限");
    if ([...max_speeds_deg_s, ...max_accelerations_limit_deg_s2].some(value => value <= 0)) throw new Error("速度・駆動加速度上限は正の値");
    if ([...start_angles_deg, ...start_velocities_deg_s, ...min_initial_velocities_deg_s, ...max_initial_velocities_deg_s,
      ...min_accelerations_deg_s2, ...max_accelerations_deg_s2, ...max_speeds_deg_s, ...max_accelerations_limit_deg_s2].some(value => Math.abs(value) > 1e6)) throw new Error("数値演算範囲は各成分100万以内");
    return { num_joints: size, start_angles_deg, start_velocities_deg_s, min_initial_velocities_deg_s, max_initial_velocities_deg_s,
      min_accelerations_deg_s2, max_accelerations_deg_s2, max_speeds_deg_s, max_accelerations_limit_deg_s2,
      plan_time_sec, brake_time_sec, time_step_sec, max_degree, max_num_terms,
      duration_sec: plan_time_sec + brake_time_sec };
  }

  function acceleration_values(motion, parameters) {
    finite_vector(parameters, motion.num_joints, "動作パラメータ");
    if (parameters.some(value => Math.abs(value) > 1)) throw new Error("動作パラメータの範囲は−1～1");
    return parameters.map((value, idx) => (motion.min_accelerations_deg_s2[idx] + motion.max_accelerations_deg_s2[idx]) / 2 +
      value * (motion.max_accelerations_deg_s2[idx] - motion.min_accelerations_deg_s2[idx]) / 2);
  }

  function trajectory_at(motion, time_sec, parameters = new Array(motion.num_joints).fill(0)) {
    if (!Number.isFinite(time_sec) || time_sec < 0 || time_sec > motion.duration_sec + 1e-10) throw new Error("軌道時刻は時間範囲内");
    const time = Math.min(time_sec, motion.duration_sec);
    const accelerations = acceleration_values(motion, parameters);
    const angles_deg = [], velocities_deg_s = [];
    for (let idx = 0; idx < motion.num_joints; idx += 1) {
      const start = motion.start_angles_deg[idx], velocity = motion.start_velocities_deg_s[idx], acceleration = accelerations[idx];
      const plan = motion.plan_time_sec, brake = motion.brake_time_sec;
      if (time <= plan) {
        angles_deg.push(start + velocity * time + acceleration * time * time / 2);
        velocities_deg_s.push(velocity + acceleration * time);
      } else {
        const elapsed = time - plan;
        const last_velocity = velocity + acceleration * plan;
        angles_deg.push(start + velocity * plan + acceleration * plan * plan / 2 + last_velocity * (elapsed - elapsed * elapsed / (2 * brake)));
        velocities_deg_s.push(last_velocity * (1 - elapsed / brake));
      }
    }
    return { angles_deg, velocities_deg_s, time_sec: time };
  }

  /* 位相境界と速度ゼロ時刻による全時間の位置極値。角度下限・上限の解析的評価 */
  function joint_bounds(motion, min_time_sec, max_time_sec, parameters = null) {
    if (![min_time_sec, max_time_sec].every(Number.isFinite) || min_time_sec < 0 || max_time_sec > motion.duration_sec + 1e-10 || min_time_sec > max_time_sec) throw new Error("関節区間の時刻範囲");
    const accelerations = parameters ? acceleration_values(motion, parameters).map(value => [value]) :
      motion.min_accelerations_deg_s2.map((value, idx) => [value, motion.max_accelerations_deg_s2[idx]]);
    const min_angles_deg = [], max_angles_deg = [];
    for (let idx = 0; idx < motion.num_joints; idx += 1) {
      const values = [];
      for (const acceleration of accelerations[idx]) {
        const times = [min_time_sec, max_time_sec];
        if (motion.plan_time_sec > min_time_sec && motion.plan_time_sec < max_time_sec) times.push(motion.plan_time_sec);
        const stationary = -motion.start_velocities_deg_s[idx] / acceleration;
        if (Number.isFinite(stationary) && stationary > min_time_sec && stationary < Math.min(max_time_sec, motion.plan_time_sec)) times.push(stationary);
        for (const time of times) {
          const plan = motion.plan_time_sec, velocity = motion.start_velocities_deg_s[idx];
          const elapsed = time - plan;
          values.push(motion.start_angles_deg[idx] + (time <= plan ? velocity * time + acceleration * time * time / 2 :
            velocity * plan + acceleration * plan * plan / 2 + (velocity + acceleration * plan) * (elapsed - elapsed * elapsed / (2 * motion.brake_time_sec))));
        }
      }
      min_angles_deg.push(Math.min(...values)); max_angles_deg.push(Math.max(...values));
    }
    return { min_angles_deg, max_angles_deg };
  }

  function motion_limits(config, motion, parameters = null) {
    const bounds = joint_bounds(motion, 0, motion.duration_sec, parameters);
    const accelerations = parameters ? acceleration_values(motion, parameters).map(value => [value]) :
      motion.min_accelerations_deg_s2.map((value, idx) => [value, motion.max_accelerations_deg_s2[idx]]);
    const violations = [];
    for (let idx = 0; idx < motion.num_joints; idx += 1) {
      if (bounds.min_angles_deg[idx] < config.min_angles_deg[idx] - 1e-8 || bounds.max_angles_deg[idx] > config.max_angles_deg[idx] + 1e-8) violations.push({ joint_idx: idx, kind: "position" });
      for (const acceleration of accelerations[idx]) {
        const last_velocity = motion.start_velocities_deg_s[idx] + acceleration * motion.plan_time_sec;
        if (Math.max(Math.abs(motion.start_velocities_deg_s[idx]), Math.abs(last_velocity)) > motion.max_speeds_deg_s[idx] + 1e-8) violations.push({ joint_idx: idx, kind: "speed" });
        if (Math.abs(acceleration) > motion.max_accelerations_limit_deg_s2[idx] + 1e-8) violations.push({ joint_idx: idx, kind: "acceleration" });
        if (Math.abs(last_velocity) / motion.brake_time_sec > motion.max_accelerations_limit_deg_s2[idx] + 1e-8) violations.push({ joint_idx: idx, kind: "braking" });
      }
    }
    return { is_feasible: violations.length === 0, violations, ...bounds };
  }

  /* 16進桁による共有依存係数の指数ID。3～5軸・積の指数12までの整数演算 */
  function polynomial_context(num_variables, max_degree, max_num_terms) {
    const factors = Array.from({ length: num_variables }, (_, idx) => 16 ** idx);
    const metadata = new Map([[0, { degree: 0, has_odd_power: false }]]);
    function info(key) {
      if (!metadata.has(key)) {
        const powers = factors.map(factor => Math.floor(key / factor) % 16);
        metadata.set(key, { degree: powers.reduce((sum, value) => sum + value, 0), has_odd_power: powers.some(value => value % 2 === 1) });
      }
      return metadata.get(key);
    }
    return { num_variables, max_degree, max_num_terms, factors, info, num_reduced_terms: 0, num_interval_fallbacks: 0 };
  }

  function constant(value) { return { terms: new Map(value === 0 ? [] : [[0, value]]), error: 0 }; }
  function coefficient_bound(poly) { let total = 0; for (const value of poly.terms.values()) total += Math.abs(value); return total; }
  function scale(poly, value) { return { terms: new Map([...poly.terms].map(([key, coefficient]) => [key, coefficient * value])), error: poly.error * Math.abs(value) }; }
  function add(first, second) {
    const terms = new Map(first.terms);
    for (const [key, value] of second.terms) {
      const total = (terms.get(key) ?? 0) + value;
      if (total === 0) terms.delete(key); else terms.set(key, total);
    }
    return { terms, error: first.error + second.error + 1e-13 * (coefficient_bound(first) + coefficient_bound(second) + 1) };
  }

  /* 次数・項数の削減分を独立残差へ移行。slice後も残差の保持 */
  function reduce(poly, context) {
    let error = poly.error;
    const kept = [];
    for (const item of poly.terms) {
      if (context.info(item[0]).degree > context.max_degree) { error += Math.abs(item[1]); context.num_reduced_terms += 1; }
      else kept.push(item);
    }
    kept.sort((first, second) => Number(second[0] === 0) - Number(first[0] === 0) || Math.abs(second[1]) - Math.abs(first[1]) || first[0] - second[0]);
    for (const [, value] of kept.slice(context.max_num_terms)) { error += Math.abs(value); context.num_reduced_terms += 1; }
    return { terms: new Map(kept.slice(0, context.max_num_terms)), error };
  }

  function multiply(first, second, context) {
    const terms = new Map();
    for (const [first_key, first_value] of first.terms) for (const [second_key, second_value] of second.terms) {
      const key = first_key + second_key;
      terms.set(key, (terms.get(key) ?? 0) + first_value * second_value);
    }
    const first_bound = coefficient_bound(first), second_bound = coefficient_bound(second);
    return reduce({ terms, error: first_bound * second.error + second_bound * first.error + first.error * second.error +
      1e-12 * (first_bound * second_bound + first.error * second.error + 1) }, context);
  }

  function slice(poly, values, context) {
    const terms = new Map();
    for (const [key, coefficient] of poly.terms) {
      let next_key = key, value = coefficient;
      for (const [variable, fixed] of values) {
        const power = Math.floor(key / context.factors[variable]) % 16;
        next_key -= power * context.factors[variable]; value *= fixed ** power;
      }
      terms.set(next_key, (terms.get(next_key) ?? 0) + value);
    }
    return { terms, error: poly.error + 1e-12 * (coefficient_bound(poly) + 1) };
  }

  /* 3次Taylorと4次残差の解析的上界。大角度変動時はsin・cosの全区間 */
  function rotation_models(angle, context) {
    const center = angle.terms.get(0) ?? 0;
    const delta = add(angle, constant(-center));
    const extent = coefficient_bound(delta) + delta.error;
    if (extent > Math.PI / 2) {
      context.num_interval_fallbacks += 1;
      return { cos: { terms: new Map(), error: 1 }, sin: { terms: new Map(), error: 1 } };
    }
    const squared = multiply(delta, delta, context), cubed = multiply(squared, delta, context);
    const even = add(constant(1), scale(squared, -0.5));
    const odd = add(delta, scale(cubed, -1 / 6));
    const remainder = extent ** 4 / 24;
    const cos = add(scale(even, Math.cos(center)), scale(odd, -Math.sin(center)));
    const sin = add(scale(even, Math.sin(center)), scale(odd, Math.cos(center)));
    cos.error += remainder; sin.error += remainder;
    return { cos: reduce(cos, context), sin: reduce(sin, context) };
  }

  function affine_variable(context, variable, min_value, max_value) {
    const result = constant((min_value + max_value) / 2);
    if (max_value !== min_value) result.terms.set(context.factors[variable], (max_value - min_value) / 2);
    return result;
  }

  function time_intervals(motion) {
    const intervals = [];
    for (const [min_time_sec, max_time_sec, phase] of [[0, motion.plan_time_sec, "plan"], [motion.plan_time_sec, motion.duration_sec, "brake"]]) {
      const num_steps = Math.max(1, Math.ceil((max_time_sec - min_time_sec) / motion.time_step_sec - 1e-10));
      for (let idx = 0; idx < num_steps; idx += 1) intervals.push({ min_time_sec: min_time_sec + idx * (max_time_sec - min_time_sec) / num_steps,
        max_time_sec: min_time_sec + (idx + 1) * (max_time_sec - min_time_sec) / num_steps, phase });
    }
    return intervals;
  }

  /* 初速度区間・加速度区間の関節到達集合。全関節・全回転で共通の時間変数 */
  function prepare_joint_reachability(motion) {
    const start_ms = performance.now();
    const context = polynomial_context(2 * motion.num_joints + 1, motion.max_degree, motion.max_num_terms);
    const frames = time_intervals(motion).map(frame => {
      const time = affine_variable(context, 0, frame.min_time_sec, frame.max_time_sec);
      let velocity_coefficient = time, acceleration_coefficient = scale(multiply(time, time, context), 0.5);
      if (frame.phase === "brake") {
        const elapsed = add(time, constant(-motion.plan_time_sec));
        const travel = add(elapsed, scale(multiply(elapsed, elapsed, context), -0.5 / motion.brake_time_sec));
        velocity_coefficient = add(constant(motion.plan_time_sec), travel);
        acceleration_coefficient = add(constant(motion.plan_time_sec ** 2 / 2), scale(travel, motion.plan_time_sec));
      }
      const joints = [];
      for (let idx = 0; idx < motion.num_joints; idx += 1) {
        const velocity = affine_variable(context, 1 + 2 * idx, motion.min_initial_velocities_deg_s[idx], motion.max_initial_velocities_deg_s[idx]);
        const acceleration = affine_variable(context, 2 + 2 * idx, motion.min_accelerations_deg_s2[idx], motion.max_accelerations_deg_s2[idx]);
        const angle = reduce(scale(add(multiply(velocity, velocity_coefficient, context), multiply(acceleration, acceleration_coefficient, context)), angle_scale), context);
        joints.push({ angle, ...rotation_models(angle, context) });
      }
      return { ...frame, joints };
    });
    return { motion, context, frames, preparation_ms: performance.now() - start_ms };
  }

  function velocity_slice(motion) {
    return motion.start_velocities_deg_s.map((value, idx) => {
      const min_value = motion.min_initial_velocities_deg_s[idx], max_value = motion.max_initial_velocities_deg_s[idx];
      return [1 + 2 * idx, max_value === min_value ? 0 : (2 * value - min_value - max_value) / (max_value - min_value)];
    });
  }

  /* 初速度でのsliceと現在角への回転。原著のCORA到達計算ではなく解析的区間多項式 */
  function compose_reachability(config, prepared, method = "armtd_polynomial") {
    if (!["armtd_polynomial", "planar_polynomial"].includes(method)) throw new Error("回転合成・平面累積角の方式指定");
    const start_ms = performance.now(), { motion, context } = prepared;
    const fixed = velocity_slice(motion);
    const frames = prepared.frames.map(frame => {
      let cos = constant(1), sin = constant(0), angle = constant(0);
      let x = constant(0), y = constant(0);
      const points = [{ x, y }];
      for (let idx = 0; idx < motion.num_joints; idx += 1) {
        const joint = frame.joints[idx], start = motion.start_angles_deg[idx] * angle_scale;
        if (method === "planar_polynomial") {
          angle = reduce(add(angle, add(slice(joint.angle, fixed, context), constant(start))), context);
          ({ cos, sin } = rotation_models(angle, context));
        } else {
          const joint_cos = slice(joint.cos, fixed, context), joint_sin = slice(joint.sin, fixed, context);
          const rotated_cos = add(scale(joint_cos, Math.cos(start)), scale(joint_sin, -Math.sin(start)));
          const rotated_sin = add(scale(joint_sin, Math.cos(start)), scale(joint_cos, Math.sin(start)));
          const next_cos = reduce(add(multiply(cos, rotated_cos, context), scale(multiply(sin, rotated_sin, context), -1)), context);
          sin = reduce(add(multiply(sin, rotated_cos, context), multiply(cos, rotated_sin, context)), context);
          cos = next_cos;
        }
        x = reduce(add(x, scale(cos, config.lengths[idx])), context); y = reduce(add(y, scale(sin, config.lengths[idx])), context);
        points.push({ x, y });
      }
      return { min_time_sec: frame.min_time_sec, max_time_sec: frame.max_time_sec, phase: frame.phase, points, tool_rotation: { cos, sin } };
    });
    return { method, motion, context, frames, composition_ms: performance.now() - start_ms,
      model_scope: "planar_ideal_kinematics", source_url, implementation: "analytic_joint_reachability_polynomial_rotation",
      limitations: ["原著CORA・GPU・IPOPTの再現なし", "丸め誤差は余裕項、検証付き浮動小数点演算の保証なし", "実機追従・トルク・人の動きのモデルなし"] };
  }

  function slice_reachability(reach, parameters) {
    acceleration_values(reach.motion, parameters);
    const values = parameters.map((value, idx) => [2 + 2 * idx, value]);
    const frames = reach.frames.map(frame => ({ ...frame, points: frame.points.map(point => ({
      x: slice(point.x, values, reach.context), y: slice(point.y, values, reach.context) })),
      tool_rotation: { cos: slice(frame.tool_rotation.cos, values, reach.context), sin: slice(frame.tool_rotation.sin, values, reach.context) } }));
    return { ...reach, frames, parameters: [...parameters] };
  }

  const directions = Array.from({ length: 64 }, (_, idx) => ({ x: Math.cos(idx * Math.PI / 32), y: Math.sin(idx * Math.PI / 32) }));
  /* 端点ごとの係数整列と64方向への投影。方向ごとのMap生成・多項式合成なし */
  function point_supports(point, context) {
    const keys = new Set([...point.x.terms.keys(), ...point.y.terms.keys()]);
    const coefficients = [...keys].map(key => ({ key, x: point.x.terms.get(key) ?? 0,
      y: point.y.terms.get(key) ?? 0, has_odd_power: context.info(key).has_odd_power }));
    return directions.map(direction => {
      let value = Math.abs(direction.x) * point.x.error + Math.abs(direction.y) * point.y.error + 1e-9;
      for (const coefficient of coefficients) {
        const projected = coefficient.x * direction.x + coefficient.y * direction.y;
        value += coefficient.key === 0 ? projected : coefficient.has_odd_power ? Math.abs(projected) : Math.max(0, projected);
      }
      return value;
    });
  }

  function support_primitives(config, reach) {
    return reach.frames.map(frame => {
      let straight_reach = 0;
      const supports = frame.points.map(point => point_supports(point, reach.context));
      return frame.points.slice(1).map((_, idx) => {
        straight_reach += config.lengths[idx];
        return { supports: directions.map((_, direction_idx) => Math.max(supports[idx][direction_idx],
          supports[idx + 1][direction_idx]) + config.radii[idx]), max_reach: straight_reach + config.radii[idx],
          min_time_sec: frame.min_time_sec, max_time_sec: frame.max_time_sec, link_idx: idx };
      });
    });
  }

  /* 同一格子への1回の合併登録。セル中心基準、連続形状の衝突保証とは別用途 */
  function rasterize_primitives(config, frames) {
    const occupied = new Uint8Array(config.grid_width ** 2);
    const { resolution, world_half_size, grid_width } = config;
    for (const primitives of frames) for (const primitive of primitives) {
      const { supports, max_reach } = primitive;
      const min_y = Math.max(0, Math.ceil((-supports[48] + world_half_size) / resolution - 0.5));
      const max_y = Math.min(grid_width - 1, Math.floor((supports[16] + world_half_size) / resolution - 0.5));
      for (let y_idx = min_y; y_idx <= max_y; y_idx += 1) {
        const y = (y_idx + 0.5) * resolution - world_half_size;
        if (Math.abs(y) > max_reach + 1e-10) continue;
        const extent = Math.sqrt(Math.max(0, max_reach ** 2 - y ** 2));
        let min_x = -extent, max_x = extent;
        directions.forEach((direction, idx) => {
          if (direction.x > 1e-10) max_x = Math.min(max_x, (supports[idx] - direction.y * y) / direction.x);
          else if (direction.x < -1e-10) min_x = Math.max(min_x, (supports[idx] - direction.y * y) / direction.x);
        });
        const first = Math.max(0, Math.ceil((min_x + world_half_size) / resolution - 0.5 - 1e-9));
        const last = Math.min(grid_width - 1, Math.floor((max_x + world_half_size) / resolution - 0.5 + 1e-9));
        if (first <= last) occupied.fill(1, y_idx * grid_width + first, y_idx * grid_width + last + 1);
      }
    }
    const occupied_ids = [];
    for (let idx = 0; idx < occupied.length; idx += 1) if (occupied[idx]) occupied_ids.push(idx);
    return occupied_ids;
  }

  /* 支持上界による障害物との分離確認。分離未確認は衝突確定ではない状態 */
  function obstacle_separation(frames, obstacles = []) {
    const unresolved = [];
    for (const obstacle of obstacles) if (!Array.isArray(obstacle) || obstacle.length < 3 || !obstacle.every(point => Array.isArray(point) && point.length === 2 && point.every(Number.isFinite))) throw new Error("障害物は有限値の頂点列");
    frames.forEach((primitives, frame_idx) => primitives.forEach(primitive => obstacles.forEach((obstacle, obstacle_idx) => {
      const can_separate = directions.some((direction, idx) => primitive.supports[idx] + 1e-9 <
        Math.min(...obstacle.map(point => point[0] * direction.x + point[1] * direction.y)));
      if (!can_separate) unresolved.push({ frame_idx, link_idx: primitive.link_idx, obstacle_idx });
    })));
    return { can_separate: unresolved.length === 0, has_obstacles: obstacles.length > 0, unresolved };
  }

  function create_backend(engine) {
    let prepared_key = "", cached_prepared = null;
    function evaluate(config, motion, method, parameters = null, obstacles = []) {
      if (!["joint_interval", "armtd_polynomial", "planar_polynomial"].includes(method)) throw new Error("到達集合の方式指定");
      if (config.num_joints !== motion.num_joints) throw new Error("到達集合とロボットの軸数一致");
      const start_ms = performance.now();
      let frames, reach = null, preparation_ms = 0, composition_ms = 0, is_preparation_reused = false;
      if (method === "joint_interval") {
        frames = time_intervals(motion).map(frame => engine.build_region_envelope(config,
          joint_bounds(motion, frame.min_time_sec, frame.max_time_sec, parameters), { enable_rasterization: false }).primitives.map((primitive, idx) => ({
          ...primitive, link_idx: idx, min_time_sec: frame.min_time_sec, max_time_sec: frame.max_time_sec })));
      } else {
        const key = JSON.stringify({ ...motion, start_angles_deg: null, start_velocities_deg_s: null });
        is_preparation_reused = key === prepared_key;
        if (!is_preparation_reused) { cached_prepared = prepare_joint_reachability(motion); prepared_key = key; preparation_ms = cached_prepared.preparation_ms; }
        reach = compose_reachability(config, { ...cached_prepared, motion }, method);
        composition_ms = reach.composition_ms;
        if (parameters) reach = slice_reachability(reach, parameters);
        frames = support_primitives(config, reach);
      }
      const occupied_ids = rasterize_primitives(config, frames);
      return { method, occupied_ids, primitives: frames, reach, preparation_ms, composition_ms,
        elapsed_ms: performance.now() - start_ms, is_preparation_reused, num_time_intervals: frames.length,
        num_voxel_registrations: 1, limits: motion_limits(config, motion, parameters), separation: obstacle_separation(frames, obstacles) };
    }

    function compare(config, options = {}, parameters = null, obstacles = []) {
      const motion = create_motion(config, options);
      const methods = ["joint_interval", "armtd_polynomial", "planar_polynomial"].map(method => evaluate(config, motion, method, parameters, obstacles));
      const start_pose = engine.tool_pose(config, motion.start_angles_deg);
      const central_terminal_pose = engine.tool_pose(config, trajectory_at(motion, motion.duration_sec,
        parameters ?? new Array(config.num_joints).fill(0)).angles_deg);
      return { motion, methods, parameters, source_url, start_pose, central_terminal_pose,
        task_difference: engine.tool_pose_difference(start_pose, central_terminal_pose),
        comparison_scope: parameters ? "single_trajectory" : "same_trajectory_family" };
    }

    /* グラフへの組み込み口。終端姿勢からの加速度設定、制動・全時間制約の別確認 */
    function evaluate_graph_edge(config, first_cell, second_cell, start_angles_deg, options = {}) {
      const target_angles_deg = finite_vector(options.target_angles_deg ?? second_cell.center_angles_deg, config.num_joints, "目標角度");
      const velocities = finite_vector(options.start_velocities_deg_s ?? new Array(config.num_joints).fill(0), config.num_joints, "開始速度");
      finite_vector(start_angles_deg, config.num_joints, "開始角度");
      const plan = options.plan_time_sec ?? 0.5, brake = options.brake_time_sec ?? 0.5, duration = plan + brake;
      const accelerations = target_angles_deg.map((value, idx) =>
        (value - start_angles_deg[idx] - velocities[idx] * (duration + plan) / 2) * 2 / (plan * duration));
      const motion = create_motion(config, { ...options, start_angles_deg, start_velocities_deg_s: velocities,
        min_accelerations_deg_s2: accelerations, max_accelerations_deg_s2: accelerations });
      const result = evaluate(config, motion, options.method ?? "armtd_polynomial", null, options.obstacles ?? []);
      const times = [0, motion.plan_time_sec, motion.duration_sec];
      for (const [min_time, max_time] of [[0, motion.plan_time_sec], [motion.plan_time_sec, motion.duration_sec]]) {
        /* 二次軌道と各一次制約の境界交点。中点だけの固定刻み検査なし */
        const sample = [min_time, (min_time + max_time) / 2, max_time].map(time => trajectory_at(motion, time).angles_deg);
        const span = max_time - min_time;
        for (const cell of [first_cell, second_cell]) for (const constraint of cell.region.constraints) for (const bound of [constraint.min_dev_deg, constraint.max_dev_deg]) {
          const values = sample.map(q => constraint.coefficients.reduce((sum, value, idx) => sum + value * (q[idx] - cell.center_angles_deg[idx]), 0) - bound);
          const a = 2 * (values[2] - 2 * values[1] + values[0]), b = values[2] - values[0] - a, c = values[0];
          const roots = Math.abs(a) < 1e-10 ? (Math.abs(b) < 1e-10 ? [] : [-c / b]) :
            b * b - 4 * a * c >= 0 ? [(-b - Math.sqrt(Math.max(0, b * b - 4 * a * c))) / (2 * a), (-b + Math.sqrt(Math.max(0, b * b - 4 * a * c))) / (2 * a)] : [];
          for (const root of roots) if (root > 0 && root < 1) times.push(min_time + root * span);
        }
      }
      times.sort((a, b) => a - b);
      const check_times = [...times, ...times.slice(1).map((time, idx) => (time + times[idx]) / 2)];
      const can_stay_in_regions = check_times.every(time => {
        const q = trajectory_at(motion, time).angles_deg;
        return engine.contains_pose(first_cell, q) || engine.contains_pose(second_cell, q);
      });
      const start_pose = engine.tool_pose(config, start_angles_deg), terminal_pose = engine.tool_pose(config, target_angles_deg);
      return { ...result, motion, target_angles_deg: [...target_angles_deg], start_pose, terminal_pose,
        task_difference: engine.tool_pose_difference(start_pose, terminal_pose), can_stay_in_regions,
        can_connect: engine.contains_pose(first_cell, start_angles_deg) && engine.contains_pose(second_cell, target_angles_deg) &&
          can_stay_in_regions && result.limits.is_feasible && result.separation.can_separate,
        connection_scope: "ideal_kinematics_numeric_bounds", is_real_robot_certified: false };
    }
    return { evaluate, compare, evaluate_graph_edge };
  }

  function export_motion(comparison) {
    return { format: "planar_armtd_comparison_v1", source_url, motion: comparison.motion,
      comparison_scope: comparison.comparison_scope, parameters: comparison.parameters,
      start_pose: comparison.start_pose, central_terminal_pose: comparison.central_terminal_pose, task_difference: comparison.task_difference,
      model_scope: "planar_ideal_kinematics", is_real_robot_certified: false,
      methods: comparison.methods.map(({ reach, ...result }) => result) };
  }

  return { create_motion, trajectory_at, joint_bounds, motion_limits, prepare_joint_reachability,
    compose_reachability, slice_reachability, support_primitives, rasterize_primitives, obstacle_separation, create_backend, export_motion };
})();

if (typeof module === "object" && module.exports) module.exports = armtd_reachability;
else globalThis.armtd_reachability = armtd_reachability;
