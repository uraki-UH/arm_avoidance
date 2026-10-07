import * as three from './vendor/three/build/three.module.js';

const identity_rotation = [1, 0, 0, 0, 1, 0, 0, 0, 1];

export function camera_preset(mode = 'mono') {
  if (!['mono', 'stereo', 'fisheye'].includes(mode)) throw Error('未対応のカメラ方式');
  const is_fisheye = mode === 'fisheye';
  const focal = is_fisheye ? 480 / Math.PI : 640 / (2 * Math.tan(35 * Math.PI / 180));
  const lens = {width: 640, height: 480, fx: focal, fy: focal, cx: 319.5, cy: 239.5,
    distortion_model: is_fisheye ? 'equidistant' : 'plumb_bob',
    d: is_fisheye ? [0, 0, 0, 0] : [0, 0, 0, 0, 0], max_fov_deg: is_fisheye ? 180 : 160};
  return {format: 'topo-camera/1', label: '汎用設定（未実機校正）', mode, rate_hz: 5,
    cube_size: 512, near_m: 0.005, far_m: 20,
    mount: {translation_m: [0, 0, 0], rpy_deg: [0, 0, 0]},
    left: lens, right: structuredClone(lens),
    right_in_left: {translation_m: [0.06, 0, 0], rotation: [...identity_rotation]}};
}

function require_number(value, min_value, max_value, name) {
  if (!Number.isFinite(value) || value < min_value || value > max_value) throw Error(`${name}の範囲: ${min_value}〜${max_value}`);
}

function require_vector(value, length, name) {
  if (!Array.isArray(value) || value.length !== length || !value.every(Number.isFinite)) throw Error(`${name}: 有限数${length}個が必要`);
}

function fisheye_radius(theta, d) {
  const squared = theta * theta;
  return theta * (1 + squared * (d[0] + squared * (d[1] + squared * (d[2] + squared * d[3]))));
}

export function validate_camera(input) {
  const config = structuredClone(input);
  if (config?.format !== 'topo-camera/1') throw Error('校正形式はtopo-camera/1');
  if (!['mono', 'stereo', 'fisheye'].includes(config.mode)) throw Error('未対応のカメラ方式');
  if (typeof config.label !== 'string' || config.label.length > 200) throw Error('設定名は200文字以内');
  require_number(config.rate_hz, 1, 30, '取得Hz');
  if (![256, 512, 1024, 2048].includes(config.cube_size)) throw Error('cube_size: 256 / 512 / 1024 / 2048');
  require_number(config.near_m, 0.001, 1, 'near_m');
  require_number(config.far_m, config.near_m + 0.001, 200, 'far_m');
  require_vector(config.mount?.translation_m, 3, 'mount.translation_m');
  require_vector(config.mount?.rpy_deg, 3, 'mount.rpy_deg');
  config.mount.translation_m.forEach(value => require_number(value, -10, 10, '取付位置 [m]'));
  for (const side of config.mode === 'stereo' ? ['left', 'right'] : ['left']) {
    const lens = config[side];
    if (!lens) throw Error(`${side}の内部パラメータが必要`);
    for (const [name, max_value] of [['width', 1920], ['height', 1080]]) {
      require_number(lens[name], 16, max_value, name);
      if (!Number.isInteger(lens[name])) throw Error(`${name}は整数`);
    }
    for (const name of ['fx', 'fy']) require_number(lens[name], 1, 20000, name);
    require_number(lens.cx, -0.5, lens.width - 0.5, 'cx');
    require_number(lens.cy, -0.5, lens.height - 0.5, 'cy');
    if (!['plumb_bob', 'equidistant'].includes(lens.distortion_model)) throw Error('歪み方式: plumb_bob / equidistant');
    const is_fisheye = lens.distortion_model === 'equidistant';
    require_vector(lens.d, is_fisheye ? 4 : 5, `${side}.d`);
    require_number(lens.max_fov_deg, 1, is_fisheye ? 180 : 179, 'max_fov_deg');
    if (config.mode === 'fisheye' && !is_fisheye) throw Error('魚眼モードはequidistantが必要');
    if (!is_fisheye) continue;
    // 指定画角内の単調性検査による魚眼逆投影の多価解防止
    let previous = 0;
    for (let idx = 1; idx <= 512; idx++) {
      const radius = fisheye_radius(lens.max_fov_deg * Math.PI / 360 * idx / 512, lens.d);
      if (!Number.isFinite(radius) || radius <= previous) throw Error('魚眼歪みが指定画角内で折り返し');
      previous = radius;
    }
  }
  if (config.mode === 'stereo') {
    const extrinsic = config.right_in_left;
    require_vector(extrinsic?.translation_m, 3, 'right_in_left.translation_m');
    require_vector(extrinsic?.rotation, 9, 'right_in_left.rotation');
    const baseline = Math.hypot(...extrinsic.translation_m);
    require_number(baseline, 0.0001, 2, '基線長 [m]');
    const rotation = new three.Matrix3().set(...extrinsic.rotation);
    const product = rotation.clone().transpose().multiply(rotation).elements;
    if (Math.abs(rotation.determinant() - 1) > 1e-5 || product.some((v, idx) => Math.abs(v - identity_rotation[idx]) > 1e-5)) throw Error('右カメラ回転は右手系の直交行列が必要');
  }
  return config;
}

function distort_pinhole(x, y, d) {
  const [k1, k2, p1, p2, k3] = d, squared = x * x + y * y;
  const radial = 1 + squared * (k1 + squared * (k2 + squared * k3));
  return [x * radial + 2 * p1 * x * y + p2 * (squared + 2 * x * x),
    y * radial + p1 * (squared + 2 * y * y) + 2 * p2 * x * y];
}

// 光学座標: X右・Y下・Z前。画素(0,0)は左上画素の中心
export function pixel_ray(lens, u, v) {
  const target_x = (u - lens.cx) / lens.fx, target_y = (v - lens.cy) / lens.fy;
  const max_theta = lens.max_fov_deg * Math.PI / 360;
  if (lens.distortion_model === 'equidistant') {
    const radius = Math.hypot(target_x, target_y);
    if (radius < 1e-12) return [0, 0, 1];
    if (radius > fisheye_radius(max_theta, lens.d)) return null;
    let low = 0, high = max_theta;
    for (let iter = 0; iter < 32; iter++) {
      const middle = (low + high) / 2;
      if (fisheye_radius(middle, lens.d) < radius) low = middle;
      else high = middle;
    }
    const theta = (low + high) / 2, scale = Math.sin(theta) / radius;
    return [target_x * scale, target_y * scale, Math.cos(theta)];
  }
  let x = target_x, y = target_y;
  const [k1, k2, p1, p2, k3] = lens.d;
  for (let iter = 0; iter < 20; iter++) {
    const [projected_x, projected_y] = distort_pinhole(x, y, lens.d);
    const error_x = projected_x - target_x, error_y = projected_y - target_y;
    if (Math.hypot(error_x, error_y) < 1e-9) {
      if (Math.atan(Math.hypot(x, y)) > max_theta) return null;
      const length = Math.hypot(x, y, 1);
      return [x / length, y / length, 1 / length];
    }
    const squared = x * x + y * y;
    const radial = 1 + squared * (k1 + squared * (k2 + squared * k3));
    const slope = k1 + squared * (2 * k2 + 3 * k3 * squared);
    const xx = radial + 2 * x * x * slope + 2 * p1 * y + 6 * p2 * x;
    const xy = 2 * x * y * slope + 2 * p1 * x + 2 * p2 * y;
    const yy = radial + 2 * y * y * slope + 6 * p1 * y + 2 * p2 * x;
    const determinant = xx * yy - xy * xy;
    if (!Number.isFinite(determinant) || determinant < 1e-10) return null;
    x -= (yy * error_x - xy * error_y) / determinant;
    y -= (xx * error_y - xy * error_x) / determinant;
  }
  return null;
}

export function project_ray(lens, ray) {
  const [x, y, z] = ray;
  const theta = Math.atan2(Math.hypot(x, y), z);
  if (theta > lens.max_fov_deg * Math.PI / 360 || Math.hypot(x, y, z) === 0) return null;
  let projected;
  if (lens.distortion_model === 'equidistant') {
    const radius = Math.hypot(x, y), scale = radius ? fisheye_radius(theta, lens.d) / radius : 0;
    projected = [x * scale, y * scale];
  } else {
    if (z <= 0) return null;
    projected = distort_pinhole(x / z, y / z, lens.d);
  }
  return [lens.fx * projected[0] + lens.cx, lens.fy * projected[1] + lens.cy];
}

export function camera_transforms(config, optical_to_world) {
  const mount = new three.Matrix4().compose(new three.Vector3(...config.mount.translation_m),
    new three.Quaternion().setFromEuler(new three.Euler(...config.mount.rpy_deg.map(v => v * Math.PI / 180), 'ZYX')), new three.Vector3(1, 1, 1));
  const left = optical_to_world.clone().multiply(mount);
  if (config.mode !== 'stereo') return {left};
  const r = config.right_in_left.rotation, t = config.right_in_left.translation_m;
  const right_in_left = new three.Matrix4().set(r[0], r[1], r[2], t[0], r[3], r[4], r[5], t[1], r[6], r[7], r[8], t[2], 0, 0, 0, 1);
  return {left, right: left.clone().multiply(right_in_left)};
}

function ray_texture(lens) {
  const data = new Float32Array(lens.width * lens.height * 4);
  let num_valid = 0;
  for (let v = 0; v < lens.height; v++) for (let u = 0; u < lens.width; u++) {
    const ray = pixel_ray(lens, u, v);
    if (!ray) continue;
    data.set([...ray, 1], (v * lens.width + u) * 4);
    num_valid++;
  }
  if (!num_valid) throw Error('逆投影できる画素なし');
  const texture = new three.DataTexture(data, lens.width, lens.height, three.RGBAFormat, three.FloatType);
  texture.needsUpdate = true;
  return texture;
}

export class color_camera {
  constructor(renderer, scene, config = camera_preset()) {
    this.renderer = renderer;
    this.scene = scene;
    this.num_frames = 0;
    this.is_pending = false;
    this.resources = [];
    this.configure(config);
  }

  configure(input) {
    if (this.is_pending) throw Error('画像取得の完了後に設定可能');
    const config = validate_camera(input), resources = [];
    try {
      for (const side of config.mode === 'stereo' ? ['left', 'right'] : ['left']) {
        const lens = config[side];
        const rays = ray_texture(lens);
        resources.push({side, lens, rays});
        const resource = resources.at(-1);
        resource.cube = new three.WebGLCubeRenderTarget(config.cube_size, {colorSpace: three.SRGBColorSpace, generateMipmaps: false});
        resource.camera = new three.CubeCamera(config.near_m, config.far_m, resource.cube);
        resource.camera.matrixAutoUpdate = false;
        resource.output = new three.WebGLRenderTarget(lens.width, lens.height, {colorSpace: three.SRGBColorSpace, depthBuffer: false});
        resource.material = new three.ShaderMaterial({depthTest: false, depthWrite: false, toneMapped: false,
          uniforms: {ray_map: {value: rays}, color_cube: {value: resource.cube.texture}},
          vertexShader: 'varying vec2 image_uv; void main(){image_uv=uv;gl_Position=vec4(position.xy,0.,1.);}',
          fragmentShader: `uniform sampler2D ray_map; uniform samplerCube color_cube; varying vec2 image_uv;
            void main(){vec4 ray=texture2D(ray_map,vec2(image_uv.x,1.-image_uv.y));
              gl_FragColor=ray.a>0.5 ? vec4(textureCube(color_cube,ray.xyz).rgb,1.) : vec4(0.);
              #include <colorspace_fragment>
            }`});
        resource.geometry = new three.PlaneGeometry(2, 2);
        resource.screen = new three.Scene();
        resource.screen.add(new three.Mesh(resource.geometry, resource.material));
      }
    } catch (error) {
      this.dispose_resources(resources);
      throw error;
    }
    this.dispose_resources(this.resources);
    this.resources = resources;
    this.config = config;
  }

  async capture(optical_to_world, exclude = []) {
    if (this.is_pending) throw Error('画像取得中');
    this.is_pending = true;
    const renderer = this.renderer, scene = this.scene;
    const config = structuredClone(this.config), transforms = camera_transforms(config, optical_to_world);
    const frame = {format: 'topo-camera-frame/1', id: ++this.num_frames, timestamp_ms: Date.now(), calibration: config, images: {}};
    const previous = {target: renderer.getRenderTarget(), face: renderer.getActiveCubeFace(), mip: renderer.getActiveMipmapLevel(),
      auto_clear: renderer.autoClear, shadow_update: renderer.shadowMap.autoUpdate, shadow_needed: renderer.shadowMap.needsUpdate,
      xr_enabled: renderer.xr.enabled, override: scene.overrideMaterial};
    const visibility = [...new Set(exclude.filter(Boolean))].map(object => [object, object.visible]);
    const reads = [];
    try {
      try {
        visibility.forEach(([object]) => object.visible = false);
        renderer.autoClear = true;
        renderer.shadowMap.autoUpdate = false;
        renderer.shadowMap.needsUpdate = false;
        scene.overrideMaterial = null;
        // 同一シーン時刻での左右描画と読み出し要求。非同期待ちは全描画後
        for (const resource of this.resources) {
          const {side, lens, camera, output, screen} = resource;
          camera.matrix.copy(transforms[side]);
          camera.updateMatrixWorld(true);
          camera.update(renderer, scene);
          renderer.setRenderTarget(output);
          renderer.render(screen, new three.Camera());
          const raw = new Uint8Array(lens.width * lens.height * 4);
          reads.push(renderer.readRenderTargetPixelsAsync(output, 0, 0, lens.width, lens.height, raw).then(() => {
            const rgba = new Uint8ClampedArray(raw.length), stride = lens.width * 4;
            for (let v = 0; v < lens.height; v++) rgba.set(raw.subarray((lens.height - 1 - v) * stride, (lens.height - v) * stride), v * stride);
            frame.images[side] = {width: lens.width, height: lens.height, rgba, optical_to_world: transforms[side].toArray()};
          }));
          // 次眼のテクスチャ生成前の非同期読み出しバッファ解除
          renderer.getContext().bindBuffer(renderer.getContext().PIXEL_PACK_BUFFER, null);
        }
      } finally {
        renderer.getContext().bindBuffer(renderer.getContext().PIXEL_PACK_BUFFER, null);
        renderer.setRenderTarget(previous.target, previous.face, previous.mip);
        renderer.autoClear = previous.auto_clear;
        renderer.shadowMap.autoUpdate = previous.shadow_update;
        renderer.shadowMap.needsUpdate = previous.shadow_needed;
        renderer.xr.enabled = previous.xr_enabled;
        scene.overrideMaterial = previous.override;
        visibility.forEach(([object, is_visible]) => object.visible = is_visible);
      }
      await Promise.all(reads);
      return frame;
    } finally {
      // 片眼の失敗時にも残りのGPU読み出し完了後の解放
      await Promise.allSettled(reads);
      this.is_pending = false;
    }
  }

  dispose_resources(resources) {
    for (const resource of resources) for (const name of ['rays', 'cube', 'output', 'material', 'geometry']) resource[name]?.dispose();
  }

  dispose() {
    if (this.is_pending) throw Error('画像取得中の解放不可');
    this.dispose_resources(this.resources);
    this.resources = [];
  }
}
