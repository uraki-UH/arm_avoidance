import {camera_preset, color_camera} from './camera-core.js';
import {zipFiles as zip_files} from './capture-zip.js';

const element = id => document.getElementById(id);
const mode_labels = {mono: '単眼', stereo: 'ステレオ', fisheye: '魚眼'};
const size_options = ['320,240', '640,480', '1280,720', '1920,1080'];
const lens_fields = ['width', 'height', 'fx', 'fy', 'cx', 'cy', 'max_fov_deg'];

function numeric_field(id, label, min_value, max_value, step = 'any') {
  return `<label>${label}<input id="${id}" type="number" min="${min_value}" max="${max_value}" step="${step}"></label>`;
}

function lens_editor(side, title) {
  return `<details id="camera-${side}-settings"><summary>${title}</summary><div class="field-grid">
    ${numeric_field(`camera-${side}-width`, '幅 [px]', 16, 1920, 1)}
    ${numeric_field(`camera-${side}-height`, '高さ [px]', 16, 1080, 1)}
    ${numeric_field(`camera-${side}-fx`, '焦点距離 fx [px]', 1, 20000)}
    ${numeric_field(`camera-${side}-fy`, '焦点距離 fy [px]', 1, 20000)}
    ${numeric_field(`camera-${side}-cx`, '主点 cx [px]', -0.5, 1919.5)}
    ${numeric_field(`camera-${side}-cy`, '主点 cy [px]', -0.5, 1079.5)}
    <label>レンズ方式<select id="camera-${side}-distortion"><option value="plumb_bob">通常レンズ</option><option value="equidistant">魚眼レンズ</option></select></label>
    ${numeric_field(`camera-${side}-max_fov_deg`, '有効画角上限 [°]', 1, 180)}
    ${Array.from({length: 5}, (_, idx) => `<label id="camera-${side}-coefficient-${idx}"><span></span><input id="camera-${side}-d-${idx}" type="number" step="any"></label>`).join('')}
    </div></details>`;
}

export class camera_workspace {
  constructor({scene, renderer, robot, environment, exclude, toast, download, aim}) {
    Object.assign(this, {robot, environment, exclude, toast, download});
    this.sensor = new color_camera(renderer, scene);
    this.is_live = false;
    this.is_pending = false;
    this.generation = 0;
    this.next_capture_ms = 0;
    this.last_frame = null;
    this.draft_sizes = {};
    element('camera-panel').innerHTML = `
      <div class="panel-heading"><h2>カメラ</h2><span class="chip">画像取得</span></div>
      <label class="field-label">汎用プリセット<select id="camera-mode">
        <option value="mono">単眼</option><option value="stereo">ステレオ</option><option value="fisheye">魚眼</option>
      </select></label>
      <div class="row-actions"><button id="camera-aim">テーブルを見る</button><button id="camera-once">1回取得</button><button id="camera-live">▶ 連続取得</button></div>
      <div class="sensor-images">
        <figure><figcaption id="camera-left-label">単眼</figcaption><canvas id="camera-left" width="640" height="480"></canvas></figure>
        <figure id="camera-right-figure" hidden><figcaption>右画像</figcaption><canvas id="camera-right" width="640" height="480"></canvas></figure>
      </div>
      <output id="camera-status" class="sensor-stats">未取得</output>
      <button id="camera-export" class="wide-button" disabled>↓ 画像・校正値を保存</button>
      <details id="camera-capture-settings"><summary>撮影設定</summary>
        <div class="field-grid">
          <label>解像度（左右共通）<select id="camera-size">${size_options.map(size => `<option value="${size}">${size.replace(',', ' × ')}</option>`).join('')}<option value="custom">個別・任意サイズ</option></select></label>
          <label>出力<select id="camera-color"><option value="rgb">カラー</option><option value="monochrome">モノクロ</option></select></label>
          ${numeric_field('camera-rate', '取得上限 [Hz]', 1, 30, 1)}
          <label>内部描画品質<select id="camera-quality"><option value="256">軽量 · 256 px</option><option value="512">標準 · 512 px</option><option value="1024">高精細 · 1024 px</option><option value="2048">最高精細 · 2048 px</option></select></label>
        </div>
        ${lens_editor('left', '単眼・左レンズの解像度と歪み')}
        ${lens_editor('right', '右レンズの解像度と歪み')}
        <details><summary>取付位置・左右間隔</summary><div class="field-grid">
          ${['x', 'y', 'z'].map(axis => numeric_field(`camera-mount-${axis}`, `位置 ${axis.toUpperCase()} [m]`, -10, 10)).join('')}
          ${['roll', 'pitch', 'yaw'].map((axis, idx) => numeric_field(`camera-mount-${axis}`, `${['ロール', 'ピッチ', 'ヨー'][idx]} [°]`, -360, 360)).join('')}
          <label id="camera-baseline-field">左右間隔 [mm]<input id="camera-baseline" type="number" min="0.1" max="2000" step="any"></label>
        </div></details>
        <div class="row-actions"><button id="camera-settings-apply">設定を適用</button></div>
        <p class="sub-note">レンズ欄で任意の解像度・校正値を指定。解像度変更時は画角を維持。適用後の次の撮影から反映。</p>
      </details>
      <details><summary>詳細JSON・読込</summary>
        <p class="sub-note">全設定の保存・読込と、右カメラの回転・並進の個別指定。初期値は未実機校正。</p>
        <label class="field-label" for="camera-calibration">校正JSON</label><textarea id="camera-calibration" spellcheck="false"></textarea>
        <div class="row-actions"><button id="camera-apply">JSONを適用</button><button id="camera-load">JSON読込</button><input id="camera-file" type="file" accept=".json,application/json" hidden></div>
      </details>
      <p class="sub-note">RGB-Dとは独立。幾何・レンズ投影のみ。露光・ノイズ・ROS画像配信は未対応。</p>`;
    element('camera-mode').onchange = event => {
      const preset = camera_preset(event.target.value);
      for (const key of ['color_mode', 'rate_hz', 'cube_size', 'mount']) preset[key] = structuredClone(this.sensor.config[key]);
      this.try_configure(preset);
    };
    element('camera-size').onchange = event => {
      if (event.target.value === 'custom') { element('camera-left-settings').open = true; return; }
      const [width, height] = event.target.value.split(',').map(Number);
      for (const side of this.sensor.config.mode === 'stereo' ? ['left', 'right'] : ['left']) this.resize_fields(side, width, height);
    };
    for (const side of ['left', 'right']) {
      for (const field of ['width', 'height']) element(`camera-${side}-${field}`).onchange = () => {
        this.resize_fields(side, element(`camera-${side}-width`).valueAsNumber, element(`camera-${side}-height`).valueAsNumber);
        element('camera-size').value = 'custom';
      };
      element(`camera-${side}-distortion`).onchange = () => {
        for (let idx = 0; idx < 5; idx++) element(`camera-${side}-d-${idx}`).value = 0;
        if (element(`camera-${side}-distortion`).value === 'plumb_bob') element(`camera-${side}-max_fov_deg`).value = Math.min(179, element(`camera-${side}-max_fov_deg`).valueAsNumber);
        this.sync_coefficients(side);
      };
    }
    element('camera-aim').onclick = aim;
    element('camera-once').onclick = () => this.capture();
    element('camera-live').onclick = () => this.set_live(!this.is_live);
    element('camera-settings-apply').onclick = () => this.apply_settings();
    element('camera-apply').onclick = () => {
      try { this.configure(JSON.parse(element('camera-calibration').value)); }
      catch (error) { this.toast('校正エラー：' + error.message); }
    };
    element('camera-load').onclick = () => element('camera-file').click();
    element('camera-file').onchange = async event => {
      const file = event.target.files[0];
      event.target.value = '';
      if (!file) return;
      try {
        if (file.size > 100000) throw Error('校正JSONは100 KB以内');
        this.configure(JSON.parse(await file.text()));
      } catch (error) { this.toast('校正読込エラー：' + error.message); }
    };
    element('camera-export').onclick = () => this.export_capture().catch(error => this.toast('保存エラー：' + error.message));
    this.sync_settings();
  }

  set_live(is_live) {
    this.is_live = is_live;
    element('camera-live').textContent = is_live ? '■ 停止' : '▶ 連続取得';
    this.sync_controls();
  }

  sync_controls() {
    // フレームごとの明滅防止。連続取得中は無効状態を維持
    const is_disabled = this.is_live || this.is_pending;
    for (const id of ['camera-once', 'camera-mode', 'camera-settings-apply', 'camera-apply', 'camera-load']) {
      const control = element(id);
      if (control.disabled !== is_disabled) control.disabled = is_disabled;
    }
  }

  sync_settings() {
    const config = this.sensor.config;
    element('camera-mode').value = config.mode;
    element('camera-rate').value = config.rate_hz;
    element('camera-color').value = config.color_mode;
    element('camera-quality').value = config.cube_size;
    const size = `${config.left.width},${config.left.height}`;
    const is_same_size = config.mode !== 'stereo' || (config.right.width === config.left.width && config.right.height === config.left.height);
    element('camera-size').value = is_same_size && size_options.includes(size) ? size : 'custom';
    for (const side of config.mode === 'stereo' ? ['left', 'right'] : ['left']) {
      const lens = config[side];
      for (const key of lens_fields) element(`camera-${side}-${key}`).value = lens[key];
      element(`camera-${side}-distortion`).value = lens.distortion_model;
      element(`camera-${side}-distortion`).querySelector('[value=plumb_bob]').disabled = config.mode === 'fisheye';
      for (let idx = 0; idx < 5; idx++) element(`camera-${side}-d-${idx}`).value = lens.d[idx] ?? 0;
      this.draft_sizes[side] = [lens.width, lens.height];
      this.sync_coefficients(side);
    }
    element('camera-right-settings').hidden = config.mode !== 'stereo';
    element('camera-baseline-field').hidden = config.mode !== 'stereo';
    element('camera-baseline').value = config.mode === 'stereo' ? Math.hypot(...config.right_in_left.translation_m) * 1000 : 60;
    ['x', 'y', 'z'].forEach((axis, idx) => element(`camera-mount-${axis}`).value = config.mount.translation_m[idx]);
    ['roll', 'pitch', 'yaw'].forEach((axis, idx) => element(`camera-mount-${axis}`).value = config.mount.rpy_deg[idx]);
    element('camera-calibration').value = JSON.stringify(config, null, 2);
    element('camera-right-figure').hidden = config.mode !== 'stereo';
    element('camera-left-label').textContent = config.mode === 'stereo' ? '左画像' : mode_labels[config.mode];
  }

  sync_coefficients(side) {
    const is_fisheye = element(`camera-${side}-distortion`).value === 'equidistant';
    const names = is_fisheye ? ['k1', 'k2', 'k3', 'k4'] : ['k1', 'k2', 'p1', 'p2', 'k3'];
    element(`camera-${side}-max_fov_deg`).max = is_fisheye ? 180 : 179;
    for (let idx = 0; idx < 5; idx++) {
      const label = element(`camera-${side}-coefficient-${idx}`);
      label.hidden = idx >= names.length;
      label.querySelector('span').textContent = names[idx] ? `歪み ${names[idx]}` : '';
    }
  }

  resize_fields(side, width, height) {
    if (!Number.isInteger(width) || !Number.isInteger(height) || width < 16 || width > 1920 || height < 16 || height > 1080) return;
    const [previous_width, previous_height] = this.draft_sizes[side];
    const scale_x = width / previous_width, scale_y = height / previous_height;
    // 画素中心と画角を保持する内部パラメータのリサイズ
    for (const [key, scale] of [['fx', scale_x], ['fy', scale_y], ['cx', scale_x], ['cy', scale_y]]) {
      const field = element(`camera-${side}-${key}`), offset = key.startsWith('c') ? 0.5 : 0;
      field.value = (field.valueAsNumber + offset) * scale - offset;
    }
    element(`camera-${side}-width`).value = width;
    element(`camera-${side}-height`).value = height;
    this.draft_sizes[side] = [width, height];
  }

  clear_frame() {
    this.generation++;
    this.last_frame = null;
    element('camera-export').disabled = true;
    element('camera-status').textContent = '未取得';
    for (const side of ['left', 'right']) {
      const canvas = element('camera-' + side);
      canvas.getContext('2d').clearRect(0, 0, canvas.width, canvas.height);
    }
  }

  configure(config) {
    this.sensor.configure(config);
    this.set_live(false);
    this.clear_frame();
    this.sync_settings();
  }

  try_configure(config) {
    try { this.configure(config); }
    catch (error) { this.sync_settings(); this.toast('設定エラー：' + error.message); }
  }

  apply_settings() {
    const config = structuredClone(this.sensor.config);
    config.rate_hz = element('camera-rate').valueAsNumber;
    config.color_mode = element('camera-color').value;
    config.cube_size = Number(element('camera-quality').value);
    for (const side of config.mode === 'stereo' ? ['left', 'right'] : ['left']) {
      const lens = config[side];
      for (const key of lens_fields) lens[key] = element(`camera-${side}-${key}`).valueAsNumber;
      lens.distortion_model = element(`camera-${side}-distortion`).value;
      lens.d = Array.from({length: lens.distortion_model === 'equidistant' ? 4 : 5}, (_, idx) => element(`camera-${side}-d-${idx}`).valueAsNumber);
    }
    config.mount.translation_m = ['x', 'y', 'z'].map(axis => element(`camera-mount-${axis}`).valueAsNumber);
    config.mount.rpy_deg = ['roll', 'pitch', 'yaw'].map(axis => element(`camera-mount-${axis}`).valueAsNumber);
    if (config.mode === 'stereo') {
      const scale = element('camera-baseline').valueAsNumber / 1000 / Math.hypot(...config.right_in_left.translation_m);
      config.right_in_left.translation_m = config.right_in_left.translation_m.map(value => value * scale);
    }
    this.try_configure(config);
  }

  reset_for_robot(robot) {
    const previous_link = this.robot.links.camera_link;
    this.exclude = this.exclude.map(object => object === previous_link ? robot.links.camera_link : object);
    this.robot = robot;
    this.set_live(false);
    this.clear_frame();
  }

  tick(now) {
    if (this.is_live && !this.is_pending && !document.hidden && now >= this.next_capture_ms) this.capture();
  }

  async capture() {
    if (this.is_pending) return null;
    this.is_pending = true;
    const generation = this.generation, started_ms = performance.now();
    this.sync_controls();
    try {
      this.robot.updateWorldMatrix(true, true);
      const optical_to_world = this.robot.links.camera_optical_frame.matrixWorld.clone();
      const metadata = {robot_model: this.robot.modelId, joint_positions: {...this.robot.getPose()},
        robot_to_world: this.robot.matrixWorld.toArray(), scene: structuredClone(this.environment.getState())};
      const frame = await this.sensor.capture(optical_to_world, this.exclude);
      if (generation !== this.generation) return null;
      Object.assign(frame, metadata);
      frame.capture_ms = performance.now() - started_ms;
      for (const [side, data] of Object.entries(frame.images)) {
        const canvas = element('camera-' + side);
        if (canvas.width !== data.width) canvas.width = data.width;
        if (canvas.height !== data.height) canvas.height = data.height;
        canvas.getContext('2d').putImageData(new ImageData(data.rgba, data.width, data.height), 0, 0);
      }
      this.last_frame = frame;
      element('camera-export').disabled = false;
      element('camera-status').textContent = `${mode_labels[frame.calibration.mode]} · ${frame.calibration.color_mode === 'monochrome' ? 'モノクロ' : 'カラー'} · #${frame.id}\n${Object.entries(frame.images).map(([side, data]) => `${side === 'left' ? '画像' : '右'}: ${data.width} × ${data.height} px`).join(' / ')}\n取得時間: ${frame.capture_ms.toFixed(0)} ms · シーン描画: ${frame.num_scene_passes} 面`;
      return frame;
    } catch (error) {
      if (generation === this.generation) {
        this.set_live(false);
        element('camera-status').textContent = '取得エラー：' + error.message;
        this.toast('カメラ：' + error.message);
      }
      return null;
    } finally {
      this.is_pending = false;
      this.sync_controls();
      this.next_capture_ms = performance.now() + Math.max(1000 / this.sensor.config.rate_hz, performance.now() - started_ms);
    }
  }

  async export_capture() {
    const frame = this.last_frame;
    if (!frame) return;
    const {images, scene, ...metadata} = frame;
    metadata.images = {};
    const entries = {'calibration.json': JSON.stringify(frame.calibration, null, 2), 'scene.json': JSON.stringify(scene, null, 2)};
    for (const [side, data] of Object.entries(images)) {
      const canvas = document.createElement('canvas');
      canvas.width = data.width; canvas.height = data.height;
      canvas.getContext('2d').putImageData(new ImageData(data.rgba, data.width, data.height), 0, 0);
      const file = `${side}.png`;
      entries[file] = await new Promise((resolve, reject) => canvas.toBlob(blob => blob ? resolve(blob) : reject(Error('PNG生成失敗')), 'image/png'));
      metadata.images[side] = {file, width: data.width, height: data.height, optical_to_world: data.optical_to_world};
    }
    entries['frame.json'] = JSON.stringify(metadata, null, 2);
    await this.download('Camera-capture.zip', await zip_files(entries), 'application/zip');
  }
}
