import {camera_preset, color_camera} from './camera-core.js';
import {zipFiles as zip_files} from './capture-zip.js';

const element = id => document.getElementById(id);
const mode_labels = {mono: '単眼', stereo: 'ステレオ', fisheye: '魚眼'};

export class camera_workspace {
  constructor({scene, renderer, robot, environment, exclude, toast, download, aim}) {
    Object.assign(this, {robot, environment, exclude, toast, download});
    this.sensor = new color_camera(renderer, scene);
    this.is_live = false;
    this.is_pending = false;
    this.generation = 0;
    this.next_capture_ms = 0;
    this.last_frame = null;
    element('camera-panel').innerHTML = `
      <div class="panel-heading"><h2>カメラ</h2><span class="chip">カラー画像</span></div>
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
      <details open><summary>撮影設定</summary>
        <div class="field-grid">
          <label>解像度<select id="camera-size"><option value="640,480">640 × 480</option><option value="1280,720">1280 × 720</option><option value="1920,1080">1920 × 1080</option><option value="custom">校正値のまま</option></select></label>
          <label>取得上限 [Hz]<input id="camera-rate" type="number" min="1" max="30" step="1" value="5"></label>
        </div>
        <div class="row-actions"><button id="camera-settings-apply">撮影設定を適用</button></div>
      </details>
      <details><summary>実機校正・取付位置</summary>
        <p class="sub-note">fx・fy・cx・cy: px／取付位置: m／取付角度: °。初期値は未実機校正。左右の個別設定はJSONで指定。</p>
        <label class="field-label" for="camera-calibration">校正JSON</label><textarea id="camera-calibration" spellcheck="false"></textarea>
        <div class="row-actions"><button id="camera-apply">JSONを適用</button><button id="camera-load">JSON読込</button><input id="camera-file" type="file" accept=".json,application/json" hidden></div>
      </details>
      <p class="sub-note">RGB-Dとは独立。幾何・レンズ投影のみ。露光・ノイズ・ROS画像配信は未対応。</p>`;
    element('camera-mode').onchange = event => this.try_configure(camera_preset(event.target.value));
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
    const size = `${config.left.width},${config.left.height}`;
    element('camera-size').value = ['640,480', '1280,720', '1920,1080'].includes(size) ? size : 'custom';
    element('camera-calibration').value = JSON.stringify(config, null, 2);
    element('camera-right-figure').hidden = config.mode !== 'stereo';
    element('camera-left-label').textContent = config.mode === 'stereo' ? '左画像' : mode_labels[config.mode];
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
    config.rate_hz = Number(element('camera-rate').value);
    const size = element('camera-size').value;
    if (size !== 'custom') {
      const [width, height] = size.split(',').map(Number);
      for (const side of config.mode === 'stereo' ? ['left', 'right'] : ['left']) {
        const lens = config[side], scale_x = width / lens.width, scale_y = height / lens.height;
        // 画素中心を保持するリサイズ。実機の別撮影モードでは個別の校正値を使用
        lens.fx *= scale_x; lens.fy *= scale_y;
        lens.cx = (lens.cx + 0.5) * scale_x - 0.5;
        lens.cy = (lens.cy + 0.5) * scale_y - 0.5;
        lens.width = width; lens.height = height;
      }
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
      element('camera-status').textContent = `${mode_labels[frame.calibration.mode]} · #${frame.id}\n${Object.entries(frame.images).map(([side, data]) => `${side === 'left' ? '画像' : '右'}: ${data.width} × ${data.height} px`).join(' / ')}\n取得時間: ${frame.capture_ms.toFixed(0)} ms`;
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
