import assert from 'node:assert/strict';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
import net from 'node:net';
import path from 'node:path';
import {unlink, mkdtemp, rm} from 'node:fs/promises';
import {tmpdir} from 'node:os';
import {fileURLToPath} from 'node:url';

// 任意のPlaywright導入先と独立ブラウザ・空きポートでの検証
const {chromium} = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const capture_dir = await mkdtemp(path.join(tmpdir(), 'topo-camera-test-'));
const probe = net.createServer(); probe.listen(0, '127.0.0.1'); await once(probe, 'listening');
const port = probe.address().port; await new Promise(resolve => probe.close(resolve));
const server = spawn(process.execPath, ['app/server.mjs'], {cwd: root, env: {...process.env, PORT: String(port)}, stdio: 'pipe'});
const base = `http://127.0.0.1:${port}`;
let browser, saved_url;
const deadline = setTimeout(() => {
  process.exitCode = 1;
  console.error('試験時間上限: 180秒');
  void browser?.close();
  server.kill('SIGTERM');
}, 180000);
const pause = ms => new Promise(resolve => setTimeout(resolve, ms));
try {
  for (let iter = 0; iter < 100; iter++) {
    try { if ((await fetch(base + '/api/health')).ok) break; } catch {}
    await pause(50);
  }
  browser = await chromium.launch({headless: true, executablePath: process.env.CHROME_PATH || '/usr/bin/google-chrome',
    args: ['--use-gl=angle', '--use-angle=' + (process.env.CAMERA_TEST_SOFTWARE ? 'swiftshader' : 'gl-egl'), '--enable-unsafe-swiftshader'],
    env: {...process.env, __NV_PRIME_RENDER_OFFLOAD: '0', __GLX_VENDOR_LIBRARY_NAME: 'mesa', __EGL_VENDOR_LIBRARY_FILENAMES: '/usr/share/glvnd/egl_vendor.d/50_mesa.json'}});
  const page = await browser.newPage({viewport: {width: 1400, height: 1000}});
  const errors = [];
  let num_vm_unavailable = 0;
  page.on('pageerror', error => errors.push(error.message));
  page.on('console', message => {
    if (message.type() !== 'error') return;
    // 単体Nodeサーバーに含まれない既存VM接続確認のみ別集計
    if (message.location().url === base + '/api/status' && message.text().includes('404')) { num_vm_unavailable++; return; }
    errors.push(message.text() + ' ' + message.location().url);
  });
  await page.goto(base + '/?model=long');
  await page.waitForFunction(() => window.simulator?.diagnostics.ready, null, {timeout: 90000});
  console.log('モデル起動: Long');
  const projection = await page.evaluate(async () => {
    const three = await import('/vendor/three/build/three.module.js');
    const {camera_preset, color_camera} = await import('/camera-core.js');
    const renderer = new three.WebGLRenderer();
    renderer.outputColorSpace = three.SRGBColorSpace;
    const scene = new three.Scene(); scene.background = new three.Color(0);
    const resources = [];
    const marker = (x, y, z, color, radius = 0.1) => {
      const geometry = new three.SphereGeometry(radius, 20, 12), material = new three.MeshBasicMaterial({color});
      resources.push(geometry, material);
      const mesh = new three.Mesh(geometry, material); mesh.position.set(x, y, z); scene.add(mesh); return mesh;
    };
    marker(0, 0, 2, 0xff0000);
    marker(0.4, 0.3, 2, 0x00ff00);
    marker(-0.5, -0.4, 2, 0x0000ff);
    const theta = 80 * Math.PI / 180;
    marker(2 * Math.sin(theta), 0, 2 * Math.cos(theta), 0xffffff);
    const excluded = marker(0, 0, 0.3, 0xffffff);
    const config = camera_preset('stereo');
    config.cube_size = 256;
    Object.assign(config.left, {width: 160, height: 120, fx: 120, fy: 110, cx: 77.2, cy: 61.7});
    config.right = {...config.left, fx: 108, cx: 74.3};
    config.right_in_left.translation_m = [0.12, 0, 0];
    const sensor = new color_camera(renderer, scene, config);
    const check = (is_passed, name) => { if (!is_passed) throw Error(name); };
    const pixel = (image, u, v) => Array.from(image.rgba.slice((Math.round(v) * image.width + Math.round(u)) * 4, (Math.round(v) * image.width + Math.round(u)) * 4 + 4));
    const has_color = (image, u, v, channel) => {
      const color = pixel(image, u, v);
      return color[channel] > 220 && color.filter((value, idx) => idx < 3 && idx !== channel).every(value => value < 25);
    };
    try {
      renderer.autoClear = false; renderer.shadowMap.needsUpdate = true;
      const pending = sensor.capture(new three.Matrix4(), [excluded]);
      check(renderer.autoClear === false && renderer.shadowMap.needsUpdate && excluded.visible, '描画状態の即時復旧');
      let has_rejected = false;
      try { sensor.configure(config); } catch { has_rejected = true; }
      check(has_rejected, '取得中の校正変更拒否');
      const stereo = await pending;
      for (const [side, image] of Object.entries(stereo.images)) {
        const lens = config[side], offset = side === 'right' ? 0.12 : 0;
        check(has_color(image, lens.cx - lens.fx * offset / 2, lens.cy, 0), side + '赤色中心・基線 ' + JSON.stringify({pixel: pixel(image, lens.cx - lens.fx * offset / 2, lens.cy), error: renderer.getContext().getError()}));
        check(has_color(image, lens.cx + lens.fx * (0.4 - offset) / 2, lens.cy + lens.fy * 0.3 / 2, 1), side + '右下の緑色');
        check(has_color(image, lens.cx + lens.fx * (-0.5 - offset) / 2, lens.cy - lens.fy * 0.4 / 2, 2), side + '左上の青色');
      }
      const fish = camera_preset('fisheye'); fish.cube_size = 256;
      Object.assign(fish.left, {width: 240, height: 240, fx: 70, fy: 70, cx: 119.5, cy: 119.5});
      sensor.configure(fish);
      const wide = (await sensor.capture(new three.Matrix4(), [excluded])).images.left;
      check(pixel(wide, 119.5 + 70 * theta, 119.5).slice(0, 3).every(value => value > 220), '魚眼80°側方の可視性');
      check(pixel(wide, 0, 0)[3] === 0, '魚眼画角外の透明マスク');
      check(has_color(wide, 119.5, 119.5, 0), '魚眼中心の赤色');
      const red = marker(2, 0, 0, 0xff0000);
      const rotated = (await sensor.capture(new three.Matrix4().makeRotationY(Math.PI / 2), [excluded])).images.left;
      check(has_color(rotated, 119.5, 119.5, 0), '取付回転の反映');
      scene.remove(red);
      const original_render = renderer.render;
      renderer.render = () => { throw Error('試験用描画失敗'); };
      let has_failed = false;
      try { await sensor.capture(new three.Matrix4(), [excluded]); } catch { has_failed = true; }
      renderer.render = original_render;
      check(has_failed && !sensor.is_pending && excluded.visible && !renderer.autoClear && renderer.shadowMap.needsUpdate, '描画例外後の復旧');
      return {stereo: '左右の投影・視差', fisheye: '80°側方・画角外', transform: '回転', restoration: '正常・例外'};
    } finally {
      sensor.dispose(); resources.forEach(resource => resource.dispose()); renderer.dispose();
    }
  });
  console.log(JSON.stringify(projection));
  await page.click('[data-panel=camera]');
  await page.click('#camera-aim');
  await pause(1800);
  for (const mode of ['mono', 'stereo', 'fisheye']) {
    await page.selectOption('#camera-mode', mode);
    const frame = await page.evaluate(async () => {
      const panel = simulator.color_camera_panel, frame = await panel.capture();
      if (!frame) throw Error(document.getElementById('camera-status').textContent);
      return {mode: frame.calibration.mode, model: frame.robot_model, sides: Object.keys(frame.images),
        width: frame.images.left.width, height: frame.images.left.height,
        num_pixels: frame.images.left.rgba.filter((value, idx) => idx % 4 !== 3 && value > 0).length};
    });
    assert.equal(frame.mode, mode); assert.equal(frame.model, 'long');
    assert.equal(frame.sides.length, mode === 'stereo' ? 2 : 1); assert.ok(frame.num_pixels > 1000);
    console.log(JSON.stringify(frame));
    if (mode === 'fisheye') await page.screenshot({path: path.join(capture_dir, 'fisheye.png')});
  }
  await page.evaluate(() => {
    const panel = simulator.color_camera_panel;
    const before = JSON.stringify(panel.sensor.config);
    document.getElementById('camera-calibration').value = '{}';
    document.getElementById('camera-apply').click();
    if (JSON.stringify(panel.sensor.config) !== before) throw Error('不正JSONによる設定破壊');
    const config = structuredClone(panel.sensor.config);
    config.left.d = [0.01, 0, 0, 0]; config.mount.translation_m = [0.01, 0.02, 0.03];
    document.getElementById('camera-calibration').value = JSON.stringify(config);
    document.getElementById('camera-apply').click();
    if (panel.sensor.config.left.d[0] !== 0.01 || panel.last_frame !== null) throw Error('校正適用と旧画像破棄');
  });
  await page.selectOption('#camera-mode', 'stereo');
  await page.evaluate(() => simulator.color_camera_panel.capture());
  await page.screenshot({path: process.env.CAMERA_SCREENSHOT || path.join(capture_dir, 'stereo.png')});
  await page.click('#camera-export');
  await page.waitForFunction(() => simulator.diagnostics.lastExport?.endsWith('-Camera-capture.zip'));
  saved_url = await page.evaluate(() => simulator.diagnostics.lastExport);
  const archive = Buffer.from(await (await fetch(base + saved_url)).arrayBuffer());
  const entries = {};
  for (let offset = 0; archive.readUInt32LE(offset) === 0x04034b50;) {
    assert.equal(archive.readUInt16LE(offset + 8), 0);
    const size = archive.readUInt32LE(offset + 18), name_length = archive.readUInt16LE(offset + 26);
    const name = archive.subarray(offset + 30, offset + 30 + name_length).toString();
    const start = offset + 30 + name_length + archive.readUInt16LE(offset + 28);
    entries[name] = archive.subarray(start, start + size); offset = start + size;
  }
  assert.deepEqual(Object.keys(entries).sort(), ['left.png', 'right.png', 'frame.json', 'calibration.json', 'scene.json'].sort());
  const metadata = JSON.parse(entries['frame.json']);
  assert.deepEqual(JSON.parse(entries['calibration.json']), metadata.calibration);
  for (const side of ['left', 'right']) {
    const png = entries[side + '.png'];
    assert.equal(png.subarray(0, 8).toString('hex'), '89504e470d0a1a0a');
    assert.equal(png.readUInt32BE(16), metadata.images[side].width);
    assert.equal(png.readUInt32BE(20), metadata.images[side].height);
  }
  assert.notDeepEqual(entries['left.png'], entries['right.png']);
  await page.evaluate(async () => {
    const panel = simulator.color_camera_panel;
    const pending = panel.capture(); panel.reset_for_robot(simulator.robot);
    await pending;
    if (panel.last_frame !== null) throw Error('旧世代のフレーム残留');
    await simulator.switchModel('standard');
    const frame = await panel.capture();
    if (frame?.robot_model !== 'standard') throw Error('標準モデルの撮影失敗');
  });
  console.log('標準モデル・旧フレーム破棄・ZIP保存: 成功');
  await page.evaluate(() => {
    simulator.color_camera_panel.set_live(true);
    window.camera_button_changes = [];
    window.camera_button_observer = new MutationObserver(records => camera_button_changes.push(...records.map(record => record.target.id)));
    for (const id of ['camera-once', 'camera-mode', 'camera-settings-apply', 'camera-apply', 'camera-load']) {
      const button = document.getElementById(id);
      if (!button.disabled) throw Error('連続取得中のボタン無効化');
      camera_button_observer.observe(button, {attributes: true, attributeFilter: ['disabled']});
    }
  });
  const previous_id = await page.evaluate(() => simulator.color_camera_panel.last_frame.id);
  await page.waitForFunction(id => simulator.color_camera_panel.last_frame?.id >= id + 3, previous_id);
  assert.deepEqual(await page.evaluate(() => camera_button_changes), [], '連続取得3フレームのボタン明滅防止');
  await page.evaluate(() => { camera_button_observer.disconnect(); simulator.color_camera_panel.set_live(false); });
  await page.waitForFunction(() => !simulator.color_camera_panel.is_pending);
  assert.equal(await page.isDisabled('#camera-once'), false, '停止後の手動取得復帰');
  await page.evaluate(async () => {
    const panel = simulator.color_camera_panel;
    const pending = panel.capture();
    if (!document.getElementById('camera-once').disabled) throw Error('手動取得中の二重操作防止');
    await pending;
    if (document.getElementById('camera-once').disabled) throw Error('手動取得後のボタン復帰');
  });
  console.log('連続取得3フレームのボタン状態変化: 0回／停止・手動取得後の復帰: 成功');
  const rgbd = await page.evaluate(async () => {
    const {runSensorQA} = await import('/rgbd-qa.js');
    return await runSensorQA(simulator.renderer);
  });
  console.log(JSON.stringify({rgbd: {passed: rgbd.passed, num_tests: rgbd.tests.length}}));
  assert.equal(rgbd.passed, true, 'RGB-D回帰');
  assert.deepEqual(errors, [], 'ブラウザエラー');
  console.log(`既存VM未接続の404（カメラ試験対象外）: ${num_vm_unavailable}件`);
  console.log('カメラUI・WebGL検証: 成功');
} finally {
  clearTimeout(deadline);
  try {
    await browser?.close();
    if (saved_url) await unlink(path.join(root, 'app', saved_url));
    await rm(capture_dir, {recursive: true, force: true});
  } finally {
    if (server.exitCode === null && server.signalCode === null) {
      const exited = once(server, 'exit'); server.kill('SIGTERM'); await exited;
    }
  }
  console.log('試験用ブラウザ・HTTPサーバー: 停止済み');
}
