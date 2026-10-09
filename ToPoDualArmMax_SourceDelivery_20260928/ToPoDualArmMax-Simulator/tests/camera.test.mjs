import test from 'node:test';
import assert from 'node:assert/strict';
import {Matrix4} from '../app/vendor/three/build/three.module.js';
import {camera_preset, validate_camera, pixel_ray, project_ray, camera_transforms, camera_ray_map} from '../app/camera-core.js';

function close(actual, expected, tolerance = 1e-6) {
  assert.equal(actual.length, expected.length);
  actual.forEach((value, idx) => assert.ok(Math.abs(value - expected[idx]) < tolerance, `${idx}: ${value} / ${expected[idx]}`));
}

test('全プリセットの検証と左右設定の独立性', () => {
  for (const mode of ['mono', 'stereo', 'fisheye']) {
    const config = camera_preset(mode), result = validate_camera(config);
    assert.equal(result.mode, mode);
    result.left.fx = 1;
    assert.notEqual(config.left.fx, 1);
    assert.notEqual(result.right.fx, 1);
  }
});

test('非対称内部パラメータと画素中心のピンホール逆投影', () => {
  const lens = {...camera_preset().left, fx: 431, fy: 397, cx: 312.3, cy: 235.7};
  for (const [u, v] of [[0, 0], [639, 479], [lens.cx, lens.cy], [101, 205]]) {
    const expected = [(u - lens.cx) / lens.fx, (v - lens.cy) / lens.fy, 1];
    const length = Math.hypot(...expected);
    close(pixel_ray(lens, u, v), expected.map(value => value / length));
    close(project_ray(lens, expected), [u, v]);
  }
});

test('Brown–Conradyの独立計算との照合と逆投影', () => {
  const lens = {...camera_preset().left, d: [-0.16, 0.03, 0.002, -0.003, 0.005]};
  for (const [x, y] of [[0.1, 0.2], [-0.4, 0.3], [0.6, -0.4]]) {
    const squared = x ** 2 + y ** 2, radial = 1 - 0.16 * squared + 0.03 * squared ** 2 + 0.005 * squared ** 3;
    const u = lens.fx * (x * radial + 0.004 * x * y - 0.003 * (squared + 2 * x * x)) + lens.cx;
    const v = lens.fy * (y * radial + 0.002 * (squared + 2 * y * y) - 0.006 * x * y) + lens.cy;
    close(project_ray(lens, [x, y, 1]), [u, v]);
    close(pixel_ray(lens, u, v), [x, y, 1].map(value => value / Math.hypot(x, y, 1)));
  }
});

test('等距離魚眼の非ゼロ歪み・中心・画角マスク', () => {
  const lens = {...camera_preset('fisheye').left, d: [0.02, -0.001, 0.0002, 0.00001]};
  for (const theta of [0.1, 0.6, 1.3, Math.PI / 2]) {
    const radius = theta * (1 + 0.02 * theta ** 2 - 0.001 * theta ** 4 + 0.0002 * theta ** 6 + 0.00001 * theta ** 8);
    const ray = [Math.sin(theta), 0, Math.cos(theta)];
    close(project_ray(lens, ray), [lens.cx + lens.fx * radius, lens.cy]);
    close(pixel_ray(lens, lens.cx + lens.fx * radius - 1e-8, lens.cy), ray);
  }
  close(pixel_ray(lens, lens.cx, lens.cy), [0, 0, 1]);
  assert.equal(pixel_ray(lens, -200, -200), null);
  assert.equal(project_ray(lens, [0, 0, -1]), null);
});

test('不正・非対応の校正値の拒否', () => {
  const changes = [c => c.left.d = [1], c => c.left.width = 1921, c => c.left.fx = NaN,
    c => c.left.distortion_model = 'rational_polynomial', c => c.cube_size = 4096,
    c => c.mount.translation_m = [0, 0], c => c.right_in_left.rotation[0] = -1,
    c => c.right_in_left.translation_m = [0, 0, 0], c => c.right.fy = -1, c => c.rate_hz = Infinity];
  for (const change of changes) {
    const config = camera_preset('stereo'); change(config);
    assert.throws(() => validate_camera(config));
  }
  const config = camera_preset('fisheye'); config.left.d = [-1, 0, 0, 0];
  assert.throws(() => validate_camera(config), /折り返し/);
});

test('ステレオ基線と視差 fx × 基線長 / 奥行き', () => {
  const config = camera_preset('stereo'), matrices = camera_transforms(config, new Matrix4());
  close(matrices.left.elements.slice(12, 15), [0, 0, 0]);
  close(matrices.right.elements.slice(12, 15), [0.06, 0, 0]);
  const left = project_ray(config.left, [0.1, 0.2, 2]);
  const right = project_ray(config.right, [0.04, 0.2, 2]);
  close([left[0] - right[0], left[1] - right[1]], [config.left.fx * 0.06 / 2, 0]);
});

test('取付姿勢とロボット姿勢の合成・右眼回転の維持', () => {
  const config = camera_preset('stereo');
  config.mount.translation_m = [0.1, 0.2, 0.3]; config.mount.rpy_deg = [0, 0, 90];
  config.right_in_left.rotation = [0, -1, 0, 1, 0, 0, 0, 0, 1];
  const matrices = camera_transforms(validate_camera(config), new Matrix4().makeTranslation(1, 2, 3));
  close(matrices.left.elements.slice(12, 15), [1.1, 2.2, 3.3]);
  close(matrices.right.elements.slice(12, 15), [1.1, 2.26, 3.3]);
  close(matrices.right.elements.slice(0, 3), [-1, 0, 0]);
});

test('旧校正のカラー互換とモノクロ設定の検証', () => {
  const config = camera_preset(); delete config.color_mode;
  assert.equal(validate_camera(config).color_mode, 'rgb');
  config.color_mode = 'monochrome';
  assert.equal(validate_camera(config).color_mode, 'monochrome');
  config.color_mode = 'infrared';
  assert.throws(() => validate_camera(config), /出力方式/);
});

test('画角に対応する面・描画領域だけの選択', () => {
  for (const [mode, num_faces] of [['mono', 1], ['fisheye', 5]]) {
    const map = camera_ray_map(camera_preset(mode).left, 512);
    assert.equal(map.faces.length, num_faces);
    assert.ok(map.faces.every(face => face.face !== 5));
    for (const face of map.faces) {
      assert.ok(face.x >= 0 && face.y >= 0 && face.width > 0 && face.height > 0);
      assert.ok(face.x + face.width <= 512 && face.y + face.height <= 512);
    }
    assert.ok(map.faces.reduce((sum, face) => sum + face.width * face.height, 0) < num_faces * 512 ** 2);
  }
});
