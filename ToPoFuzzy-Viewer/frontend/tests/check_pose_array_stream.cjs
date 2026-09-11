// Docker内のROS入力から実WebSocketまでの結合確認。既存Viewerの停止なし
const assert = require('node:assert/strict');
const { spawn } = require('node:child_process');
const WebSocket = require('ws');
const { once } = require('node:events');

async function main() {
    const command = ['exec', '-i', 'gng_cpu_container', 'bash', '-lc',
        'source /ros2_ws/install/setup.bash\nROS_DOMAIN_ID=217 ROS_LOCALHOST_ONLY=1 timeout --signal=INT --kill-after=15 100 python3 /ros2_ws/src/ToPoFuzzy-Viewer/backend/src/topo_fuzzy_viewer/test/pose_array_stream_fixture.py'];
    console.log('起動: docker ' + command.join(' '));
    const fixture = spawn('docker', command, { stdio: ['pipe', 'pipe', 'inherit'] });
    let stdout = '';
    fixture.stdout.on('data', chunk => { stdout += chunk.toString(); });
    const completed = once(fixture, 'exit');
    let socket;
    const messages = [];
    let req_num = 0;
    const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
    async function wait_for(predicate, label) {
        const deadline = Date.now() + 10000;
        while (Date.now() < deadline) {
            const value = predicate();
            if (value) return value;
            if (fixture.exitCode !== null) throw Error('fixture終了: ' + stdout);
            await sleep(30);
        }
        throw Error('時間超過: ' + label);
    }
    async function connect() {
        for (let iter = 0; iter < 40; ++iter) {
            const candidate = new WebSocket('ws://127.0.0.1:19091');
            const has_opened = await new Promise(resolve => {
                candidate.once('open', () => resolve(true));
                candidate.once('error', () => resolve(false));
            });
            if (has_opened) {
                candidate.on('message', (data, is_binary) => {
                    if (!is_binary) messages.push(JSON.parse(data.toString()));
                });
                return candidate;
            }
            await sleep(100);
        }
        throw Error('WebSocket接続失敗');
    }
    async function rpc(method, params = {}) {
        const id = `probe_${++req_num}`;
        socket.send(JSON.stringify({ id, method, params }));
        const response = await wait_for(() => messages.find(msg => msg.id === id), method);
        assert.equal(response.ok, true, JSON.stringify(response));
        return response.result;
    }
    const is_pose_packet = msg => msg.type === 'stream.marker_array' && msg.tag === '/grasp_pose_cands';
    try {
        await wait_for(() => stdout.includes('READY'), 'fixture起動');
        socket = await connect();
        let has_source = false;
        for (let iter = 0; iter < 30 && !has_source; ++iter) {
            const result = await rpc('sources.list');
            has_source = result.sources.some(source => source.id === '/grasp_pose_cands' && source.type === 'marker');
            if (!has_source) await sleep(100);
        }
        assert.equal(has_source, true);
        // 購読前の1回配信をtransient-localで受信
        fixture.stdin.write(JSON.stringify({ stamp: 11, poses: [
            [[1, 2, 3], [2, 0, 0, 0]], [[0, 0, 0], [0, 0, 0, 0]],
        ] }) + '\n');
        await wait_for(() => stdout.includes('PUBLISHED'), '候補配信');
        await rpc('sources.setActive', { sourceId: '/grasp_pose_cands', active: true });
        const first = await wait_for(() => messages.find(is_pose_packet), '候補ストリーム');
        assert.equal(first.source_type, 'pose_array');
        assert.equal(first.markers.length, 1);
        assert.equal(first.markers[0].frameId, 'graspnet_table');
        assert.deepEqual(first.markers[0].points[0], [1, 2, 3]);
        assert.ok(Math.abs(first.markers[0].points[1][2] - 2.92) < 1e-9);
        fixture.stdin.write(JSON.stringify({ stamp: 12, poses: [] }) + '\n');
        await wait_for(() => messages.find(msg => is_pose_packet(msg) && msg.markers.length === 0), '空候補');
        socket.close();
        await once(socket, 'close');
        messages.length = 0;
        socket = await connect();
        socket.send(JSON.stringify({ type: 'request.state' }));
        await wait_for(() => messages.find(msg => is_pose_packet(msg) && msg.markers.length === 0), '再接続時の空状態');
        await rpc('sources.setActive', { sourceId: '/grasp_pose_cands', active: false, removeLayer: true });
        await wait_for(() => messages.find(msg => msg.type === 'stream.delete' && msg.tag === '/grasp_pose_cands'), '購読解除');
        console.log('成功: ソース一覧、保持サンプル、下向きZ軸、不正姿勢除外、空候補、再接続、購読解除');
    } catch (error) {
        console.error('受信状態:', JSON.stringify(messages.slice(-8)), stdout);
        throw error;
    } finally {
        if (socket) socket.close();
        fixture.stdin.end('STOP\n');
        const [code] = await completed;
        assert.equal(code, 0, stdout);
        assert.ok(stdout.includes('STOPPED'), stdout);
        console.log('停止済み: 検証gateway・入力ノード・ポート19091');
    }
}
main().catch(error => { console.error(error); process.exitCode = 1; });
