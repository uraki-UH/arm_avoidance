"""関節WebSocketの境界値・送受信・切断時の購読解除検証。"""
import asyncio
import json
import socket
import unittest
import time
from types import SimpleNamespace
from tornado.httpclient import HTTPRequest, HTTPClientError
from tornado.websocket import websocket_connect
from joint_stream import start_joint_stream


class JointStreamTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        from builtin_interfaces.msg import Time
        self.subscriptions = []
        self.published = []
        def subscribe(kind, topic, callback, qos):
            sub = SimpleNamespace(topic=topic, callback=callback)
            self.subscriptions.append(sub)
            return sub
        node = SimpleNamespace(create_subscription=subscribe, destroy_subscription=self.subscriptions.remove,
                               get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)))
        exchange = SimpleNamespace(joint_names={'long': {'neck_pan_joint'}}, joints=SimpleNamespace(publish=self.published.append))
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            self.port = probe.getsockname()[1]
        self.stop = start_joint_stream(node, exchange, '127.0.0.1', self.port, {'http://localhost:8877'})
        self.clients = []

    async def asyncTearDown(self):
        for client in self.clients:
            client.close()
        await asyncio.sleep(.03)
        self.stop()

    async def connect(self, **overrides):
        client = await websocket_connect(HTTPRequest(f'ws://127.0.0.1:{self.port}/joints', headers={'Origin': 'http://localhost:8877'}))
        self.clients.append(client)
        await client.write_message(json.dumps(dict(dict(type='config', model='long', hz=100), **overrides)))
        return client, json.loads(await asyncio.wait_for(client.read_message(), 2))

    async def test_rate_bounds(self):
        for hz in (1, 200, 0, 201):
            client = await websocket_connect(HTTPRequest(f'ws://127.0.0.1:{self.port}/joints', headers={'Origin': 'http://localhost:8877'}))
            self.clients.append(client)
            await client.write_message(json.dumps(dict(type='config', model='long', hz=hz)))
            response = json.loads(await asyncio.wait_for(client.read_message(), 2))
            self.assertEqual(response['type'], 'ready' if hz in (1, 200) else 'error')
            client.close()

    async def test_joint_roundtrip_and_cleanup(self):
        client, response = await self.connect(receive=True, topic='/joint_states')
        self.assertEqual(response['type'], 'ready')
        await client.write_message(json.dumps(dict(type='joints', pose={'neck_pan_joint': .1})))
        self.assertEqual(json.loads(await client.read_message())['type'], 'ack')
        self.assertEqual(self.published[0].position[0], .1)
        self.subscriptions[0].callback(SimpleNamespace(name=['neck_pan_joint'], position=[.2]))
        response = json.loads(await asyncio.wait_for(client.read_message(), 2))
        self.assertEqual(response['pose'], {'neck_pan_joint': .2})
        client.close()
        await asyncio.sleep(.05)
        self.assertFalse(self.subscriptions)

    async def test_one_hz_receive(self):
        client, response = await self.connect(hz=1, receive=True)
        self.assertEqual(response['type'], 'ready')
        self.subscriptions[0].callback(SimpleNamespace(name=['neck_pan_joint'], position=[.2]))
        response = json.loads(await asyncio.wait_for(client.read_message(), 2))
        self.assertEqual(response['pose'], {'neck_pan_joint': .2})

    async def test_physics_input_rejects_stale_and_replayed_stamps(self):
        from sensor_msgs.msg import JointState
        client, response = await self.connect(receive=True, topic='/leader/joint_states', enable_fresh_input=True)
        self.assertEqual(response['type'], 'ready')
        def message(stamp, value):
            sample = JointState(name=['neck_pan_joint'], position=[value])
            sample.header.stamp.sec, sample.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            return sample
        now = time.time_ns()
        callback = self.subscriptions[0].callback
        callback(message(now-1_000_000_000, .8))
        callback(message(now+1_000_000_000, .7))
        callback(message(now, .2))
        callback(message(now, .9))
        response = json.loads(await asyncio.wait_for(client.read_message(), 2))
        self.assertEqual(response['pose'], {'neck_pan_joint': .2})
        self.assertAlmostEqual(response['stamp_sec'], now*1e-9, delta=1e-6)

    async def test_invalid_pose_rejected(self):
        client, _ = await self.connect()
        await client.write_message(json.dumps(dict(type='joints', pose={'unknown_joint': .1})))
        response = json.loads(await asyncio.wait_for(client.read_message(), 2))
        self.assertEqual(response['type'], 'error')
        self.assertFalse(self.published)

    async def test_self_subscription_rejected(self):
        _, response = await self.connect(receive=True, topic='/sim/joint_states')
        self.assertEqual(response['type'], 'error')
        self.assertFalse(self.subscriptions)

    async def test_physics_watchdog_without_browser_rendering(self):
        client = await websocket_connect(HTTPRequest(f'ws://127.0.0.1:{self.port}/physics', headers={'Origin': 'http://localhost:8877'}))
        self.clients.append(client)
        robot = dict(model='long', position=[0, 0, 0], quaternion=[0, 0, 0, 1], pose={},
                     enable_leader_follow=True, max_leader_age_sec=.15)
        await client.write_message(json.dumps(dict(type='start', bodies=[], robot=robot)))
        self.assertEqual(json.loads(await asyncio.wait_for(client.read_message(), 10))['type'], 'ready')
        await client.write_message(json.dumps(dict(type='poses', poses=[], joints={'R_joint1': .1}, leader_stamp_sec=time.time())))
        deadline = time.monotonic()+2.
        frame = {}
        while time.monotonic() < deadline:
            frame = json.loads(await asyncio.wait_for(client.read_message(), 1))
            if frame.get('is_leader_stopped'):
                break
        self.assertTrue(frame.get('is_leader_stopped'))
        await client.write_message(json.dumps(dict(type='poses', poses=[], joints={'R_joint1': .5}, leader_stamp_sec=time.time())))
        frame = json.loads(await asyncio.wait_for(client.read_message(), 1))
        self.assertTrue(frame['is_leader_stopped'])

    async def test_origin_rejected(self):
        with self.assertRaises(HTTPClientError):
            await websocket_connect(HTTPRequest(f'ws://127.0.0.1:{self.port}/joints', headers={'Origin': 'http://other.invalid'}))


if __name__ == '__main__':
    unittest.main()
