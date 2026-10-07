"""関節WebSocketの境界値・送受信・切断時の購読解除検証。"""
import asyncio
import json
import socket
import unittest
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

    async def test_origin_rejected(self):
        with self.assertRaises(HTTPClientError):
            await websocket_connect(HTTPRequest(f'ws://127.0.0.1:{self.port}/joints', headers={'Origin': 'http://other.invalid'}))


if __name__ == '__main__':
    unittest.main()
