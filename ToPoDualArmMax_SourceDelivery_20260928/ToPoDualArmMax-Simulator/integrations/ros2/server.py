#!/usr/bin/env python3
"""Local-only read-only ROS -> TFV1 HTTP bridge. No ROS control interfaces."""
import argparse
import collections
import json
import mimetypes
import os
from pathlib import Path
import queue
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlsplit

import numpy as np
from compact_frame import read_snapshot_header, parse_snapshot
from protocol import CDR, FormatError, parse_graph, parse_markers, encode_frame

SERVICE = 'topo-fuzzy-viewer'
VERSION = '1.0.0'


class Store:
    def __init__(self):
        self.lock = threading.Lock()
        self.packet = None
        self.sequence = 0
        self.updated = None
        self.last_source = None
        self.last_error = None
        self.stats = collections.Counter()
        self.times = collections.deque(maxlen=300)

    def error(self, exc):
        with self.lock:
            self.stats['rejected_frames'] += 1
            self.last_error = str(exc)
        print('BRIDGE_REJECTED:', str(exc), flush=True)

    def status(self):
        with self.lock:
            age = (time.monotonic()-self.updated)*1000 if self.updated is not None else None
            samples = list(self.times)
            return dict(service=SERVICE, version=VERSION, instance_id=os.environ.get('TOPO_VIEWER_INSTANCE', ''),
                        pid=os.getpid(), frame_count=self.sequence, source_sequence=self.last_source,
                        frame_age_ms=age, healthy=age is not None and age < 2000,
                        stale=age is None or age >= 2000, full_range=True,
                        mode='READ_ONLY_VISUALIZATION', metrics_available=self.packet is not None,
                        packet_bytes=len(self.packet) if self.packet else 0,
                        processing_bridge_ms=dict(mean=float(np.mean(samples)), p95=float(np.percentile(samples,95)),
                                                  maximum=max(samples), samples=len(samples)) if samples else None,
                        stats=dict(self.stats), last_error=self.last_error,
                        endpoints=['/api/status','/api/frame?after=N','/api/latest'],
                        stale_threshold_ms=2000, topics=['/ais_gng/fvg_frame','/topological_map',
                        '/fvg_observer/add','/fvg_observer/delete','/fvg_observer/memory',
                        '/ais_gng_fvg/processing_metrics'])


class Joiner:
    """Bounded same-stamp, same-frame, source-sequence and arrival-window join."""
    def __init__(self, store):
        self.store = store
        self.cache = {k: collections.OrderedDict() for k in ('snapshot','map','add','delete','memory','metrics')}
        self.queue = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.last_snapshot_sequence = None
        self.worker = threading.Thread(target=self.work, daemon=True, name='tfv-encoder')
        self.worker.start()

    def ingest(self, kind, raw):
        now = time.monotonic_ns()
        try:
            if kind == 'snapshot':
                stamp, frame, seq, *_ = read_snapshot_header(raw)
                key = (stamp, frame, seq)
                if self.last_snapshot_sequence is not None:
                    gap = seq-self.last_snapshot_sequence-1
                    if gap < 0:
                        for cache in self.cache.values():
                            cache.clear()
                        self.store.stats['source_sequence_resets'] += 1
                    else:
                        self.store.stats['snapshot_sequence_gaps'] += max(0, gap)
                self.last_snapshot_sequence = seq
            elif kind == 'metrics':
                raw = json.loads(raw.data)
                seq = int(raw['frame_sequence'])
                frame = raw['fvg']['frame_id']
                if raw['ais']['frame_id'] != frame:
                    raise FormatError('Metric stages use different coordinate frames')
                key = (int(raw['stamp_ns']), frame, seq)
            elif kind == 'map':
                stamp, frame = CDR(raw).header()
                key = (stamp, frame)
            else:
                # MarkerArray starts with sequence length followed by first header.
                c = CDR(raw)
                if c.read('I',4) < 1:
                    raise FormatError('Empty FVG MarkerArray has no frame identity')
                key = c.header()
            self.store.stats[kind+'_received'] += 1
            self.cache[kind][key] = (raw, now)
            for cache in self.cache.values():
                for old, (_, arrival) in list(cache.items()):
                    if now-arrival > 1_500_000_000:
                        del cache[old]
                        self.store.stats['join_cache_evictions'] += 1
                while len(cache) > 24:
                    cache.popitem(last=False)
                    self.store.stats['join_cache_evictions'] += 1
            for skey in list(self.cache['snapshot']):
                short = skey[:2]
                lookup = {k: skey if k in ('snapshot','metrics') else short for k in self.cache}
                if not all(lookup[k] in self.cache[k] for k in self.cache):
                    continue
                pieces = {k:self.cache[k].pop(lookup[k]) for k in self.cache}
                arrivals = [p[1] for p in pieces.values()]
                if max(arrivals)-min(arrivals) > 1_000_000_000:
                    self.store.stats['arrival_guard_rejections'] += 1
                    continue
                if self.queue.full():
                    try:
                        self.queue.get_nowait()
                        self.store.stats['visualization_queue_drops'] += 1
                    except queue.Empty:
                        pass
                self.queue.put_nowait(pieces)
                self.store.stats['same_frame_joins'] += 1
        except (ValueError, KeyError, TypeError, IndexError, UnicodeError) as exc:
            self.store.error(exc)

    def work(self):
        while not self.stop.is_set():
            try:
                p = self.queue.get(timeout=.2)
            except queue.Empty:
                continue
            begin = time.perf_counter()
            now = time.monotonic_ns()
            if now-min(x[1] for x in p.values()) > 1_500_000_000:
                self.store.stats['stale_worker_drops'] += 1
                continue
            try:
                snapshot = parse_snapshot(p['snapshot'][0])
                graph = parse_graph(p['map'][0])
                fvg = {}
                for kind in ('add','delete','memory'):
                    stamp, frame, values = parse_markers(p[kind][0])
                    if (stamp, frame) != (snapshot.stamp_ns, snapshot.frame_id):
                        raise FormatError('FVG marker frame mismatch')
                    fvg[kind] = values
                metrics = p['metrics'][0]
                end = int(metrics['fvg']['worker_end_steady_ns'])
                if end > now+100_000_000 or now-end > 2_000_000_000:
                    raise FormatError('Processing metrics are stale or from a different execution')
                with self.store.lock:
                    seq = self.store.sequence+1
                packet = encode_frame(snapshot, graph, fvg, metrics, seq,
                    max(0., (now-end)/1e6), (max(x[1] for x in p.values())-min(x[1] for x in p.values()))/1e6)
                elapsed = (time.perf_counter()-begin)*1000
                with self.store.lock:
                    self.store.packet = packet
                    self.store.sequence = seq
                    self.store.updated = time.monotonic()
                    self.store.last_source = snapshot.frame_sequence
                    self.store.times.append(elapsed)
                    self.store.stats['encoded_frames'] += 1
            except Exception as exc:
                self.store.error(exc)


def make_handler(store, web_root, port):
    web_root = Path(web_root).resolve(strict=True)

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):
            if args and str(args[1] if len(args)>1 else '').startswith('4'):
                super().log_message(fmt, *args)

        def allowed(self):
            host = self.headers.get('Host', '')
            hosts = {'127.0.0.1:'+str(port), 'localhost:'+str(port)}
            if host not in hosts:
                return False
            origin = self.headers.get('Origin')
            if origin is not None and origin not in {'http://'+h for h in hosts}:
                return False
            return self.headers.get('Sec-Fetch-Site') not in ('cross-site',)

        def respond(self, status, data=b'', mime='application/octet-stream', head=False):
            self.send_response(status)
            self.send_header('Content-Type', mime)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Cross-Origin-Resource-Policy', 'same-origin')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.end_headers()
            if not head and data:
                try:
                    self.wfile.write(data)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        def do_HEAD(self):
            self.do_GET(head=True)

        def do_GET(self, head=False):
            if not self.allowed():
                return self.respond(403, b'Local same-origin requests only', 'text/plain', head)
            parsed = urlsplit(self.path)
            if parsed.path == '/api/status':
                return self.respond(200, json.dumps(store.status(), allow_nan=False).encode(), 'application/json', head)
            if parsed.path in ('/api/frame', '/api/latest'):
                try:
                    after = int(parse_qs(parsed.query).get('after', ['-1'])[0])
                except ValueError:
                    return self.respond(400, b'Invalid sequence', 'text/plain', head)
                with store.lock:
                    packet, seq = store.packet, store.sequence
                if packet is None or after == seq:
                    return self.respond(204, head=head)
                return self.respond(200, packet, head=head)
            try:
                path = (web_root/unquote(parsed.path).lstrip('/')).resolve()
                path.relative_to(web_root)
                if path.is_dir():
                    path = (path/'index.html').resolve()
                    path.relative_to(web_root)
                if not path.is_file():
                    return self.respond(404, b'Not found', 'text/plain', head)
                mime = mimetypes.guess_type(str(path))[0] or 'application/octet-stream'
                if path.suffix == '.js':
                    mime = 'text/javascript'
                return self.respond(200, path.read_bytes(), mime, head)
            except (ValueError, OSError):
                return self.respond(403, b'Invalid path', 'text/plain', head)

        def do_POST(self):
            self.close_connection = True
            return self.respond(405, b'Read-only service', 'text/plain')

        do_PUT = do_DELETE = do_PATCH = do_POST

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1', choices=['127.0.0.1'])
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--web-root', type=Path, required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '42':
        raise RuntimeError('Explicit ROS_DOMAIN_ID=42 is required; bridge does not control a ROS session')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from std_msgs.msg import String, ByteMultiArray
    from visualization_msgs.msg import MarkerArray
    from ais_gng_msgs.msg import TopologicalMap

    store = Store()
    joiner = Joiner(store)
    rclpy.init()
    node = Node('topo_fuzzy_viewer_read_only_bridge', start_parameter_services=False, enable_rosout=False)
    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=4, reliability=ReliabilityPolicy.BEST_EFFORT)
    subscriptions = [node.create_subscription(ByteMultiArray, '/ais_gng/fvg_frame', lambda m:joiner.ingest('snapshot',m),qos,raw=True),
                     node.create_subscription(TopologicalMap, '/topological_map', lambda m:joiner.ingest('map',m),qos,raw=True),
                     node.create_subscription(String, '/ais_gng_fvg/processing_metrics',lambda m:joiner.ingest('metrics',m),qos)]
    for kind in ('add','delete','memory'):
        subscriptions.append(node.create_subscription(MarkerArray,'/fvg_observer/'+kind,
                             lambda m,k=kind:joiner.ingest(k,m),qos,raw=True))
    httpd = ThreadingHTTPServer((args.host,args.port), make_handler(store,args.web_root,args.port))
    httpd.daemon_threads = True
    httpthread = threading.Thread(target=httpd.serve_forever,daemon=True,name='tfv-http')
    httpthread.start()
    print(json.dumps(dict(service=SERVICE,version=VERSION,pid=os.getpid(),url=f'http://{args.host}:{args.port}',mode='READ_ONLY_VISUALIZATION')),flush=True)
    def stop_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_signal)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        joiner.stop.set()
        httpd.shutdown(); httpd.server_close()
        joiner.worker.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
