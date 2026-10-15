"""ROSプラグインの登録・型検証・所有launch・ブラウザ接続の共通管理。"""
import copy
import importlib
import json
import math
from pathlib import Path
import re
import os
import signal
import subprocess
import tempfile
import threading
import time
import asyncio
from concurrent.futures import ThreadPoolExecutor


plugin_directory = Path(__file__).resolve().parent / 'plugins'
identifier_pattern = re.compile(r'[a-z][a-z0-9_]*\Z')


def definitions(directory=None):
    result = {}
    for path in sorted(Path(directory or plugin_directory).glob('*.json')):
        value = json.loads(path.read_text())
        name = value.get('id')
        if not isinstance(name, str) or not identifier_pattern.fullmatch(name) or name in result:
            raise ValueError('プラグインIDが不正または重複です')
        if value.get('version') != 1 or value.get('category') not in ('launch', 'physics'):
            raise ValueError('プラグインの形式・分類が不正です')
        if not isinstance(value.get('label'), str) or not isinstance(value.get('parameters', {}), dict):
            raise ValueError('プラグインの表示名・引数定義が不正です')
        for field, spec in value.get('parameters', {}).items():
            if not identifier_pattern.fullmatch(field) or spec.get('type') not in ('string', 'topic', 'bool', 'number', 'integer'):
                raise ValueError('プラグインの引数定義が不正です')
            if 'default' in spec:
                validate_value(field, spec['default'], spec)
        result[name] = value
    return result


def definition(plugin_id, directory=None):
    try:
        return definitions(directory)[plugin_id]
    except (KeyError, TypeError):
        raise ValueError('未登録のプラグインです') from None


def validate_value(name, value, spec):
    kind = spec['type']
    if kind == 'bool':
        has_valid_type = type(value) is bool
    elif kind in ('number', 'integer'):
        has_valid_type = type(value) in (int, float) and math.isfinite(value)
        if kind == 'integer':
            has_valid_type = has_valid_type and type(value) is int
        has_valid_type = has_valid_type and spec.get('min', -math.inf) <= value <= spec.get('max', math.inf)
    else:
        has_valid_type = isinstance(value, str) and len(value) <= spec.get('max_length', 2048) and '\x00' not in value and '\n' not in value and '\r' not in value
        if kind == 'topic':
            has_valid_type = has_valid_type and re.fullmatch(r'/[A-Za-z_][A-Za-z0-9_/]*', value) is not None and '//' not in value and not value.endswith('/')
    if not has_valid_type:
        raise ValueError('プラグイン引数の値が不正です: '+name)
    if 'choices' in spec and value not in spec['choices']:
        raise ValueError('プラグイン引数の選択肢が不正です: '+name)
    return value


def parameters(spec, values=None):
    fields = spec.get('parameters', {})
    if values is None:
        values = {}
    if not isinstance(values, dict) or set(values) - set(fields):
        raise ValueError('未登録のプラグイン引数です')
    result = {}
    for name, field in fields.items():
        if name in values:
            result[name] = validate_value(name, values[name], field)
        elif 'default' in field:
            result[name] = copy.deepcopy(field['default'])
        elif field.get('is_required'):
            raise ValueError('必須のプラグイン引数がありません: '+name)
    return result


def public_definition(value):
    return {key: copy.deepcopy(value[key]) for key in ('version', 'id', 'label', 'description', 'category', 'models', 'parameters', 'capabilities', 'settings_id', 'display', 'interfaces') if key in value}


def launch_command(spec, values=None, context=None):
    values = parameters(spec, values)
    launch = spec.get('launch')
    if not isinstance(launch, dict):
        raise ValueError('launch定義がありません')
    context = dict(context or {})
    command = ['ros2', 'launch', '--noninteractive']
    if 'package' in launch:
        package, file = launch['package'], launch['file']
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', package) or not re.fullmatch(r'[A-Za-z0-9_.-]+\.launch\.(py|xml|yaml)', file):
            raise ValueError('launchのpackage・ファイル名が不正です')
        command += [package, file]
    else:
        path = (Path(__file__).resolve().parent / launch['path']).resolve()
        if not path.is_relative_to(Path(__file__).resolve().parent / 'plugins') or not path.is_file():
            raise ValueError('登録launchのパスが不正です')
        command.append(str(path))
    for name, binding in launch.get('arguments', {}).items():
        if not identifier_pattern.fullmatch(name):
            raise ValueError('launch引数名が不正です')
        if isinstance(binding, dict):
            if 'parameter' in binding:
                value = values[binding['parameter']]
            elif 'context' in binding:
                value = context[binding['context']]
            else:
                raise ValueError('launch引数の対応付けが不正です')
        else:
            value = binding
        value = str(value).lower() if type(value) is bool else str(value)
        if any(char in value for char in ('\x00', '\n', '\r')):
            raise ValueError('launch引数の文字列が不正です')
        command.append(name+':='+value)
    return command


def physics_adapter(settings, robot):
    settings = dict(settings or {})
    plugin_id = settings.pop('mode', 'none')
    if plugin_id == 'none':
        return None, {}, None
    spec = definition(plugin_id)
    if spec['category'] != 'physics' or not robot or robot['model'] not in spec.get('models', [robot['model']]):
        raise ValueError('プラグインの対象モデル・分類が不一致です')
    values = parameters(spec, settings)
    try:
        module, name = spec['adapter'].split(':')
        adapter = getattr(importlib.import_module(module), name)()
    except (ImportError, AttributeError, KeyError) as error:
        raise ValueError('登録した物理アダプタが見つかりません') from error
    return adapter, dict(mode=plugin_id, **values), adapter.prepare(values, robot)


class launch_runtime:
    def __init__(self, spec, values=None, context=None, log=None):
        self.spec, self.context = spec, dict(context or {})
        self.command = launch_command(spec, values, self.context)
        self.process = None
        self.cancel_event = threading.Event()
        self.directory = None
        self.log = log
        self.has_owned_log = log is None
        self.phase = 'idle'
        self.began_sec = None
        self.last_ready_check_sec = -float('inf')
        self.missing_ready_sec = None
        self.is_closed = False
        self.lock = threading.RLock()

    def start(self):
        with self.lock:
            if self.process is not None or self.cancel_event.is_set():
                raise RuntimeError('launchは開始済みまたは停止済みです')
            if self.has_owned_log:
                self.directory = tempfile.TemporaryDirectory(prefix='topo-launch-')
                self.log = open(Path(self.directory.name)/'launch.log', 'w+')
            try:
                self.process = subprocess.Popen(self.command, stdin=subprocess.DEVNULL, stdout=self.log,
                                                stderr=subprocess.STDOUT, start_new_session=True)
                self.began_sec = time.monotonic()
                self.phase = 'starting'
            except Exception:
                self.close()
                raise
        return self

    def poll(self, node=None):
        if self.is_closed:
            return self.phase
        if self.process is None:
            return self.phase
        code = self.process.poll()
        if code is not None:
            self.phase = 'exited' if code == 0 else 'failed'
            return self.phase
        now_sec = time.monotonic()
        if now_sec-self.last_ready_check_sec < .5:
            return self.phase
        self.last_ready_check_sec = now_sec
        ready = self.spec.get('launch', {}).get('ready', {})
        has_ready = self.is_ready(node)
        if has_ready:
            self.phase = 'running'
            self.missing_ready_sec = None
        elif self.phase == 'running':
            if self.missing_ready_sec is None:
                self.missing_ready_sec = now_sec
            if now_sec-self.missing_ready_sec > 2:
                self.phase = 'failed'
                raise RuntimeError('launchのROS出力が失効しました')
        elif now_sec-self.began_sec > ready.get('max_wait_sec', 20):
            self.phase = 'failed'
            raise RuntimeError('launchの準備が時間切れです')
        return self.phase

    def is_ready(self, node=None):
        ready = self.spec.get('launch', {}).get('ready', {})
        if not ready:
            return True
        if node is None:
            return False
        name = ready['name'].format_map(self.context)
        message_type = ready.get('message_type')
        if ready['kind'] == 'service':
            return any(service == name and (not message_type or message_type in types)
                       for service, types in node.get_service_names_and_types())
        if ready['kind'] == 'topic':
            return node.count_publishers(name) > 0 and (not message_type or any(
                topic == name and message_type in types for topic, types in node.get_topic_names_and_types()))
        raise ValueError('未対応のlaunch準備判定です')

    def wait_ready(self, node=None):
        while not self.cancel_event.is_set():
            phase = self.poll(node)
            if phase == 'running':
                return
            if phase in ('failed', 'exited'):
                raise RuntimeError('launchが準備完了前に終了しました')
            self.cancel_event.wait(.1)
        raise RuntimeError('launchの開始を取消しました')

    def snapshot(self, node=None, enable_logs=False):
        error = None
        try:
            phase = self.poll(node)
        except RuntimeError as failure:
            phase, error = self.phase, str(failure)
        result = dict(plugin_id=self.spec['id'], phase=phase, pid=self.process.pid if self.process else None,
                      exit_code=self.process.poll() if self.process else None)
        if error:
            result['error'] = error
        if enable_logs and self.log is not None:
            self.log.flush()
            with open(self.log.name, 'rb') as stream:
                stream.seek(0, 2)
                stream.seek(max(0, stream.tell()-8192))
                result['log_tail'] = stream.read(8192).decode('utf-8', errors='replace')
        return result

    def cancel(self):
        self.cancel_event.set()

    def close(self):
        self.cancel()
        with self.lock:
            if self.is_closed:
                return
            self.is_closed = True
            try:
                if self.process is not None:
                    # 新規セッションで開始したプロセスグループだけを対象とした停止
                    for sig, max_wait_sec in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
                        try:
                            os.killpg(self.process.pid, sig)
                        except ProcessLookupError:
                            break
                        until_sec = time.monotonic()+max_wait_sec
                        while time.monotonic() < until_sec:
                            self.process.poll()
                            try:
                                os.killpg(self.process.pid, 0)
                            except ProcessLookupError:
                                break
                            time.sleep(.05)
                        else:
                            continue
                        break
                    self.process.wait(timeout=2)
            finally:
                self.phase = 'stopped'
                if self.has_owned_log and self.log is not None:
                    self.log.close()
                    self.log = None
                if self.directory is not None:
                    self.directory.cleanup()


lease_lock = threading.Lock()
active_leases = set()


class plugin_session:
    def __init__(self, node):
        self.node = node
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='topo-plugin')
        self.runtime = None
        self.lease = None
        self.is_closed = False
        self.last_log = ''
        self.terminal_phase = None

    def start(self, value):
        if not isinstance(value, dict) or set(value)-{'type', 'plugin_id', 'parameters'}:
            raise ValueError('未登録のプラグイン操作項目です')
        if self.is_closed or self.runtime is not None and not self.runtime.is_closed:
            raise ValueError('プラグインは開始済みまたは切断済みです')
        spec = definition(value['plugin_id'])
        if spec['category'] != 'launch':
            raise ValueError('物理プラグインは物理セッションから開始してください')
        values = parameters(spec, value.get('parameters'))
        self.lease = (spec['id'], values.get('namespace', 'singleton'))
        with lease_lock:
            if self.lease in active_leases:
                self.lease = None
                raise ValueError('同じ名前空間のプラグインが既に実行中です')
            active_leases.add(self.lease)
        try:
            self.terminal_phase = None
            self.last_log = ''
            self.runtime = launch_runtime(spec, values, context=values)
            if self.is_closed:
                raise RuntimeError('接続切断のため開始を取消しました')
            if spec.get('launch', {}).get('ready') and self.runtime.is_ready(self.node):
                raise ValueError('指定した名前空間に準備済みのROS出力が既に存在します')
            self.runtime.start()
            self.runtime.wait_ready(self.node)
            return self.snapshot()
        except Exception:
            self.stop()
            raise

    def snapshot(self):
        value = self.runtime.snapshot(self.node, enable_logs=True) if self.runtime else dict(phase='idle')
        if self.runtime and self.runtime.is_closed and self.terminal_phase:
            value['phase'] = self.terminal_phase
        if not value.get('log_tail'):
            value['log_tail'] = self.last_log
        return dict(type='status', **value)

    def stop(self, terminal_phase='stopped'):
        try:
            if self.runtime is not None:
                try:
                    self.last_log = self.runtime.snapshot(enable_logs=True).get('log_tail') or self.last_log
                except (RuntimeError, ValueError):
                    pass
                self.runtime.close()
                self.terminal_phase = terminal_phase
        finally:
            if self.lease is not None:
                with lease_lock:
                    active_leases.discard(self.lease)
                self.lease = None
        return self.snapshot()

    def close(self):
        if self.is_closed:
            return None
        self.is_closed = True
        if self.runtime:
            self.runtime.cancel()
        future = self.executor.submit(self.stop)
        self.executor.shutdown(wait=False)
        return future


def plugin_handler(origins, clients, node, on_release):
    import tornado.ioloop
    import tornado.websocket

    class Handler(tornado.websocket.WebSocketHandler):
        def check_origin(self, origin):
            return origin in origins

        def open(self):
            self.session = plugin_session(node)
            clients.add(self)
            self.timer = tornado.ioloop.PeriodicCallback(self.status, 1000)
            self.timer.start()
            self.write_message(dict(type='status', phase='idle'))

        async def on_message(self, raw):
            try:
                if len(raw) > 16384:
                    raise ValueError('プラグイン要求が大きすぎます')
                value = json.loads(raw)
                if not isinstance(value, dict):
                    raise ValueError('プラグイン要求の形式不正')
                if value.get('type') == 'start':
                    self.write_message(dict(type='status', phase='starting', plugin_id=value.get('plugin_id')))
                    result = await asyncio.wrap_future(self.session.executor.submit(self.session.start, value))
                elif value.get('type') == 'stop':
                    if self.session.runtime:
                        self.session.runtime.cancel()
                    result = await asyncio.wrap_future(self.session.executor.submit(self.session.stop))
                else:
                    raise ValueError('未対応のプラグイン操作です')
                if not self.ws_connection.is_closing():
                    self.write_message(result)
            except tornado.websocket.WebSocketClosedError:
                pass
            except Exception as error:
                if self.ws_connection and not self.ws_connection.is_closing():
                    snapshot = await asyncio.wrap_future(self.session.executor.submit(self.session.snapshot))
                    self.write_message(dict(type='error', error=str(error), log_tail=snapshot.get('log_tail', '')))

        async def status(self):
            if self.ws_connection is None or self.session.is_closed:
                return
            try:
                value = await asyncio.wrap_future(self.session.executor.submit(self.session.snapshot))
                if value.get('phase') in ('failed', 'exited') and self.session.runtime and not self.session.runtime.is_closed:
                    value = await asyncio.wrap_future(self.session.executor.submit(self.session.stop, value['phase']))
                await self.write_message(value)
            except tornado.websocket.WebSocketClosedError:
                pass
            except Exception as error:
                self.write_message(dict(type='error', error=str(error)))
                await asyncio.wrap_future(self.session.executor.submit(self.session.stop, 'failed'))

        def on_close(self):
            self.timer.stop()
            future = self.session.close()
            if future is not None:
                on_release(future)
            clients.discard(self)
    return Handler
