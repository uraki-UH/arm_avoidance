"""統合操作用UDP出力の明示許可・全関節照合・失効時停止。"""

from collections.abc import Sequence
import ipaddress
import math
import re
import socket

from joint_command_model import joint_command_model


def is_finite_number(value):
    """真偽値を除く有限数値の確認。"""
    try:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    except (OverflowError, TypeError):
        return False


class udp_output:
    """受信側の停止・watchdog確認を前提とするUDP指令ゲート。"""

    def __init__(self, urdf_path, config, allow_remote_udp=False, socket_factory=socket.socket):
        if not isinstance(config, dict) or not isinstance(allow_remote_udp, bool):
            raise ValueError('UDP設定または実機送信許可が不正です')
        self.model = joint_command_model(urdf_path)
        self.joint_names = config.get('joint_names')
        if (not isinstance(self.joint_names, list) or
                any(not isinstance(name, str) for name in self.joint_names) or
                len(self.joint_names) != 19 or len(set(self.joint_names)) != 19 or
                set(self.joint_names) != set(self.model.independent_names)):
            raise ValueError('UDP対応表には独立19関節の重複・欠落のない明示順序が必要です')
        self.joint_names = list(self.joint_names)
        self.scales = config.get('joint_scales')
        self.offsets_deg = config.get('joint_offsets_deg')
        if (not isinstance(self.scales, list) or not isinstance(self.offsets_deg, list) or
                len(self.scales) != 19 or len(self.offsets_deg) != 19 or
                any(not is_finite_number(value) or value == 0 for value in self.scales) or
                any(not is_finite_number(value) for value in self.offsets_deg)):
            raise ValueError('UDP校正値には19関節分の非零scaleと有限offsetが必要です')
        self.scales, self.offsets_deg = list(self.scales), list(self.offsets_deg)
        try:
            robot_ip = ipaddress.IPv4Address(config['robot_ip'])
            listen_ip = ipaddress.IPv4Address(config['listen_ip'])
            if not isinstance(config['robot_ip'], str) or not isinstance(config['listen_ip'], str):
                raise ValueError('数値IPv4文字列が必要です')
        except (KeyError, ipaddress.AddressValueError, TypeError, ValueError) as error:
            raise ValueError('robot_ip/listen_ipには数値IPv4文字列が必要です') from error
        if robot_ip.is_unspecified or robot_ip.is_multicast or str(robot_ip) == '255.255.255.255':
            raise ValueError('UDP送信先には単一受信機のIPv4アドレスが必要です')
        if listen_ip.is_multicast or str(listen_ip) == '255.255.255.255':
            raise ValueError('UDP待受アドレスが不正です')
        self.is_remote = not robot_ip.is_loopback
        if not self.is_remote and not listen_ip.is_loopback:
            raise ValueError('loopback試験の待受にはloopbackアドレスが必要です')
        for name in ('robot_port', 'listen_port', 'feedback_source_port'):
            value = config.get(name)
            if not isinstance(value, int) or isinstance(value, bool) or not 1 <= value <= 65535:
                raise ValueError('UDPポートの指定が不正です: '+name)
        self.destination = (str(robot_ip), config['robot_port'])
        self.feedback_source = (str(robot_ip), config['feedback_source_port'])
        self.enable_packet = self.packet(config.get('enable_packet', ''), 'enable_packet')
        self.stop_packet = self.packet(config.get('stop_packet', ''), 'stop_packet')
        if self.is_remote:
            if (not allow_remote_udp or
                    any(config.get(name) is not True for name in (
                        'is_mapping_verified', 'is_receiver_stop_verified',
                        'is_receiver_watchdog_verified')) or
                    not is_finite_number(config.get('receiver_watchdog_sec')) or
                    not 0 < config['receiver_watchdog_sec'] <= 1 or
                    not self.enable_packet or not self.stop_packet):
                raise ValueError('実機UDPには明示許可・校正・受信側停止・watchdog確認が必要です')
            if self.enable_packet == self.stop_packet:
                raise ValueError('受信側の有効化・停止パケットが同一です')
        for name, default, max_value in (
                ('max_command_age_sec', 0.5, 1.0), ('max_feedback_age_sec', 0.5, 1.0),
                ('max_start_dev_rad', 0.05, math.inf), ('max_tracking_dev_rad', 0.2, math.inf),
                ('max_joint_velocity', 0.3, math.inf), ('update_hz', 50.0, 100.0)):
            value = config.get(name, default)
            if not is_finite_number(value) or not 0 < value <= max_value:
                raise ValueError('UDP保護設定が不正です: '+name)
            setattr(self, name, value)
        self.max_send_age_sec = min(self.max_command_age_sec, self.max_feedback_age_sec,
                                    config['receiver_watchdog_sec'] if self.is_remote else 1.0)
        if 1/self.update_hz >= self.max_send_age_sec:
            raise ValueError('UDP送信周期に対して入力失効または受信側watchdogが短すぎます')
        self.is_enabled = False
        self.is_closed = False
        self.target = {}
        self.feedback = {}
        self.commanded = {}
        self.target_stamp_sec = None
        self.target_received_sec = None
        self.feedback_received_sec = None
        self.command_sent_sec = None
        self.num_sent = 0
        self.detail = '実機送信OFF'
        self.socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.setblocking(False)
            self.socket.bind((str(listen_ip), config['listen_port']))
        except Exception:
            self.socket.close()
            raise

    @staticmethod
    def packet(value, name):
        if not isinstance(value, str) or len(value) > 1024 or '\x00' in value:
            raise ValueError('UDP制御パケットが不正です: '+name)
        try:
            return value.encode('ascii')
        except UnicodeEncodeError as error:
            raise ValueError('UDP制御パケットにはASCII文字列が必要です: '+name) from error

    @staticmethod
    def is_fresh(received_sec, now, max_age_sec):
        return (is_finite_number(now) and received_sec is not None and
                0 <= now-received_sec <= max_age_sec)

    def validate_positions(self, values, max_rounding_dev_rad=0.0):
        if len(values) != 19 or any(not is_finite_number(value) for value in values):
            raise ValueError('UDP角度には19関節分の有限数値が必要です')
        for name, value in zip(self.joint_names, values):
            min_position, max_position, _ = self.model.bounds[name]
            if not min_position-max_rounding_dev_rad <= value <= max_position+max_rounding_dev_rad:
                raise ValueError('UDP角度がURDF可動範囲外です: '+name)

    def encode(self, values):
        positions = [values[name] for name in self.joint_names]
        self.validate_positions(positions)
        encoded = []
        for value, scale, offset in zip(positions, self.scales, self.offsets_deg):
            wire_value = (math.degrees(value)*scale+offset)*10
            if not math.isfinite(wire_value) or not -(2**31) <= wire_value <= 2**31-1:
                raise ValueError('UDP整数角度が32bit範囲外です')
            encoded.append(int(round(wire_value)))
        rounded = self.decode_values(encoded)
        return (','.join(map(str, encoded))+',').encode('ascii'), rounded

    def decode_values(self, encoded):
        values = [math.radians((value/10-offset)/scale)
                  for value, scale, offset in zip(encoded, self.scales, self.offsets_deg)]
        self.validate_positions(values, max_rounding_dev_rad=0.001)
        return dict(zip(self.joint_names, values))

    def decode(self, data):
        try:
            lines = data.decode('ascii').strip().splitlines()
            angles = [line[4:].strip() for line in lines if line.startswith('agl,')]
            if len(angles) != 1:
                raise ValueError('UDP実測値のagl行が欠落または重複しています')
            tokens = angles[0].removesuffix(',').split(',')
            if len(tokens) != 19 or any(re.fullmatch(r'[+-]?[0-9]{1,11}', item) is None for item in tokens):
                raise ValueError('UDP実測値の関節数または整数形式が不正です')
            encoded = [int(item) for item in tokens]
            if any(not -(2**31) <= value <= 2**31-1 for value in encoded):
                raise ValueError('UDP実測値が32bit範囲外です')
            return self.decode_values(encoded)
        except (UnicodeDecodeError, OverflowError, ZeroDivisionError) as error:
            raise ValueError('UDP実測値の符号化または校正結果が不正です') from error

    def fail(self, message):
        try:
            self.disable(message)
        except OSError as error:
            self.detail = message+' / 停止パケット送信失敗'
            raise ValueError(self.detail) from error
        raise ValueError(message)

    def update_target(self, names, positions, stamp_sec, now):
        try:
            if (self.is_closed or not is_finite_number(stamp_sec) or stamp_sec < 0 or
                    not is_finite_number(now) or not isinstance(names, Sequence) or
                    not isinstance(positions, Sequence) or isinstance(positions, (str, bytes)) or len(names) != 19 or
                    len(positions) != 19 or any(not isinstance(name, str) for name in names) or
                    len(set(names)) != 19 or set(names) != set(self.joint_names)):
                raise ValueError('UDP目標の名前・角度・時刻が不正です')
            values = dict(zip(names, positions))
            self.validate_positions([values[name] for name in self.joint_names])
            if self.target_stamp_sec is not None:
                if stamp_sec < self.target_stamp_sec:
                    raise ValueError('UDP目標時刻が逆行しています')
                if stamp_sec == self.target_stamp_sec:
                    return False
            if self.target_received_sec is not None and now < self.target_received_sec:
                raise ValueError('UDP目標受信時刻が逆行しています')
            self.encode(values)
        except (ValueError, TypeError, OverflowError) as error:
            self.fail(str(error))
        self.target = values
        self.target_stamp_sec, self.target_received_sec = stamp_sec, now
        return True

    def poll(self, now):
        if self.is_closed or not is_finite_number(now):
            self.fail('UDP受信の状態または時刻が不正です')
        if self.feedback_received_sec is not None and now < self.feedback_received_sec:
            self.feedback_received_sec = None
            self.fail('UDP実測値の受信時刻が逆行しています')
        for _ in range(32):
            try:
                data, source = self.socket.recvfrom(4096)
            except BlockingIOError:
                break
            except OSError as error:
                self.feedback_received_sec = None
                self.fail('UDP受信失敗: '+str(error))
            if source != self.feedback_source:
                continue
            try:
                values = self.decode(data)
            except ValueError as error:
                self.feedback.clear()
                self.feedback_received_sec = None
                if self.is_enabled:
                    self.fail(str(error))
                self.detail = str(error)
                break
            self.feedback = values
            self.feedback_received_sec = now

    def send(self, packet):
        if self.socket.sendto(packet, self.destination) != len(packet):
            raise OSError('UDP送信長が不一致です')
        self.num_sent += 1

    def enable(self, now):
        if self.is_enabled:
            return True
        if (self.is_closed or not self.is_fresh(self.target_received_sec, now, self.max_command_age_sec) or
                not self.is_fresh(self.feedback_received_sec, now, self.max_feedback_age_sec)):
            self.fail('UDP有効化には新しい全関節目標・実測値が必要です')
        if any(abs(self.target[name]-self.feedback[name]) > self.max_start_dev_rad
               for name in self.joint_names):
            self.fail('UDP有効化時のGazebo目標と実機実測値が離れています')
        self.commanded = dict(self.feedback)
        self.command_sent_sec = now
        # enableパケットの送信例外時も停止要求の対象とする内部状態
        self.is_enabled = True
        try:
            if self.enable_packet:
                self.send(self.enable_packet)
        except OSError as error:
            self.fail('UDP有効化パケット送信失敗: '+str(error))
        self.detail = 'UDP送信ON / 物理停止確認なし'
        return True

    def disable(self, reason):
        has_active_output = self.is_enabled
        self.is_enabled = False
        self.target.clear()
        self.target_received_sec = None
        self.commanded.clear()
        self.command_sent_sec = None
        self.detail = str(reason)
        if has_active_output and self.stop_packet:
            self.send(self.stop_packet)

    def tick(self, now):
        if not self.is_enabled:
            return False
        if not self.is_fresh(self.target_received_sec, now, self.max_command_age_sec):
            self.fail('UDP目標の受信失効です')
        if not self.is_fresh(self.feedback_received_sec, now, self.max_feedback_age_sec):
            self.fail('UDP実測値の受信失効です')
        duration_sec = now-self.command_sent_sec
        if not math.isfinite(duration_sec) or not 0 <= duration_sec <= self.max_send_age_sec:
            self.fail('UDP送信周期が不正です')
        if duration_sec+1e-12 < 1/self.update_hz:
            return False
        try:
            packet, positions = self.encode(self.target)
            for name in self.joint_names:
                max_step = min(self.max_joint_velocity, self.model.bounds[name][2])*duration_sec
                if abs(positions[name]-self.commanded[name]) > max_step+0.001:
                    raise ValueError('UDP指令の関節速度が設定範囲外です: '+name)
                if abs(positions[name]-self.feedback[name]) > self.max_tracking_dev_rad:
                    raise ValueError('UDP目標と実機実測値の追従偏差が設定範囲外です: '+name)
            self.send(packet)
        except (ValueError, OSError) as error:
            self.fail(str(error))
        self.commanded = positions
        self.command_sent_sec = now
        return True

    def status(self, now):
        return {
            'enable_hardware_output': self.is_enabled,
            'has_fresh_feedback': self.is_fresh(self.feedback_received_sec, now, self.max_feedback_age_sec),
            'has_fresh_target': self.is_fresh(self.target_received_sec, now, self.max_command_age_sec),
            'detail': self.detail, 'is_remote': self.is_remote, 'num_sent': self.num_sent,
            'is_physical_stop_confirmed': False,
        }

    def close(self):
        if self.is_closed:
            return
        try:
            self.disable('UDP終了 / 実機送信OFF')
        finally:
            self.is_closed = True
            self.socket.close()
