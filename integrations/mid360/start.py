#!/usr/bin/env python3
"""MID-360直結LANの固定IPv4設定とCompose起動。"""

import argparse
import ipaddress
import json
import os
from pathlib import Path
import shutil
import subprocess

import yaml


root = Path(__file__).resolve().parents[2]
profile_name = "mid360-direct"


def output(*command):
    return subprocess.check_output(command, text=True).strip()


def network_config(config):
    # MID-360既定ネットマスクでの直結接続。異なるサブネットは手動設定対象
    host = ipaddress.IPv4Interface(f"{config['host_ip']}/24")
    lidar = ipaddress.IPv4Address(config["lidar_ip"])
    for addr in (host.ip, lidar):
        if (addr.is_loopback or addr.is_multicast or addr.is_unspecified
                or addr in (host.network.network_address, host.network.broadcast_address)):
            raise ValueError("host_ip・lidar_ipには有効な実機用IPv4が必要です")
    if host.ip == lidar or lidar not in host.network:
        raise ValueError("PCとLiDARには同じ/24サブネット内の異なるIPが必要です")
    return host


def select_interface(host, devices, addresses, routes, interface):
    candidates = [name for name, has_carrier in devices.items() if has_carrier]
    if interface:
        if interface not in candidates:
            raise ValueError("指定LANは物理リンクのあるethernetではありません")
    elif len(candidates) == 1:
        interface = candidates[0]
    else:
        raise ValueError(f"接続LANを一意に選択できません: {candidates}。配線確認、または--interfaceで指定してください")
    has_host_ip = False
    for device in addresses:
        for addr in device.get("addr_info", []):
            if (device["ifname"] == interface and addr.get("family") == "inet6"
                    and addr.get("scope") == "global"):
                raise ValueError("グローバルIPv6を使用中のLANは自動変更しません")
            if addr.get("family") != "inet":
                continue
            local = ipaddress.IPv4Interface(f"{addr['local']}/{addr['prefixlen']}")
            if device["ifname"] == interface:
                if local != host:
                    raise ValueError(f"{interface}に別のIPv4設定があります: {local}。既存設定は変更しません")
                has_host_ip = True
            elif local.network.overlaps(host.network):
                raise ValueError(f"別LANとサブネットが競合しています: {device['ifname']} {local}")
    for route in routes:
        dest = route.get("dst", "default")
        if dest == "default":
            if route.get("dev") == interface:
                raise ValueError("デフォルト経路に使用中のLANは自動変更しません")
        elif route.get("dev") != interface and host.network.overlaps(ipaddress.IPv4Network(dest, strict=False)):
            raise ValueError(f"別経路とサブネットが競合しています: {dest}")
    return interface, has_host_ip


def setup_commands(interface, host):
    matches = [row.split(":", 1)[0] for row in output(
        "nmcli", "-t", "-f", "UUID,NAME", "connection", "show").splitlines()
        if row.split(":", 1)[-1] == profile_name]
    if len(matches) > 1:
        raise ValueError("mid360-directが重複しています。自動変更は中止します")
    if matches:
        uuid = matches[0]
        expected = {
            "connection.interface-name": interface, "connection.type": "802-3-ethernet",
            "ipv4.method": "manual", "ipv4.addresses": str(host),
            "ipv4.gateway": "", "ipv4.never-default": "yes",
        }
        for key, value in expected.items():
            if output("nmcli", "-g", key, "connection", "show", "uuid", uuid) != value:
                raise ValueError(f"既存mid360-directの{key}が不一致。上書きは行いません")
        return [["sudo", "nmcli", "connection", "up", "uuid", uuid]]
    return [
        ["sudo", "nmcli", "connection", "add", "type", "ethernet",
         "con-name", profile_name, "ifname", interface,
         "ipv4.method", "manual", "ipv4.addresses", str(host),
         "ipv4.never-default", "yes", "ipv6.method", "disabled",
         "connection.autoconnect", "no"],
        ["sudo", "nmcli", "connection", "up", profile_name],
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", help="複数の有線LANが接続中の場合の対象DEVICE")
    parser.add_argument("--check", dest="enable_check", action="store_true", help="選択・設定内容の確認のみ。変更・起動なし")
    args = parser.parse_args()
    for command in ("ip", "nmcli", "docker", "sudo"):
        if not shutil.which(command):
            raise ValueError(f"ホストPCに{command}が必要です")
    config = yaml.safe_load((root / "integrations/mid360/mid360.yaml").read_text())
    host = network_config(config)
    devices = {}
    for row in output("nmcli", "-t", "-f", "DEVICE,TYPE", "device", "status").splitlines():
        name, kind = row.rsplit(":", 1)
        if kind == "ethernet":
            try:
                devices[name] = Path(f"/sys/class/net/{name}/carrier").read_text().strip() == "1"
            except OSError:
                devices[name] = False
    interface, has_host_ip = select_interface(
        host, devices, json.loads(output("ip", "-j", "address", "show")),
        json.loads(output("ip", "-j", "-4", "route", "show", "table", "all")), args.interface)
    commands = [] if has_host_ip else setup_commands(interface, host)
    print(f"MID-360: LAN={interface} | PC={host} | LiDAR={config['lidar_ip']} | "
          f"IP設定={'既存を使用' if has_host_ip else '専用プロファイルを有効化'}", flush=True)
    if args.enable_check:
        print("確認のみ: ネットワーク変更・Docker起動なし。センサ識別・IP重複の実機確認は対象外")
        return
    for command in commands:
        subprocess.run(command, check=True)
    # 実際のIP割当確認後のCompose起動。sudoの対象はNetworkManager操作のみ
    _, has_host_ip = select_interface(host, devices,
        json.loads(output("ip", "-j", "address", "show")),
        json.loads(output("ip", "-j", "-4", "route", "show", "table", "all")), interface)
    if not has_host_ip:
        raise ValueError("固定IPv4が反映されていません。Docker起動は中止します")
    print("Ctrl+C: ドライバ停止。専用LANのIP設定は維持", flush=True)
    os.chdir(root)
    os.execvp("docker", ["docker", "compose", "up", "mid360"])


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError, subprocess.CalledProcessError) as error:
        raise SystemExit(f"MID-360起動中止: {error}")
    except KeyboardInterrupt:
        raise SystemExit(130)
