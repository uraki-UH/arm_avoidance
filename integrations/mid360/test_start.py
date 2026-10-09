"""ホスト側自動設定の安全条件。実ネットワーク変更なし。"""

import ipaddress
import subprocess
import unittest
from unittest.mock import patch

import start


def address(name, value):
    local = ipaddress.IPv4Interface(value)
    return {"ifname": name, "addr_info": [
        {"family": "inet", "local": str(local.ip), "prefixlen": local.network.prefixlen}]}


class StartTest(unittest.TestCase):
    def setUp(self):
        self.host = start.network_config({"host_ip": "192.168.1.5", "lidar_ip": "192.168.1.141"})
        self.devices = {"test_lan": True, "unplugged": False}
        self.addresses = [address("wifi", "192.168.11.25/24")]
        self.routes = [{"dst": "default", "dev": "wifi"}]

    def select(self, interface=None):
        return start.select_interface(self.host, self.devices, self.addresses, self.routes, interface)

    def test_single_lan(self):
        self.assertEqual(self.select(), ("test_lan", False))

    def test_existing_address(self):
        self.addresses.append(address("test_lan", str(self.host)))
        self.assertEqual(self.select(), ("test_lan", True))

    def test_no_link(self):
        self.devices["test_lan"] = False
        with self.assertRaises(ValueError):
            self.select()

    def test_multiple_lans_require_selection(self):
        self.devices["second_lan"] = True
        with self.assertRaises(ValueError):
            self.select()
        self.assertEqual(self.select("test_lan"), ("test_lan", False))
        with self.assertRaises(ValueError):
            self.select("wifi")

    def test_existing_other_ip(self):
        self.addresses.append(address("test_lan", "10.0.0.2/24"))
        with self.assertRaises(ValueError):
            self.select()

    def test_subnet_conflict(self):
        self.addresses.append(address("other_lan", "192.168.1.99/24"))
        with self.assertRaises(ValueError):
            self.select()

    def test_default_route_protection(self):
        self.routes.append({"dst": "default", "dev": "test_lan"})
        with self.assertRaises(ValueError):
            self.select()

    def test_route_conflict(self):
        self.routes.append({"dst": "192.168.1.0/24", "dev": "vpn"})
        with self.assertRaises(ValueError):
            self.select()

    def test_global_ipv6_protection(self):
        self.addresses.append({"ifname": "test_lan", "addr_info": [
            {"family": "inet6", "scope": "global"}]})
        with self.assertRaises(ValueError):
            self.select()

    def test_invalid_ips(self):
        for host, lidar in [("", "192.168.1.141"), ("192.168.1.5", "192.168.2.141"),
                            ("192.168.1.5", "192.168.1.5"), ("192.168.1.5", "192.168.1.255"),
                            ("127.0.0.5", "127.0.0.6")]:
            with self.subTest(host=host, lidar=lidar), self.assertRaises(ValueError):
                start.network_config({"host_ip": host, "lidar_ip": lidar})

    def test_create_profile(self):
        with patch.object(start, "output", return_value="old-uuid:Wired connection 1"):
            commands = start.setup_commands("test_lan", self.host)
        self.assertEqual(len(commands), 2)
        self.assertIn("192.168.1.5/24", commands[0])
        self.assertNotIn("modify", commands[0])

    def test_reuse_profile(self):
        with patch.object(start, "output", side_effect=[
            "test-uuid:mid360-direct", "test_lan", "802-3-ethernet", "manual",
            "192.168.1.5/24", "", "yes"]):
            self.assertEqual(start.setup_commands("test_lan", self.host), [
                ["sudo", "nmcli", "connection", "up", "uuid", "test-uuid"]])

    def test_do_not_overwrite_profile(self):
        with patch.object(start, "output", side_effect=["test-uuid:mid360-direct", "other_lan"]):
            with self.assertRaises(ValueError):
                start.setup_commands("test_lan", self.host)

    def test_check_mode_has_no_writes(self):
        with (patch("sys.argv", ["start.py", "--check"]),
              patch.object(start.shutil, "which", return_value="/mock/command"),
              patch.object(start.Path, "read_text", side_effect=[
                  'host_ip: "192.168.1.5"\nlidar_ip: "192.168.1.141"', "1"]),
              patch.object(start, "output", side_effect=["test_lan:ethernet", "[]", "[]"]),
              patch.object(start, "setup_commands", return_value=[["sudo", "nmcli"]]),
              patch.object(start.subprocess, "run") as run,
              patch.object(start.os, "execvp") as execute):
            start.main()
            run.assert_not_called()
            execute.assert_not_called()

    def test_setup_failure_does_not_start_docker(self):
        with (patch("sys.argv", ["start.py"]),
              patch.object(start.shutil, "which", return_value="/mock/command"),
              patch.object(start.Path, "read_text", side_effect=[
                  'host_ip: "192.168.1.5"\nlidar_ip: "192.168.1.141"', "1"]),
              patch.object(start, "output", side_effect=["test_lan:ethernet", "[]", "[]"]),
              patch.object(start, "setup_commands", return_value=[["sudo", "nmcli"]]),
              patch.object(start.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "nmcli")),
              patch.object(start.os, "execvp") as execute):
            with self.assertRaises(subprocess.CalledProcessError):
                start.main()
            execute.assert_not_called()

    def test_duplicate_profile(self):
        with patch.object(start, "output", return_value="one:mid360-direct\ntwo:mid360-direct"):
            with self.assertRaises(ValueError):
                start.setup_commands("test_lan", self.host)


if __name__ == "__main__":
    unittest.main()
