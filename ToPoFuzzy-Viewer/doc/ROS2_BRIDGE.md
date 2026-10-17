# ROS 2接続

ワークスペース全体のDocker起動は[RUN_GUIDE](../../RUN_GUIDE.md)。Viewer単体は以下。

```bash
cd ToPoFuzzy-Viewer/backend
source /opt/ros/humble/setup.bash
colcon build --packages-select topo_fuzzy_viewer --symlink-install
source install/setup.bash
ros2 launch topo_fuzzy_viewer viewer_stack.launch.py
```

設定・起動ノードは[viewer_stack.launch.py](../backend/src/topo_fuzzy_viewer/launch/viewer_stack.launch.py)。
必要なメッセージパッケージは事前ビルド・環境読込みが必要。

FrontendのTopicsで入力を選択。ROS_DOMAIN_ID・DDS設定は配信元と統一。
GNGの学習起動は[ワークスペース手順](../../RUN_GUIDE.md)、ロボット表示は[GNG-VLUT](../../gng_vlut_system/README.md)を参照。

通信契約は[WS v2](../common/ws_protocol_v2.md)、分割ノードの役割は[構成](ARCHITECTURE.md)。
