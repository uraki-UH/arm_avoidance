# AiS-GNG

## 立ち上げ方

```
docker compose up -d
```

## docker 入り方
```
docker compose exec gng_cpu bash
```

## ais_gng を Docker で起動
`docker-compose.yaml` でこのリポジトリを `/ros2_ws/src/ais_gng` に、必要な依存パッケージを `/ros2_ws/src` にマウントする設定になっています。

```bash
docker compose exec gng_cpu bash
cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to ais_gng
source /ros2_ws/install/setup.bash
ros2 launch ais_gng ais_gng.launch.py
```

`colcon` の引数は `--packages-up-to` です。`--package-up-to` ではありません。

必要なら、用途に応じて次の launch も使えます。

- `ros2 launch ais_gng fixed_type_mid360.launch.py`
- `ros2 launch ais_gng fixed_type_avia.launch.py`
- `ros2 launch ais_gng mobile_type.launch.py`
- `ros2 launch ais_gng sampling.launch.py`
- `ros2 launch ais_gng rviz.launch.py`

## `not found: /ros2_ws/install/.../local_setup.bash` が出るとき
古い `build/` や `install/` が残っていると、前のワークスペースの設定を読みにいって失敗することがあります。

```bash
docker compose down -v
rm -rf build install log
docker compose up -d --build
```

そのあと、上の `colcon build` と `source /ros2_ws/install/setup.bash` をやり直してください。

## `fuzzy_classifier` について
`ais_gng` の launch は `/fuzzy_classifier/cluster_summary` などのトピック名を使いますが、これは出力先のトピック名です。`fuzzy_classifier` パッケージを起動する手順ではありません。

## 鍵の作り方
```
mkdir -p ./key/cpu
openssl ecparam -name prime256v1 -genkey -out ./key/cpu/binary_sign.pem
openssl ec -in ./key/cpu/binary_sign.pem -pubout -out ./key/cpu/binary_sign_pub.pem
openssl ec -pubin -inform PEM -in ./key/cpu/binary_sign_pub.pem -outform DER -out ./key/cpu/binary_sign_pub.der
```
