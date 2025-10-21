dynamixel_workbench_controllers
==================================================

## 概要
本パッケージは，[Dynamixelサーボモーター](https://www.dynamixel.com/index.php)をROSから駆動するためのコントローラであるオリジナルの[dynamixel_workbench_controllers](https://wiki.ros.org/dynamixel_workbench_controllers)を，ROS2に対応するよう修正したものです．

## 背景
ROS1では，ROBOTIS社から[dynamixel_workbench](https://github.com/ROBOTIS-GIT/dynamixel-workbench/tree/noetic-devel)パッケージが提供されており，この中に`dynamixel_workbench_controllers`が含まれていました．これは，[dynamixel_workbench_msgs](https://github.com/ROBOTIS-GIT/dynamixel-workbench-msgs)に定義されたサービスやメッセージを介して，ROSから`Dynamixelサーボモーター`を制御するためのものです．

ところが，ROS2では`dynamixel_workbench_msgs`は引き続き提供されているものの，`dynamixel_workbench_controllers`は終息してしまいました．そこで，その代替として，本パッケージは同様の機能を持つROS2ノードを提供します．

なお，[ros2_control](https://control.ros.org/jazzy/doc/ros2_control/doc/index.html)に準拠した`Dynamixel`制御パッケージとして[dynamixel_hardware](https://index.ros.org/p/dynamixel_hardware/)がROS2公式リポジトリから配布されています．ハードウェアの制御を`ros2_control`対応にしておけば既存の`ros2_control`準拠の[様々なコントローラ](https://control.ros.org/rolling/doc/ros2_controllers/doc/controllers_index.html)を利用できるようになりますので，移動台車や多関節アームを制御する場合はこちらを検討する方が良いかもしれません．

## インストール
本パッケージは，[Jazzy](https://docs.ros.org/en/jazzy/index.html) distributionで動作確認しています．

予め，次の2つの依存パッケージをインストールしておいてください．
```bash
$ sudo apt install ros-jazzy-dynamixel-workbench-toolbox ros-jazzy-dynamixel-workbench-msgs
```
そして，`github`から`dynamixel_workbench_controllers`を入手し，`devel-aist`ブランチを取り出します．
```bash
$ cd catkin_ws/src
$ git clone git@github.com:Automation-Research-Team/dynamixel_workbench_controllers.git
$ cd dynamixel_workbench_controllers
$ git checkout devel-aist
```
最後に，ワークスペース全体をコンパイルしてください．
```bash
$ cd catkin_ws
$ colcon build
```

## dynamixel_workbench_controllersノード
### ノードの機能
`dynamixel_workbench_controllers`ノードは，USB=シリアル変換インタフェース[U2D2](https://emanual.robotis.com/docs/en/parts/interface/u2d2/)のコントローラであり，それに接続されている1台以上の`Dynamixel`サーボモーターを制御します．主に，次の3つの機能があります．
- **~/dynamixel_command** サービス（[dynamixel_workbench_msgs/srv/DynamixelCommand](http://docs.ros.org/en/noetic/api/dynamixel_workbench_msgs/html/srv/DynamixelCommand.html)型）を介してクライアントからのリクエストを受信し，制御命令を指定されたモーターに伝えてハードウェアを動かす
- **~/joint_trajectory** トピック（[trajectory_msgs/msg/JointTrajectory](http://docs.ros.org/en/jazzy/p/trajectory_msgs/msg/JointTrajectory.html)型）から関節角空間における軌道を受信し，それをモーターに転送してハードウェアを動かす．多関節アームやパン=チルト雲台の制御を想定
- **~/cmd_vel** トピック（[geometry_msgs/msg/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)型）から2次元平面上の直交座標空間における並進・回転速度を受信し，それを左右の車輪の回転速度に変換しモーターに転送してハードウェアを動かす．差動二輪型移動台車の制御を想定

また，サービス，トピック，パラメータ等の名前は，[オリジナルのdynamixel_workbench_controllers](https://wiki.ros.org/dynamixel_workbench_controllers)のそれに一致させています．

### ROSサービス
- **~/dynamixel_command**（[dynamixel_workbench_msgs/srv/DynamixelCommand](http://docs.ros.org/en/noetic/api/dynamixel_workbench_msgs/html/srv/DynamixelCommand.html)型）：制御コマンドを指定されたIDを持つモーターに送る

コマンドのリクエストは，次の4フィールドから成ります．
- **command** (type: string): 通常使われないので，空文字列でOK
- **id** (type: uint8): コマンドの送り先となるモーターのID
- **address** (type: string): コマンド名．[Dynamixel機種一覧](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/#supported-dynamixel)にある個々のモーターの`Control Table`の`Data Name`の欄にある名前を記述
- **value** (type: int32): コマンドで与える指令値

### ROSトピック
- **~/joint_trajectory**（[trajectory_msgs/msg/JointTrajectory](http://docs.ros.org/en/jazzy/p/trajectory_msgs/msg/JointTrajectory.html)型）：MoveIt等によって生成された関節角空間における軌道をsubscribe
- **~/cmd_vel**（[geometry_msgs/msg/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)型）：JoyStick等によって生成された2次元直交座標空間における並進・回転速度をsubscribe
- **~/dynamixel_state**（[dynamixel_msgs/msg/DynamixelStateList](http://docs.ros.org/en/noetic/api/dynamixel_workbench_msgs/html/msg/DynamixelStateList.html)型）：`U2D2`に接続されている全てのモーターの状態をpublish
- **joint_states**（[sensor_msgs/msg/JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)型）：`U2D2`に接続されている全てのモーターの回転角，回転速度およびトルクをpublish

### ノードパラメータ
- **usb_port** (type: string): `U2D2`を接続するPCのUSBポート名 (default: `/dev/ttyUSB0`)
- **dxl_baud_rate** (type: int64): `U2D2`とモーターを接続するシリアルラインの通信速度 (default: `1000000` baud)
- **dxl_read_period** (type: float): `~/dynamixel_state`トピックに出力されるモーターの状態を取得する時間間隔 (default: `0.01` sec)
- **dxl_write_period** (type: float): `~/joint_trajectory`トピックから入力された関節角軌道をモーターに送信する時間間隔 (default: `0.01` sec)
- **use_joint_states_topic** (type: bool): `true`ならば`joint_states`トピックをpublishする (default: `false`)
- **use_moveit** (type: bool): `true`ならば`~/joint_trajectory`トピックでsubscribeした関節角軌道上の通過点(waypoint)をそのままモータに送る．waypointは`dxl_write_period`で指定された間隔で次々に送られるので，waypointが疎な場合，モーターは非常に高速に動くことになる．これに対し，`false`ならば，subscribeされた軌道のwaypoint間をさらに補間した上でモーターに送る (default: `false`)
- **mobile_robot_config.separation_between_wheels** (type: float): 差動二輪型移動台車の車輪間隔．この値が0以下ならば`~/cmd_vel`トピックはsubscribeされない (default: `0.0` meters)
- **mobile_robot_config.adius_of_wheel** (type: float): 差動二輪型移動台車の車輪半径．この値が0以下ならば`~/cmd_vel`トピックはsubscribeされない (default: `0.0` meters)
- **dynamixel_info** (type: string): `U2D2`に接続される全`Dynamixel`サーボモーターの設定（モーターのID等）を記述するYAMLファイルへのパス．`file:///xxx/yyy/zzz.yaml`もしくは`package://package_name/xxx/yyyy/zzz.yaml`のいずれかの形式で指定する．前者は絶対パスで指定し，後者はROS2パッケージ下のファイルを参照する．省略不可

## ノードの起動
### 注意
本ノードは[rclcppのコンポーネント](https://index.ros.org/p/rclcpp_components/)として実装されており，コンポーネントコンテナにロードして使用します．このとき，[launchファイル](./launch/launch.py)にあるように，必ずマルチスレッド対応のコンテナ(`component_container_mt`)にロードしてください．複数のコールバックグループを使用し，マルチスレッドで実行することを前提にしていますので，シングルスレッドコンテナ(`component_container`)にロードするとデッドロックに陥ってハングします．

### 設定ファイルの準備
次の2つの設定ファイルが必要です．
- **ノードパラメータ設定ファイル**: 上述のノードパラメータを設定するYAMLファイル．サンプルは[ここ](./config/default.yaml)
- **モータパラメータ設定ファイル**: モーターの設定YAMLファイル．ノードパラメータ設定ファイルの`dynamixel_info`フィールドから参照される．
記述方法は[Dynamixelのマニュアル](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/#controllers)に従う．サンプルは[ここ](./config/joint_2_0.yaml)

### 起動方法
次のコマンドを投入して起動します．
```bash
$ ros2 launch dynamixel_workbench_controllers launch.py [name:=<node_name>] [config_file:=<config_file>] [container:=<container_name>] [external_container:=true]
```
- **name**: ノードに与える名前 (default: `basic_driver`)
- **config_file**: ノードパラメータ設定ファイルへのパス (default: [default.yaml](./config/default.yaml))
- **container**: ノードのロード先となるコンポーネントコンテナの名前 (default: `dynamixel_workbench_container`)
- **external_container**: `true`ならば，`container`に指定した名前で別途起動していた既存のコンテナにロード．`false`ならば，`container`に指定した名前で新たにコンテナを起動し，それにロード (default: `false`)
