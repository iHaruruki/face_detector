# face_detector
## Node and Topic
```mermaid
flowchart LR
    A(["/camera/image_raw"]) ==> B[face_detector]
```
## インストール
```
$ sudo apt-get update # パッケージリストを更新
$ sudo apt-get install libopencv-dev # OpenCVのインストール
$ sudo apt-get install ros-humble-cv-bridge # cv_bridgeのインストール
$ sudo apt-get install ros-humble-image-transport # image_transportのインストール（オプション）

$ cd ~/ros2_ws/src  #Go to ros workspace
$ git clone https://github.com/iHaruruki/face_detector.git #clone this package
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source install/setup.bash
```
## 使い方
```
$ ros2 run face_detection_node_cpp face_detector
```
### 注意
face_detector.cpp 内のサブスクリプション部分で使用しているトピック名 /astra_camera/image_raw は、実際に使用しているトピック名と一致している必要があります。異なる場合は、以下のコマンドで現在のトピックを確認し、適切なトピック名に変更してください。
##### 確認方法
```
$ ros2 topic list
```
