# SkyWay for ROS Examples(Melodic)

SkyWay for ROSを利用するサンプルです。
このサンプルは以下を含みます。

- SkyWay for ROSを操作するためのROS Package
  - Data
    - ROS側からDataConnectionを確立するスクリプト(data_caller.launch)
    - ROS側でDataConnectionを待ち受けるスクリプト(data_callee.launch)
  - Media
    - ROS側からMediaConnectionを確立するスクリプト(media_caller.launch)
    - ROS側でMediaConnectionを待ち受けるスクリプト(media_callee.launch)
- SkyWay Web版のサンプル
  - [Data](examples_web/p2p-data/index.html)
  - [Media](examples_web/p2p-media/index.html)

ROS to ROS, ROS to Webの相互接続が可能です。

## 注意事項
事前にSkyWayに登録し、API Keyを取得してください。
[https://webrtc.ecl.ntt.com/](https://webrtc.ecl.ntt.com/)


## 利用方法
1. ROS/Webいずれかの待受側を起動して下さい
    - Web版はブラウザで起動した時点で待受を開始します
    - ROS版はroslaunchを行った時点で待受を開始します
2. 発信側を操作し、受信側のpeer_idを指定して発信して下さい
    - Web側の場合、画面内のテキストボックスにpeer_idを入力し、Connectボタンを押して下さい。
    - ROS側の場合、configディレクトリ内のyamlファイルを編集し、roslaunchで実行して下さい。

### ROS版
まず添付のWebRTC Gatewayを起動して下さい。
次にroslaunch実行前にAPI_KEYをexportしてください。

```shell
$./gateway
$export API_KEY=$YOUR_API_KEY
$roslaunch skyway_for_ros_examples media_callee.launch
```

### Web版
[key.js](examples_web/_shared/key.js)を編集してください。
Media用のサンプルの場合、PCにカメラとマイクを接続し、ブラウザからのアクセスを許可して下さい。
