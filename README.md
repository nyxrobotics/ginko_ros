ginko_ros
====

ROSを使って銀狐・ROSというロボットを動かします。（予定）<br>
<br>
## Description
<br>
### インストールが必要なパッケージ
<br>
### できたこと
・ginko_description:ロボットのモデルをURDFで読みだした(Autodesk Inventor + Blender) → まだやりかたまとめてない<br>
・ginko_joint_controller:ふたばのシリアルサーボを動かした（joint関連のパラメータが全部ソースに直書きなので直したい）<br>
・ginko_motion_player:FSM(状態遷移)のライブラリを使ってモーション再生ソフトを作った(使いにくいライブラリだったので方針変えたい)<br>
<br>
### やっていること
・サーボの角度の個体差をソフトで吸収する部分のデバッグ中<br>
・実機を動かさずにモーションを確認できるようにしたい<br>
<br>
### ためしたいこと
・この辺に書いていることをプロジェクト管理したい<br>
・FSMより使いやすいやつを探したい<br>
候補: <br>
https://github.com/JdeRobot/VisualStates  →エディタが使いにくい。途中でバグって、作成途中のものが開けなくなったりする。<br>
http://wiki.ros.org/smach  →C++で使えない<br>
https://github.com/miccol/ROS-Behavior-Tree  →ドキュメントが足りない。<br>
<br>
### できなかったこと
・ふたばのサーボで外部からのトルクを測る→電流値が電源の消費電流しか出ないのでトルクの方向がわからない＋電流トルク定数がわからない<br>
<br>
### 導入手順
・Gazebo9.0以上をインストール<br>
・ginko_rosをrosのワークスペースにクローン<br>
・銀狐用のgazebo_ros_pkgsをrosのワークスペースにクローン<br>
(https://github.com/nyxrobotics/gazebo_ros_pkgs)(branch:feature/ginko_gazebo_controller)<br>
・decision_makingをrosのワークスペースにクローン<br>
(https://github.com/cogniteam/decision_making)<br>
・ワークスペースでcatkin build<br>
<br>
### Gazeboでの動作手順
terminal-00: roslaunch ginko_description if_gazebo.launch <br>
terminal-01: roslaunch ginko_motion_player fsm_ginko.launch <br>
terminal-02: rviz <br>
→rqtのmessage publisherなどで/motion_commandに<br>
TORQUE_ON→STANDING→WALK_FRONTの順でstringを投げる<br>
<br>
## その他実機での作業(メモ)
・/dev/ttyUSB0があれば、以下のコマンドで一応通信しようとする。リターンが来ないけどエラー処理とかはしていない。<br>
terminal-00: roslaunch ginko_bringup ginko.launch <br>
terminal-01: roslaunch ginko_motion_player fsm_ginko.launch <br>
terminal-02: rviz <br>
<br>
・URDFの確認だけなら以下<br>
terminal-00: roslaunch ginko_description if_rviz_test.launch <br>
<br>
## VS. 
<br>
## Requirement
<br>
## Usage
<br>
## Install
<br>
## Contribution
<br>
## Licence
<br>
## Author
<br>