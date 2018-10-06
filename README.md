ginko_ros
====

ROSを使って銀狐・ROSというロボットを動かします。（予定）

## Description
### インストールが必要なパッケージ
・FSM:http://wiki.ros.org/decision_making<br>

### できたこと
・ginko_description:ロボットのモデルをURDFで読みだした(Autodesk Inventor + Blender) → まだやりかたまとめてない<br>
・ginko_joint_controller:ふたばのシリアルサーボを動かした（joint関連のパラメータが全部ソースに直書きなので直したい）<br>
・ginko_motion_player:FSM(状態遷移)のライブラリを使ってモーション再生ソフトを作った(使いにくいライブラリだったので方針変えたい)<br>

### やっていること
・サーボの角度の個体差をソフトで吸収する部分のデバッグ中<br>
・実機を動かさずにモーションを確認できるようにしたい<br>

### ためしたいこと
・この辺に書いていることをプロジェクト管理したい<br>
・FSMより使いやすいやつを探したい<br>
候補: <br>
https://github.com/JdeRobot/VisualStates →使い方がよくわからない<br>
http://wiki.ros.org/smach  →C++で使えない<br>


### できなかったこと
・ふたばのサーボで外部からのトルクを測る→電流値が電源の消費電流しか出ないのでトルクの方向がわからない＋電流トルク定数がわからない<br>


## Demo

## VS. 

## Requirement

## Usage

## Install

## Contribution

## Licence

## Author
