ginko_description  
====
  
Gazeboでループの閉じたリンクを再現する手順を記載します  
  
## 手順概要  

大まかに以下の手準になります  

- (1) rviz用に、閉ループのないxacroファイルを作成する  
- (2) gazebo用に、後の作業でループを閉じるための書式に合わせてxacroファイルを作成する  
- (3) (2)で作成したファイルをsdfに変換する  
- (4) sdfファイルを編集し、ループを閉じる  
  
これら作業が必要になる原因が以下  

- xacro, urdfはループを閉じることができない  
    - rosのurdf→sdfパーサの挙動として、urdfからモデルを読み込む際に「親リンク→ジョイント→子リンク」の順で解決していく仕組みになっており、ループを閉じる構造があると無限ループに入ってしまう
    - xacro→urdfパーサ
    
  
## よく使うコマンド  
- xacro→urdf変換:  
> xacro xacro --inorder -o robot.urdf robot.xacro  
- urdf→sdf変換:  
> gz sdf -p robot.urdf > robot.sdf  
  
  