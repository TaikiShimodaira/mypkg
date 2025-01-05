# mypkg  
課題2  
#   日の出・日の入りを表示
[![test](https://github.com/TaikiShimodaira/mypkg/actions/workflows/test.yml/badge.svg)](https://github.com/TaikiShimodaira/mypkg/actions/workflows/test.yml)  
## 概要  
このパッケージは、ROS2のパブリッシャとしてランダムに選ばれた都市の緯度・経度を基に、日の出・日の入りの時刻を計算して、sunrise_sunset_topicというトピックで出力する。  
## 使用方法  
### パッケージをクローン  
```
git clone https://github.com/TaikiShimodaira/mypkg.git
```
### パッケージをビルド
```
cd ~/ros2_ws
colcon build
```
### 実行  
端末1  
```
 ros2 run mypkg sunrise_sunset
```
端末2
```
 ros2 topic echo /sunrise_sunset_topic
```
### 実行例（端末2で表示）  
```
data: 'Location: Tokyo, Japan

  Sunrise: 06:51

  Sunset: 16:41'
---
data: 'Location: London, UK

  Sunrise: 08:05

  Sunset: 16:06'
---
```
## 必要なソフトウェア  
- python3
## 動作環境
- Ubuntu 22.04 LTS  
- ROS 2 バージョン: foxy
## 参考文献
- https://www.hinodehinoiri.info/WorldSun.php
- https://sciencenote.jp/diurnal-movement-of-the-sun/
- https://hsato2011.hatenablog.com/entry/2024/06/24/093518
- https://ideal-user-interface.hatenablog.com/entry/20160203/1454504050  
## ライセンス 
- このソフトウェアは 3条項BSDライセンス の下で再頒布および使用が許可されています。  
- このパッケージのコードの一部は，本人の許可を得て下記のスライドのもの（CC-BY-SA 4.0 by Ryuichi Ueda）を，自身の著作としたものです。
  - https://github.com/ryuichiueda/slides_marp/tree/master/robosys2024
-  © 2024 Taiki Shimodaira  
