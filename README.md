# mypkg  
課題2  
# sunrise_sunset コマンド  
[![test](https://github.com/TaikiShimodaira/mypkg/actions/workflows/test.yml/badge.svg)](https://github.com/TaikiShimodaira/mypkg/actions/workflows/test.yml)  
## 概要  
このプログラムは、ROS2のノードとしてランダムに選ばれた都市の緯度・経度を基に、日の出・日の入りの時刻を計算して、sunrise_sunsetというトピックで出力する。  
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
