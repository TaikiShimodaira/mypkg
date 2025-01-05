#!/bin/bash
# SPDX-FileCopyrightText: 2025 Shimodaira Taiki
# SPDX-License-Identifier: BSD-3-Clause

# ROS 2 ワークスペースのディレクトリ
dir=~
[ "$1" != "" ] && dir="$1"

# ワークスペースに移動してビルド
cd $dir/ros2_ws
colcon build
source $dir/.bashrc

# ノードをバックグラウンドで起動
timeout 14 ros2 run mypkg sunrise_sunset &

# 起動後の待機時間
sleep 3

# トピックの出力をキャプチャ
timeout 11 ros2 topic echo /sunrise_sunset_topic > /tmp/sunrise_sunset.log

# 日の出・日の入りの情報が正しく出力されているか確認
if grep -q 'Sunrise:' /tmp/sunrise_sunset.log && grep -q 'Sunset:' /tmp/sunrise_sunset.log; then
    echo "成功"
else
    echo "失敗"
fi

# ログファイルの削除
rm -f /tmp/sunrise_sunset.log
