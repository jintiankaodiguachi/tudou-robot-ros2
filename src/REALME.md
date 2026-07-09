# 启动rivz显示tudou模型

```bash
ros2 launch tudou_description display_robot.launch.py model:=./src/tudou_description/urdf/tudou/tudou.urdf.xacro
```

# 启动ELF2与下位机通信节点

```bash
ros2 run tudou_bringup stm32_bridge
```

# 启动键盘控制节点

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

# 启动建图

```bash
ros2 launch tudou_bringup tudou_slam.launch.py
```
# 保存地图

```bash
ros2 run nav2_map_server map_saver_cli -f ~/hoho/vlbot/tudou_ws/src/tudou_navigation/maps/my_room_802_map

ros2 run nav2_map_server map_saver_cli -f ~/hoho/vlbot/tudou_ws/src/tudou_navigation/maps/health_care_map
```


# 查看设备串口

```bash
ls -l /dev/ttyUSB*

ls -l /dev/ttyUSB* /dev/ttyACM*

sudo chmod 777 /dev/ttyACM0

ls -l /dev/serial/by-id/

sudo dmesg -w
```


# 里程计到雷达的标准的TF树

odom -> base_footprint -> base_link -> laser_cylinder_link -> laser_link


# 建图流程

```bash
ros2 launch tudou_bringup tudou_slam.launch.py

rviz2中，在左侧添加 Map 插件，订阅 /map 话题 ；添加 LaserScan 插件，订阅 /scan 话题

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ros2 run nav2_map_server map_saver_cli -f ~/hoho/vlbot/tudou_ws/src/tudou_navigation/maps/my_room_802_map
```

# 启动导航

```bash
ros2 launch tudou_bringup bringup.launch.py

ros2 launch tudou_navigation2 tudou_nav2.launch.py
```

# 语音导航到目标点

```bash
ros2 launch tudou_bringup bringup.launch.py

ros2 launch tudou_voice_navigation tudou_voice_nav2.launch.py
```

# 2D 位置点估计话题发布 与 2D 目标点话题发布（先点点，再运行命令等待刷新）

```bash
ros2 topic echo /initialpose

ros2 topic echo /goal_pose
```

# 启动单点巡护

```bash
ros2 launch tudou_bringup bringup.launch.py

ros2 launch tudou_patrol tudou_patrol.launch.py
```

# 启动多点巡护

```bash
ros2 launch tudou_bringup bringup.launch.py

ros2 launch tudou_patrol2 tudou_patrol2.launch.py
```













