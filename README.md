## 使用说明
解决多节点运行，和终端启动繁琐的问题。

### 安装
```bash
cd ~/ws/xtdrone_ws/
catkin build srun
```

### 说明
multirotor_keyboard_control.py: 增加'c'，一键起飞
multirotor_communication.py: 增加延迟启动，和少许修改，以和PX4联合启动


### 使用
vins
```bash
roslaunch srun vins.launch
```

ego: uav起飞完毕后再启动
```bash
roslaunch srun ego.launch
```

vins ego: 联合启动，不建议使用，会导致初始深度信息受起飞干扰
```bash
roslaunch srun vins_ego.launch
```