## 使用说明
解决多节点运行，和终端启动繁琐的问题。

### 安装
```bash
cd ~/ws/xtdrone_ws/
catkin build srun
```

### 说明
multirotor_keyboard_control.py: 增加'c'，一键起飞  
multirotor_communication.py: 增加延迟启动，和BUG修复，以和PX4联合启动


### 使用
vins  
(vins-funsion + px4 + world)
```bash
roslaunch srun vins.launch
```

ego_vins  
(适配vins-funsion的ego-planner-swarm)  
```bash
#uav起飞完毕后再启动
roslaunch srun ego_vins.launch
```

~~vins ego: 联合启动，不建议使用，会导致初始深度信息受起飞干扰~~


lio  
(fast-lio + px4 + world)
```bash
roslaunch srun lio.launch
```


ego_lio  
(适配fast-lio的ego-planner-swarm)
```bash
#uav起飞完毕后再启动
roslaunch srun ego_lio.launch
```


### 配置
以vins为例进行说明，其余类似；目前仿真场景只针对单机，所以没有多机配置。
- vins.launch修改仿真场景  

```xml
<!-- indoor4: 迷宫(视觉)；outdoor5: 树林(视觉) 
    "_laser"代表相应的(雷达)配置-->
<arg name="scene" default='indoor4'/>
```

- ($find ego_planner)/launch/single_uav.launch修改航迹点数量  

```xml
<!-- 雷达是single_uav_lio.launch -->
<arg name="point_num_set" value="1" />
```

  具体航迹点在single_uav.launch、advanced_param_xtdrone.xml下修改。