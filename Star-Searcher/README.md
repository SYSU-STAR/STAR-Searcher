# FUAE

**FUAE**, **F**ast **U**AV **A**utonomous **E**xploration in unknown environments. 
目前版本已更新建图传感器为雷达+相机，直接启动fuae.launch即可.
## 一键启动脚本(不建议使用)
```
./start_FUAE.sh
```

##  按照步骤启动
终端1
```
source devel/setup.bash

roslaunch exploration_manager env_simulation.launch
```

终端2
```
source devel/setup.bash

roslaunch exploration_manager uav_simulation.launch
```

终端3
```
source devel/setup.bash

roslaunch exploration_manager fuae.launch
```

终端4
```
source devel/setup.bash

roslaunch exploration_manager rviz.launch
```





