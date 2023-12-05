# tokuron  
## setup  
```
https://github.com/tokuron-2023/tokuron_nav.git  
catkin build  
source ~/.bashrc  
cd tokuron_nav/models  
mkdir -p ~/.gazebo/models/tokuron  
cp -r meshes/ model.* ~/.gazebo/models/tokuron/  
```
## launch  
```
roslaunch tokuron_nav tokuron.launch  
```
##  subscribe topic  
```
rostopic pub /list std_msgs/UInt8MultiArray "layout:  
  dim:  
  - label: ''  
    size: 0  
    stride: 0  
  data_offset: 0  
data:  
- 1  
- 2  
- 3  
- 4  
- 5  
- 6  
- 7"  
```
## call service
```
rosservice call /start_nav "data: true"  
```
