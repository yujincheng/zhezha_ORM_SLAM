# 小车控制

## 1. 登陆

```Shell
ssh root@192.168.31.144
```
pw: autolabor

## 2. 环境设置

```Shell
source /opt/ros/kinetic/setup.bash
cd firmware/autolabor2.5/launch
./lidar_nav_sh.sh &
```
## 3. 小车控制

```Shell
rostopic pub cmd_vel geometry_msgs/Twist -r 2 -- '[1,0,0]' '[0,0,3.5]'
```
其中 -r 参数代表了指令发送的频率（Hz单位，例子中为2Hz），第一个[x, y, z] 代表了位移，第二个[u, v, w]代表了旋转。

或者可以将指令写到一个文件中，请参考command_run.sh和command_yz.sh

# TX1控制

## 1. 登陆

```Shell
ssh nvidia@192.168.31.90
```
pw: nvidia

登陆后开启X11VNC进行调试
```Shell
./x11vncscript
```
VNC请访问192.168.31.90，pw: nvidia

## 2. ORB SLAM

```Shell
cd ~/ORB_SLAM2/build
make
./slamscript
```
在出现Loading ORB Vocabulary以后，optitrack可以开始准备录制。
结束后，请在应用窗口双击```Esc```退出程序，将在build文件夹下保存两个轨迹文档。

## 3. 方波调试

```Shell
cd ~/Dependency/jetsonTX1GPIO
g++ test_gpio.cpp jetsonGPIO.c --std=c++11 -o testgpio
sudo ./testgpio k
```
k为频率（Hz）