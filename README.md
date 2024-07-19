# 环境搭建与代码运行手册
本说明包含了从Ubuntu操作系统搭建ROS2 Hunble的运行环境以及招新题目第一题代码的运行说明
## 环境搭建
### 在Windows系统上安装Ubuntu
Ubuntu是基于Linux内核的桌面PC操作系统，也称为Linux的发行版本，而ROS2对于Linux系统的支持最好，在Windows系统上运行Ubuntu包含以下几种方式：
1. 虚拟机
2. 双系统
3. Windows Subsystem for Linux (WSL)（Windows子系统）
这里采用第一种方法。

#### 下载VMware虚拟机软件
VMware在今年五月份被Broadcom收购后，向个人用户免费开放了VMware Workstation Pro，可前往Broadcom官网注册账号进行下载，这里使用VMware Workstation Pro 16.2.4。
[Broadcom官网下载地址](https://support.broadcom.com/group/ecx/downloads)
#### 选择Ubuntu版本
Ubuntu在偶数年的四月份会发布长期支持版本（LTS），保证在五年内持续维护更新，这里需要选择与ROS2 Humble对应的LTS版本22.04 LTS。
**每一个版本的ROS2都为一个特定版本的Ubuntu版本构建，且具有许多依赖关系，在安装之前需要选择与ROS2对应的Ubuntu版本。**
* 在安装时需留意对应的cpu架构，官网下载默认为amd64
#### 下载Ubuntu的系统镜像
[官网下载链接](https://ubuntu.com/download/desktop)
#### 安装Ubuntu22
在VMware Workstation Pro中选择新建虚拟机，设置路径，配置虚拟机硬盘大小
推荐在设置中移除打印机，分配较多的硬盘大小
（至少为2G），开启虚拟化引擎，提高虚拟机性能。
配置好之后可以成功启动模拟机。

* 可能遇到的问题：
1. 启动虚拟机失败，提示主机Intel VT-x处于禁用状态。
> VT-x是intel运用Virtualization虚拟化技术中的一个指令集，是CPU的硬件虚拟化技术，VT可以同时提升虚拟化效率和虚拟机的安全性。
解决方法：
重启进入BIOS，在Advanced设置中的CPU configuration启用VT-x
2. 虚拟机中安装Ubuntu时卡住
解决方法：
为虚拟机多分配一些内存和线程。
检查发现内存只分配了2G，增加到4G内存就恢复流畅。

### 在Ubuntu上安装ROS2 Humble
1. Ubuntu镜像换源
Ubuntu安装后的官方源默认是美国的服务器，在国内安装软件会受到比较大的限制，下载软件很慢，需要切换成国内源。一般使用的国内源有阿里源，网易源和高校源，这里使用阿里源
方法：
打开系统设置，在软件和更新中更改下载服务器，选择国内源

2. 在Ubuntu中安装软件

在Ubuntu中安装软件主要有以下几种方式：
1. 使用Ubuntu软件商店
2. 使用apt命令从服务器下载
3. 使用dpkg命令安装deb包
4. 使用源码进行编译安装
安装ROS2：
ROS2对于Ubuntu系统来说属于第三方软件，需要先添加源、再添加秘钥才才使用apt进行安装，过程较为麻烦，这里我们使用简易方法
也可以跟随官方教程进行安装[https://docs.ros.org/en/humble/Installation.html](https://docs.ros.org/en/humble/Installation.html)
一键安装ROS2（鱼香ROS开发的一键安装脚本，可通过该脚本安装其他常用软件）

2.1. ctrl+alt+t打开终端

2.2. 输入命令
```
wget http://fishros.com/install -O fishros && . fishros
```
选择ROS2，Humble桌面版完成安装过程。
安装完成后在终端输入ros2检查是否安装成功。

3. 配置ROS2环境
[官方配置文档](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html)
3.1 将ros2加入系统默认的环境中
输入以下命令，将ros2加入bashrc中，这样每次启动终端时都可以自动加载ros2
```
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

至此，ROS2 Humble已经成功在Ubuntu上安装完毕。可以在终端通过ros2命令来运行ROS2

## 代码运行说明
1. 创建一个新的ROS2工作空间
ctrl+alt+t打开终端，输入以下命令
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

3. 安装代码功能包
在同一目录下输入以下命令：
```
git clone https://github.com/RhysRusty/ros2_turtle_game/tree/master
```

3. 安装依赖
输入以下命令
```
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro humble -y
```

4. 编译工作空间
```
cd ~/ros2_ws
colcon build
```

5. 设置环境
同一目录下输入
```
source install/setup.bash
```

6. 运行ROS2 turtlesim节点
```
ros2 run turtlesim turtlesim_node
```

7. 运行题目代码
开启一个新的终端，输入以下命令
```
ros2 run turtle_motion go_to_goal
```
