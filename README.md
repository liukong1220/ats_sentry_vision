

# **RV2.0配置方法**

## 一：ubuntu22.04安装ROS2

##### （1）使用鱼香ROS安装

Ctrl + ALT +T 一键打开终端 执行下面指令

```shell
wget http://fishros.com/install -O fishros && . fishros
```

![img](https://i-blog.csdnimg.cn/blog_migrate/b672bc4c3daeaaa5101c0158264ce1df.png)

- 选择数字***1***进行ROS2安装，22.04选择安装ROS2-humble,安装桌面版即可
- 安装好ROS后继续使用鱼香ROS选择数字***3***进行rosdep的更新，随后会提示运行***rosdepc update***指令（网络环境不能太差，最好不使用校园网或者使用clash）

#### **(2)**手动安装（费事）

https://blog.csdn.net/weixin_44733606/article/details/132576418?spm=1001.2014.3001.5506

（教程网页包含ROS卸载）

## 二：下载代码与配置环境

### 创建工作空间

```Shell
mkdir -p ~/ros_ws/src
```



### 下载源代码

在 `ros_ws/src` 目录下

rm_vision,自瞄，buff，海康相机，云台，串口通讯，自动录包

```Shell
git clone https://github.com/FaterYU/rm_vision.git
git clone https://github.com/FaterYU/rm_auto_aim.git
git clone https://github.com/FaterYU/rm_buff.git
git clone https://github.com/FaterYU/ros2_hik_camera.git
git clone https://github.com/FaterYU/rm_gimbal_description.git
git clone https://github.com/FaterYU/rm_serial_driver.git
git clone https://github.com/FaterYU/rm_auto_record.git
sudo apt install ros-humble-foxglove-bridge
```



### 安装openvino2023官网APT

安装网站，下面写有错误的话可以看一下官网

https://docs.openvino.ai/archive/2023.1/openvino_docs_install_guides_installing_openvino_apt.html



### 第 1 步：设置 OpenVINO 工具包 APT 存储库

​	1.安装存储库的 GPG 密钥

```
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
```

​	2.将此密钥添加到系统密钥环中：

```
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
```

​	注意

​	你可能需要安装 GnuPG：

```
sudo apt-get install gnupg
```



通过以下命令添加存储库：

检查已安装的软件包和版本

运行以下命令：

apt list --installed | grep openvino	Ubuntu 22 的

```
echo "deb https://apt.repos.intel.com/openvino/2023 ubuntu22 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2023.list
```



​	3.通过 update 命令更新软件包列表：

```
sudo apt update
```



​	4.验证 APT 存储库是否已正确设置。运行 apt-cache 命令查看所有可用 OpenVINO 软件包和组件的列表：

```
apt-cache search openvino
```



### 第 2 步：使用 APT 包管理器安装 OpenVINO 运行时

1. 安装 OpenVINO 运行时

最新版本

运行以下命令：

```
sudo apt install openvino
```



​	2.检查已安装的软件包和版本

运行以下命令：

```
apt list --installed | grep openvino
```



## 三：编译

在 `ros_ws` 目录下

安装依赖

```Shell
rosdep install --from-paths src --ignore-src -r -y
```

编译


```Shell
colcon build --symlink-install
```

### **注意：**

如果在安装依赖过程中出现以下错误

> Make Error at /opt/ros/humble/share/io_context/cmake/io_context-extras.cmake:17 (find_package):  By not providing "Findasio_cmake_module.cmake" in CMAKE_MODULE_PATH this  project has asked CMake to find a package configuration file provided by  "asio_cmake_module", but CMake did not find one.   Could not find a package configuration file provided by "asio_cmake_module"  with any of the following names:     asio_cmake_moduleConfig.cmake    asio_cmake_module-config.cmake   Add the installation prefix of "asio_cmake_module" to CMAKE_PREFIX_PATH or  set "asio_cmake_module_DIR" to a directory containing one of the above  files.  If "asio_cmake_module" provides a separate development package or  SDK, be sure it has been installed. Call Stack (most recent call first):  /opt/ros/humble/share/io_context/cmake/io_contextConfig.cmake:41 (include)  /opt/ros/humble/share/serial_driver/cmake/ament_cmake_export_dependencies-extras.cmake:21 (find_package)  /opt/ros/humble/share/serial_driver/cmake/serial_driverConfig.cmake:41 (include)  /opt/ros/humble/share/ament_cmake_auto/cmake/ament_auto_find_build_dependencies.cmake:67 (find_package)  CMakeLists.txt:20 (ament_auto_find_build_dependencies)

可以输入以下指令

**sudo apt-get install ros-humble-asio-cmake-module**





## 四：调参地方![image-20240905192318576](/home/aw/.config/Typora/typora-user-images/image-20240905192318576.png)



![image-20240905192621422](/home/aw/.config/Typora/typora-user-images/image-20240905192621422.png)





![image-20240905192635175](/home/aw/.config/Typora/typora-user-images/image-20240905192635175.png)



这三个文件可以自己看