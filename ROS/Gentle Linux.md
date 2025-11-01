# 😎 Gentle Linux 😎
*Environment: Windows 11 + WSL2 + Ubuntu 22.04 + ros2 + gazebo*

This is a **gentle method** to configure the environment without the requirement of a dual system and remote desktop. And the process runs smoothly.

## Before starting
Environment Requirement: **Windows 10 > 19044** or **Windows 11**

环境要求：**Windows 10 内部版本 19044+** 或 **Windows 11**

## WSL2
启用适用于 Linux 的 Windows 子系统。以管理员身份打开 PowerShell ，输入此命令：
```powershell
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
```

 启用虚拟机功能。以管理员身份打开 PowerShell ，输入此命令：
 ```powershell
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```
重启计算机，完成WSL2安装以及更新。

将 WSL 2 设置为默认版本。打开 PowerShell ，输入此命令：
```powerShell
wsl --set-default-version 2
```

## Ubuntu 22.04
安装 Ubuntu 22.04。以管理员身份打开 PowerShell ，输入此命令：
```powershell
wsl --install -d Ubuntu-22.04
```
安装完毕后需要创建一个新用户，按照提示输入用户名*username*和两次密码即可。Ctrl+D退出Ubuntu终端。

将Ubuntu的虚拟磁盘映像文件移至D盘（你期望的位置，E盘等）。在 D 盘下创建一个名为 WSL 的文件夹（文件夹名字可自行修改），并在该文件夹下创建一个名为 Ubuntu22.04 的文件夹（文件夹名字可自行修改）。

打开 PowerShell，输入此命令：
```powershell
wsl --export Ubuntu-22.04 D:\WSL\Ubuntu-22.04\Ubuntu-22.04.tar
```
D:\WSL\Ubuntu-22.04\Ubuntu-22.04.tar可以按照你所期望的位置和名称进行修改，对应即可。

注销原有的默认在 C 盘的 Ubuntu-22.04。打开 PowerShell，输入此命令：
```powershell
wsl --unregister Ubuntu-22.04
```

将 Ubuntu-22.04 的虚拟磁盘映像文件导入至 D:\WSL\Ubuntu-22.04 文件夹下。打开 PowerShell，输入此命令：
```powershell
wsl --import Ubuntu-22.04 D:\WSL\Ubuntu-22.04 D:\WSL\Ubuntu-22.04\Ubuntu-22.04.tar --version 2
```
同理，D:\WSL\Ubuntu-22.04可以按照你所期望的位置进行修改，D:\WSL\Ubuntu-22.04\Ubuntu-22.04.tar为你刚刚存入Ubuntu-22.04.tar的位置。

初始化并启动Ubuntu。打开 PowerShell，输入此命令：
```powershell
wsl -d Ubuntu-22.04
```
进入ubuntu终端。

在ubuntu终端，输入此命令：
```bash
usermod -aG sudo [username]   #赋予新用户 sudo 的权限
su [username]                 #将 root 账户变为用户账户
```

在 PowerShell，输入此命令：
```powershell
wsl -d Ubuntu-22.04 -u [username] #以用户账户登录 Ubuntu
```
在 PowerShell，输入此命令：
```powershell
wsl -l -v
```
确认 Ubuntu 安装成功。

## Linux GUI
*这部分比较重要，是我们后续能够优雅丝滑使用Ubuntu的重点。请不要安装任何图形化桌面环境（如Xfce等），这有可能会使我们后续进行的部分和图形化桌面环境产生冲突，且无法通过uninstall方法处理干净。*

进入ubuntu终端。更新软件包：
```bash
sudo apt update
```

### Gnome & gedit
Gnome 文本编辑器是 GNOME 桌面环境的默认文本编辑器，旧版本是gedit。进入ubuntu终端，输入命令：

安装Gnome：
```bash
sudo apt install gnome-text-editor -y
```
若要在编辑器中启动 bashrc 文件，请输入： gnome-text-editor ~/.bashrc。

安装gedit：
```bash
sudo apt install gedit -y
```
请输入： gedit，若文本编辑器窗口弹出，安装成功。

### Nautilus
Nautilus 也称为 GNOME 文件，是 GNOME 桌面的文件管理器。 （类似于 Windows 文件资源管理器）。进入ubuntu终端，输入命令：
```bash
sudo apt install nautilus -y
```
若要启动，请输入： nautilus。

### More...
如果需要安装更多功能，如firefox、VLC等，请参考Microsoft官方文档：https://learn.microsoft.com/zh-cn/windows/wsl/tutorials/gui-apps

## Gazebo
进入ubuntu终端，输入命令：
```bash
sudo apt install gazebo
```
安装后输入gazebo即可启动。

## ros2 humble
对于ros的安装，我们直接采用鱼香ROS的一键安装，方便快捷，不易出错。这里放上小鱼社区链接：https://fishros.org.cn/forum/

进入ubuntu终端，输入命令：
```bash
wget http://fishros.com/install -O fishros && bash fishros
```
选择ros2,选择humble，按照提示安装即可。

配置rosdep。同样，直接采用鱼香ROS的一键安装，进入ubuntu终端，输入命令：
```bash
wget http://fishros.com/install -O fishros && bash fishros
```
选择rosdep,按照提示安装即可。

安装完成后，在ubuntu终端，输入命令：
```bash
rosdepc update
```

到此为止，ros基本功能均完成安装，测试一下。在ubuntu终端，输入命令：
```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```
运行小海龟历程。

在ubuntu终端，输入命令：
```bash
rviz2
```
rviz2可以正常启动。

## 🎉 Congratulations! Enjoy!🎉

## Reference
https://blog.csdn.net/weixin_49272453/article/details/151787527
https://learn.microsoft.com/zh-cn/windows/wsl/tutorials/gui-apps
https://zhuanlan.zhihu.com/p/590825660
https://blog.csdn.net/m0_52113469/article/details/142001911
