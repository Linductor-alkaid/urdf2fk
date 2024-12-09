# urdf2fk

`urdf2fk` 是一个 C++ 项目，旨在通过读取 URDF（统一机器人描述格式）文件，提取关节和连杆信息，使用 DH 参数（Denavit-Hartenberg）方法计算末端执行器的正向运动学。用户可以输入每个关节的角度，程序会输出末端执行器的三维坐标位置。

## 项目结构

```text
.
├── CMakeLists.txt       # CMake 构建配置文件
├── License              # 项目许可证文件
├── tinyxml2.cpp         # tinyxml2 实现文件
├── tinyxml2.h           # tinyxml2 头文件
└── urdf2fk.cpp          # 主程序文件
```

- `CMakeLists.txt`：CMake 配置文件，定义如何编译项目。
- `tinyxml2.cpp` 和 `tinyxml2.h`：包含了 `tinyxml2` 库的实现，用于解析 URDF 文件。
- `urdf2fk.cpp`：主程序，包含了关节参数解析、DH 变换矩阵计算以及末端执行器正向运动学的实现。

## 项目功能

- **解析 URDF 文件**：从 URDF 文件中读取关节参数，包括关节名称、连杆长度、偏移量、扭转角度和旋转轴。
- **DH 参数计算**：基于 DH 参数公式计算变换矩阵。
- **正向运动学计算**：输入关节角度，计算末端执行器的位姿（位置和姿态），输出末端执行器的三维坐标（X、Y、Z）。

## 编译与运行

### 1. 安装依赖

本项目使用了 `tinyxml2` 库来解析 URDF 文件，`tinyxml2` 是一个轻量级的 XML 解析库，你可以从 [TinyXML2 GitHub](https://github.com/leethomason/tinyxml2) 获取源代码，并将其加入到项目中。

### 2. 配置 CMake

项目使用 CMake 来构建。在项目根目录下创建 `build` 文件夹并运行以下命令来编译项目：

```bash
mkdir build
cd build
cmake ..
make
```

### 3. 运行程序

编译完成后，运行生成的可执行文件：

```bash
./urdf2fk
```

程序会读取 `robot_model.urdf` 文件，提示你输入每个关节的角度，并计算末端执行器的三维位置。

### 4. 用户输入

程序启动后，会提示你输入每个关节的角度（以弧度为单位），例如：

```
Enter the angle for joint joint_1 (in radians): 0.5
Enter the angle for joint joint_2 (in radians): 1.2
```

### 5. 输出末端执行器位置

输入完所有关节的角度后，程序会根据 DH 参数计算末端执行器的位置，并输出如下信息：

```
End-Effector Position: X: 0.7, Y: 0.0, Z: 0.7
```

## 代码说明

### 1. `JointLinkParam` 结构体

该结构体保存每个关节的 DH 参数，具体包括：
- `name`: 关节名称
- `theta`: 关节角度
- `d`: 连杆偏移
- `a`: 连杆长度
- `alpha`: 连杆扭转角度
- `axis`: 关节旋转轴

### 2. `getTransformationMatrix` 函数

基于 DH 参数，计算给定关节的变换矩阵。变换矩阵用于描述从当前关节到下一个关节的坐标变换。

### 3. `calculateEndEffectorFK` 函数

该函数接收关节的 DH 参数和用户输入的角度，通过累乘变换矩阵计算末端执行器的位置。

### 4. `main` 函数

- 加载并解析 URDF 文件，提取关节信息。
- 提示用户输入每个关节的角度。
- 调用 `calculateEndEffectorFK` 函数计算并输出末端执行器的位置。

## 示例运行

假设你有一个简单的两关节机器人模型：

```xml
<robot name="simple_robot">
  <link name="base_link"/>
  <link name="link_1"/>
  <link name="link_2"/>

  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="1.0 0.0 0.5"/>
    <axis xyz="1.0 0.0 0.0"/>
  </joint>

  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="link_2"/>
    <origin xyz="0.5 0.0 0.2"/>
    <axis xyz="0.0 1.0 0.0"/>
  </joint>
</robot>
```

输入关节角度：

```
Enter the angle for joint joint_1 (in radians): 0.5
Enter the angle for joint joint_2 (in radians): 1.2
```

输出末端执行器位置：

```
End-Effector Position: X: 0.7, Y: 0.0, Z: 0.7
```

## 许可证

此项目采用 [MIT 许可证](LICENSE)，欢迎自由使用、修改和分发。
