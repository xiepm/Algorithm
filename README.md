# Algorithm

`Algorithm` 是一个基于 CMake 的 C/C++ 机器人算法项目，当前主要构建目标为共享库 `CmpHansCobotAlgorithm`，并包含两个测试可执行程序 `testDemo` 和 `torqueCompensationTest`。

## 目录结构

- `src/`: 核心算法实现，包括状态观测、滤波、摩擦模型、力控与动力学模块
- `src/dynamics/`: 机器人动力学相关实现
- `external/`: 第三方依赖和平台头文件
- `test/`: 测试与数据读写示例代码
- `build.sh`: 本地构建脚本

## 构建要求

- CMake 3.10 或更高
- 支持 C11 / C++14 的编译器
- Eigen3 头文件

项目会优先从以下位置查找 Eigen：

- `external/eigen3`
- `external/include/eigen3`
- `include/eigen3`
- 系统路径 `/usr/include/eigen3` 或 `/usr/local/include/eigen3`

## 快速开始

在项目根目录执行：

```bash
./build.sh
```

也可以手动执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

构建完成后，默认产物包括：

- `build/libCmpHansCobotAlgorithm.so`
- `build/testDemo`
- `build/torqueCompensationTest`

## 测试说明

当前测试目标由 `CMakeLists.txt` 直接生成可执行文件，尚未接入 `ctest`。可按需手动运行：

```bash
./build/testDemo
./build/torqueCompensationTest
```

部分测试代码会在 `test/` 目录下生成运行产物，例如 `test/output.csv`，这些文件已加入忽略规则。

## 说明

- `build/` 为本地产物目录，不纳入版本控制
- `external/` 下包含构建所需头文件和依赖接口，请保留目录结构
- 若后续需要补充接口说明、算法原理或输入输出格式，建议继续扩展本 README
