# Prebuilt tutorials

这些教程只通过安装后的 CMake package 使用 RS2026 SDK，不依赖源码目录或源码 target。

```powershell
cmake -S tutorials/prebuilt -B build/prebuilt_tutorials `
  -G "Visual Studio 16 2019" -A x64 `
  -DCMAKE_PREFIX_PATH=D:/path/to/PrebuiltPackages
cmake --build build/prebuilt_tutorials --config Release
```

RobotSDK 教程需要一个 URDF 或 Simscape 文件。可以把文件路径作为第一个参数传入，或者设置 `SMROBOT_DATA_ROOT`，让教程从该数据根下选择默认模型。

```powershell
$env:SMROBOT_DATA_ROOT = "D:/path/to/data"
./build/prebuilt_tutorials/robot_sdk/QuickStart/Release/RobotSDKQuickStart.exe
./build/prebuilt_tutorials/robot_sdk/CollisionQuickStart/Release/RobotSDKCollisionQuickStart.exe
```
