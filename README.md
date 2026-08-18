# RS2026 Windows Business Source

Platform: Windows x64
Toolchain: Visual Studio 2019

`Common`, `SMRobotCore`, and `SMRobotPlatform` are supplied under
`PrebuiltPackages`. Their source directories are intentionally absent.

Qt 5.15.5 and OMPL are external development prerequisites. FCL is distributed
as a separate dependency package because the public Collision headers expose
FCL types. Set `PREBUILD_DIR` for Qt/OMPL and set `SMROBOT_EXTERNAL_FCL_ROOT`
to the extracted `fcl_static_x64` root, then configure and build with the
supplied presets:

```powershell
cmake --preset windows-business-vs2019-x64 `
  -DPREBUILD_DIR=D:/path/to/windows-prebuild `
  -DSMROBOT_EXTERNAL_FCL_ROOT=D:/path/to/fcl_static_x64
cmake --build --preset windows-business-vs2019-x64-release
```

KTX is a private static implementation dependency already contained in
`SMRobotPlatform::RenderCore`. Do not install KTX and do not pass `Ktx_DIR`
when building this Business Source package.

User-facing SDK examples are under `tutorials/prebuilt`. Installed-package
tutorials cover the prebuilt `Common`, `SMRobotCore`, and `SMRobotPlatform`
libraries. Other source-delivered packages are copied in full, including their
module-owned regression tests, feature probes, diagnostics, and tutorials.

Installed-package black-box checks remain in the producer repository under
`tests/prebuilt_external_validation`; they are release gates, not package content.

Runtime model data is packaged under `data/` by default. `SMROBOT_DATA_ROOT`
may still be used to override the data root for a larger or shared model library.

The packaged `.LIC` file is prepared automatically during CMake configure and
is also copied after application target builds to
`<build>/<Config>/bin/license`. No manual license deployment is required.