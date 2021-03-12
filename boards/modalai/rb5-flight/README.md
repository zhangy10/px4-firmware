
## Needed environment variables

export ARM_CROSS_GCC_ROOT=/home/ekatzfey/Qualcomm/Hexagon_SDK/4.1.0.4/4.1.0.4/tools/linaro64
export HEXAGON_ARM_SYSROOT=$ARM_CROSS_GCC_ROOT/aarch64-linux-gnu/libc
export HEXAGON_TOOLS_ROOT=$DEFAULT_HEXAGON_TOOLS_ROOT/Tools

## ARM tools

The recommended toolchain doesn't support enough c++ for px4:
http://releases.linaro.org/archive/14.11/components/toolchain/binaries/aarch64-linux-gnu/gcc-linaro-4.9-2014.11-x86_64_aarch64-linux-gnu.tar.xz
Use this one instead:
https://releases.linaro.org/components/toolchain/binaries/5.1-2015.08/aarch64-linux-gnu/gcc-linaro-5.1-2015.08-x86_64_aarch64-linux-gnu.tar.xz

## Other

The clean command wipes out both apps and qurt builds
- Use make clean instead of make <target> clean

The clean command runs cmake then deletes all the cmake files???

Why does dspal have the cmake_hexagon directory and the boards/* ???
- Need to switch to a single cmake_hexagon directory

qurt_flags.cmake is used twice. In qurt.cmake and in platforms/qurt/CMakeLists.txt.

## Running

Installing the C++ libraries:
adb push $HEXAGON_SDK_ROOT/tools/HEXAGON_Tools/8.4.05/Tools/target/hexagon/lib/v66/G0/pic/libc++.so.1 /usr/lib/rfsa/adsp
adb push $HEXAGON_SDK_ROOT/tools/HEXAGON_Tools/8.4.05/Tools/target/hexagon/lib/v66/G0/pic/libc++abi.so.1 /usr/lib/rfsa/adsp
adb push $HEXAGON_SDK_ROOT/libs/weak_refs/ship/hexagon_toolv84/weak_refs.so /usr/lib/rfsa/adsp

To allow parameter setting:
mkdir -p /usr/share/data/adsp

For the data manager (This location can be set with the f flag):
mkdir -p /home/linaro

This is only needed if all of the dsp files are not in "/usr/lib/rfsa/adsp":
export DSP_LIBRARY_PATH="/usr/lib/rfsa/adsp;/usr/lib/rfsa/dsp/sdk"

```bash
px4 -s /etc/modalai/mainapp.config
```
