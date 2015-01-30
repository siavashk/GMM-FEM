In order to run create tetgen.exe and tetlib.lib, do the following steps:

Windows build instructions:

1. Download CMake from http://www.cmake.org/ and install it on your machine (recommended version > 2.8),
2. Open cmake-gui and set the source folder to this folder,
3. Set the binary folder to where you want to build binaries in (recommended ..\build),
4. Press the "Configure" button,
5. Depending on your platform (32-bit/64-bit) and your compiler (e.g. Visual Studio 2008), choose the correct one.
6. Errors will be highlighted in red, press "Conifgure" again to ensure no errors remain,
7. Press "Generate" to generate the solution file (.sln),
8. Open the solution file and select "ALL-BUILD". Build for both release and debug settings.
9. Run tetgen_test1.m to see if the build is correct.