rmdir /S /Q build
mkdir build
cd build
set OLDPATH=%PATH%
PATH C:\Program Files (x86)\GNU Arm Embedded Toolchain\10 2021.10\bin;%PATH%
cmake .. -DIPV6=0 -G "MSYS Makefiles"
PATH C:\msys64\usr\bin;%PATH%
make
PATH %OLDPATH%
cd ..
