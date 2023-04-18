rmdir /S /Q build
mkdir build
cd build
set OLDPATH=%PATH%
cmake .. -DIPV6=0 -G "MSYS Makefiles"
PATH C:\msys64\usr\bin;%PATH%
make
PATH %OLDPATH%
cd ..
