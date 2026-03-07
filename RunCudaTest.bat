@echo off
echo ========================================
echo RocketSim CUDA Standalone Test
echo ========================================
echo.

REM Set paths
set ROCKETSIM_DIR=C:\Users\PC\Desktop\base\RocketSim
set BUILD_DIR=%ROCKETSIM_DIR%\build
set TEST_EXE=%BUILD_DIR%\CudaStandaloneTest.exe

echo Checking for test executable...
if not exist "%TEST_EXE%" (
    echo ERROR: Test executable not found!
    echo Expected: %TEST_EXE%
    echo.
    echo Building test...
    cd /d "%ROCKETSIM_DIR%"
    
    call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat"
    
    cmake -B build -G Ninja -DROCKETSIM_CUDA=ON -DCMAKE_BUILD_TYPE=Release
    cmake --build build --target CudaStandaloneTest
    
    if not exist "%TEST_EXE%" (
        echo Build failed!
        pause
        exit /b 1
    )
)

echo.
echo Running test...
echo.
"%TEST_EXE%"

echo.
pause
