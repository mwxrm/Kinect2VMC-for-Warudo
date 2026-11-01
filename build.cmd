@echo off
setlocal enabledelayedexpansion

rem ============================================================================
rem  Kinect 2.0 -> VMC -> Warudo : one-click build
rem
rem  Requires only the .NET SDK (its bundled MSBuild) and the Kinect for Windows
rem  SDK 2.0. Visual Studio is NOT required.
rem
rem  This script is deliberately pure ASCII: cmd.exe reads .cmd files using the
rem  OEM code page, so non-ASCII text here would be mangled on a Chinese Windows.
rem  All Chinese documentation lives in docs\ instead.
rem ============================================================================

cd /d "%~dp0"

set "MSBUILD=C:\Program Files\dotnet\sdk\10.0.400\MSBuild.exe"
if exist "%MSBUILD%" goto :have_msbuild

set "MSBUILD="
for /f "delims=" %%i in ('dir /b /o-n "C:\Program Files\dotnet\sdk" 2^>nul') do (
  if not defined MSBUILD (
    if exist "C:\Program Files\dotnet\sdk\%%i\MSBuild.exe" (
      set "MSBUILD=C:\Program Files\dotnet\sdk\%%i\MSBuild.exe"
    )
  )
)
if not defined MSBUILD (
  echo [ERROR] MSBuild.exe not found. Install the .NET SDK, or edit MSBUILD in this script.
  pause
  exit /b 1
)
:have_msbuild

set "KINECTDLL=C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\Assemblies\Microsoft.Kinect.dll"
if not exist "%KINECTDLL%" (
  echo [ERROR] Kinect SDK assembly not found:
  echo         %KINECTDLL%
  echo         Install Kinect for Windows SDK 2.0 first.
  pause
  exit /b 1
)

echo.
echo === 0/8  Restoring NuGet assets ===
rem A fresh clone, or any build after deleting obj\, has no project.assets.json. Rebuild
rem alone does not restore, so restore explicitly for every project first.
for %%P in (src\Core\Kinect2VMC.Core.csproj src\App\Kinect2VMC.App.csproj src\Monitor\VmcMonitor.csproj src\VmcProbe\VmcProbe.csproj src\SelfTest\SelfTest.csproj src\UiSmoke\UiSmoke.csproj) do (
  "%MSBUILD%" %%P /t:Restore /v:quiet /nologo
  if errorlevel 1 goto :failed
)

echo.
echo === 1/8  Building core library ===
"%MSBUILD%" src\Core\Kinect2VMC.Core.csproj /t:Rebuild /p:Configuration=Release /v:minimal /nologo
if errorlevel 1 goto :failed

echo.
echo === 2/8  Building main application ===
"%MSBUILD%" src\App\Kinect2VMC.App.csproj /t:Rebuild /p:Configuration=Release /v:minimal /nologo
if errorlevel 1 goto :failed

echo.
echo === 3/8  Building VMC monitor ===
"%MSBUILD%" src\Monitor\VmcMonitor.csproj /t:Rebuild /p:Configuration=Release /v:minimal /nologo
if errorlevel 1 goto :failed

echo.
echo === 4/8  Building offline self-test ===
"%MSBUILD%" src\SelfTest\SelfTest.csproj /t:Rebuild /p:Configuration=Release /v:minimal /nologo
if errorlevel 1 goto :failed

echo.
echo === 5/8  Building UI smoke test ===
"%MSBUILD%" src\UiSmoke\UiSmoke.csproj /t:Rebuild /p:Configuration=Release /v:minimal /nologo
if errorlevel 1 goto :failed

echo.
echo === 6/8  Building VMC probe (reads Warudo's own VMC sender) ===
"%MSBUILD%" src\VmcProbe\VmcProbe.csproj /t:Rebuild /p:Configuration=Release /v:minimal /nologo
if errorlevel 1 goto :failed

echo.
echo === 7/8  Running self-tests (no Kinect needed) ===
echo.
echo --- maths / protocol / solver ---
src\SelfTest\bin\Release\SelfTest.exe
if errorlevel 1 (
  echo.
  echo [WARNING] The core self-test did not fully pass. The build will still be
  echo           produced, but the tracking maths may be wrong.
)

echo.
echo --- user interface (opens and closes a window) ---
src\UiSmoke\bin\Release\UiSmoke.exe
if errorlevel 1 (
  echo.
  echo [WARNING] The UI smoke test failed. The window may not open correctly.
)

echo.
echo === 8/8  Collecting output into dist\ ===
if exist dist rmdir /s /q dist
mkdir dist
copy /y src\App\bin\Release\Kinect2VMC.exe            dist\ >nul
copy /y src\App\bin\Release\Kinect2VMC.exe.config     dist\ >nul
copy /y src\App\bin\Release\Kinect2VMC.Core.dll       dist\ >nul
copy /y src\App\bin\Release\Microsoft.Kinect.dll      dist\ >nul
copy /y src\App\bin\Release\Microsoft.Kinect.xml      dist\ >nul
copy /y src\Monitor\bin\Release\VmcMonitor.exe        dist\ >nul
copy /y src\Monitor\bin\Release\VmcMonitor.exe.config dist\ >nul
copy /y src\VmcProbe\bin\Release\VmcProbe.exe         dist\ >nul
copy /y src\VmcProbe\bin\Release\VmcProbe.exe.config  dist\ >nul

rem The hand tracker is started by the UI, so it has to sit beside the exe.
if not exist dist\tools mkdir dist\tools
copy /y tools\hand_tracker.py       dist\tools\ >nul
copy /y tools\poses.json            dist\tools\ >nul 2>nul
copy /y tools\requirements.txt      dist\tools\ >nul 2>nul
copy /y tools\make_approx_poses.py  dist\tools\ >nul 2>nul
copy /y tools\try_axes.py           dist\tools\ >nul 2>nul

echo.
echo ============================================================
echo  Build finished. Run these from the dist\ folder:
echo.
echo    Kinect2VMC.exe   main program  (Kinect -^> VMC)
echo    VmcMonitor.exe   optional listener to verify this program's own output
echo    VmcProbe.exe     reads WARUDO's VMC sender, to inspect the model itself
echo.
echo  Read docs\ for first-time setup (Chinese).
echo ============================================================
echo.
pause
exit /b 0

:failed
echo.
echo [FAILED] Build error. Please report the messages above.
pause
exit /b 1
