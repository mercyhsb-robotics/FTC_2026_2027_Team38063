@echo off
rem Regenerates the autonomous route diagrams in docs\autos: index.html plus one SVG per route and alliance.
rem
rem It compiles ONLY the pure-Java sources (BioBuzzConstants and the autoshoot core, routes and render
rem packages) with plain javac into a temporary folder, runs RenderRoutesMain and deletes the folder again.
rem javac follows the imports from the entry point, so nothing else is read. No Gradle, no Android SDK
rem and no robot are needed; any JDK 8 or newer will do.
rem
rem Usage:  tools\render-autos.bat [output directory]      (default: docs\autos)
setlocal EnableExtensions

rem always work from the repository root, wherever the script is started from
cd /d "%~dp0.."
if errorlevel 1 (
    echo Cannot change to the repository root.
    exit /b 1
)

set "SRC=TeamCode\src\main\java"
set "MAIN_CLASS=org.firstinspires.ftc.teamcode.autoshoot.render.RenderRoutesMain"
set "MAIN_FILE=%SRC%\org\firstinspires\ftc\teamcode\autoshoot\render\RenderRoutesMain.java"
set "OUT_DIR=%~1"
if "%OUT_DIR%"=="" set "OUT_DIR=docs\autos"

rem javac: JAVA_HOME if it has one, otherwise whatever is on the PATH
set "JAVAC=javac"
set "JAVA=java"
if defined JAVA_HOME (
    if exist "%JAVA_HOME%\bin\javac.exe" (
        set "JAVAC=%JAVA_HOME%\bin\javac.exe"
        set "JAVA=%JAVA_HOME%\bin\java.exe"
    )
)
"%JAVAC%" -version >nul 2>&1
if errorlevel 1 (
    echo javac was not found. Install a JDK ^(8 or newer^) or set JAVA_HOME, then run this script again.
    exit /b 1
)

set "BUILD=%TEMP%\render-autos-%RANDOM%%RANDOM%"
mkdir "%BUILD%"
if errorlevel 1 (
    echo Cannot create the temporary folder %BUILD%.
    exit /b 1
)

"%JAVAC%" -encoding UTF-8 -nowarn -sourcepath "%SRC%" -d "%BUILD%" "%MAIN_FILE%"
if errorlevel 1 goto failed
"%JAVA%" -cp "%BUILD%" %MAIN_CLASS% "%OUT_DIR%"
if errorlevel 1 goto failed

rmdir /s /q "%BUILD%"
exit /b 0

:failed
rmdir /s /q "%BUILD%"
exit /b 1
