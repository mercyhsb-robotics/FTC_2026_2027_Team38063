@echo off
rem Prints the table of start-pose shortcuts: for each family of autonomous route, where to set the robot up (rule G304) to save the most time,
rem what the placement margin costs and how many seconds it saves against the standard start (tile B1 or C1, front on the audience wall).
rem
rem It compiles ONLY the pure-Java sources (BioBuzzConstants and the autoshoot core, routes and render packages) with plain javac into a temporary folder,
rem runs StartShortcuts and deletes the folder again. No Gradle, no Android SDK and no robot are needed; any JDK 8 or newer will do. It takes a few minutes:
rem every family of routes is tried from about 37 thousand legal starts (a place every 0.25 in along the three own walls, every 5 degrees).
rem
rem Usage:  tools\start-shortcuts.bat [family id]      (one of shoot_park, shoot_garden, shoot_flower, partner_park, partner_flower, leave_park; default: all)
setlocal EnableExtensions

rem always work from the repository root, wherever the script is started from
cd /d "%~dp0.."
if errorlevel 1 (
    echo Cannot change to the repository root.
    exit /b 1
)

set "SRC=TeamCode\src\main\java"
set "MAIN_CLASS=org.firstinspires.ftc.teamcode.autoshoot.routes.StartShortcuts"
set "MAIN_FILE=%SRC%\org\firstinspires\ftc\teamcode\autoshoot\routes\StartShortcuts.java"

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

set "BUILD=%TEMP%\start-shortcuts-%RANDOM%%RANDOM%"
mkdir "%BUILD%"
if errorlevel 1 (
    echo Cannot create the temporary folder %BUILD%.
    exit /b 1
)

"%JAVAC%" -encoding UTF-8 -nowarn -sourcepath "%SRC%" -d "%BUILD%" "%MAIN_FILE%"
if errorlevel 1 goto failed
"%JAVA%" -cp "%BUILD%" %MAIN_CLASS% %*
if errorlevel 1 goto failed

rmdir /s /q "%BUILD%"
exit /b 0

:failed
rmdir /s /q "%BUILD%"
exit /b 1
