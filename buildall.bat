@echo off
setlocal enabledelayedexpansion

REM Create buildall directory if it doesn't exist
if not exist buildall mkdir buildall
del /Q buildall\*

REM List of boards to build
set "boards=console60k console138k mega60k mega138k primer25k"

for %%b in (%boards%) do (
    echo Building for board: %%b
    
    REM Clean previous build
    make clean
    
    REM Set board environment variable and build
    set "TANG_BOARD=%%b"
    make
    
    if !errorlevel! equ 0 (
        echo Build successful for %%b
        
        REM Copy and rename the binary files
        copy /Y build\build_out\tangcore_bl616.bin buildall\tangcore_%%b.bin
        if "%%b"=="console60k" (
            copy /Y bl616_fpga_partner_60kConsole.bin buildall\bl616_fpga_partner_%%b.bin
        ) else if "%%b"=="console138k" (
            copy /Y bl616_fpga_partner_60kConsole.bin buildall\bl616_fpga_partner_%%b.bin
        ) else if "%%b"=="mega60k" (
            copy /Y bl616_fpga_partner_138k60kNeoDock.bin buildall\bl616_fpga_partner_%%b.bin
        ) else if "%%b"=="mega138k" (
            copy /Y bl616_fpga_partner_138k60kNeoDock.bin buildall\bl616_fpga_partner_%%b.bin
        ) else if "%%b"=="primer25k" (
            copy /Y bl616_fpga_partner_25kDock.bin buildall\bl616_fpga_partner_%%b.bin
        ) else if "%%b"=="nano20k" (
            copy /Y bl616_fpga_partner_20kNano.bin buildall\bl616_fpga_partner_%%b.bin
        )
        
        REM Process flash.ini
        powershell -Command "(Get-Content flash.ini) -replace 'bl616_fpga_partner.bin', 'bl616_fpga_partner_%%b.bin' -replace 'tangcore.bin', 'tangcore_%%b.bin' | Set-Content buildall\flash_%%b.ini"
    ) else (
        echo Build failed for %%b
    )
)

REM List contents of buildall directory
echo.
echo Contents of buildall directory:
dir /B buildall\

endlocal

