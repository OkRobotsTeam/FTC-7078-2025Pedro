cd /D "%~dp0"

if [%1]==[-w] goto wipe
goto push
	
:wipe
ftc_http.exe -w 

:push

ftc_http.exe -u "..\TeamCode\src\main\java\pedroPathing\constants" 
ftc_http.exe -u "..\TeamCode\src\main\java\org\firstinspires\ftc\teamcode" -b


@ECHO Done
@IF %ERRORLEVEL% EQU 0 PowerShell [System.Media.SystemSounds]::Hand.Play()
@IF %ERRORLEVEL% NEQ 0 PowerShell [System.Media.SystemSounds]::Beep.Play()
