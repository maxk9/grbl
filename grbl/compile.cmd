@echo off
SET "ToAdd=C:\Users\mk\AppData\Local\Arduino15\packages\arduino\tools\avr-gcc\7.3.0-atmel3.6.1-arduino7\avr\bin;"
SET "ToAdd=%ToAdd%;C:\Users\mk\AppData\Local\Arduino15\packages\arduino\tools\avr-gcc\7.3.0-atmel3.6.1-arduino7\bin;"
SET "ToAdd=%ToAdd%;C:\Users\mk\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.12\system\CMSIS\Examples\cmsis_example\gcc_atmel;"
SET "PATH=%PATH%;%ToAdd%"

@echo 
avr-g++.exe --version
@echo 

cd /d %~dp0

@make %1


pause
