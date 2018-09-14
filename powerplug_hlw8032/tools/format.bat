
pushd .
AStyle.exe ..\Src\*.c --options=c.opt --recursive --exclude=..\Src\system_stm32f1xx.c
popd
 
