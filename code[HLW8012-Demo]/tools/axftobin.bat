@echo off
echo AXF to BIN file generation ...

"D:\SOFTWARE\Keil5\keil5_Core\ARM\ARMCC\bin\fromelf.exe" --bin --output ..\MDK-ARM\stm32fL471RE\stm32fL471RE.bin ..\MDK-ARM\stm32fL471RE\stm32fL471RE.axf 

echo AXF to BIN file generation completed.

