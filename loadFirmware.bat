openocd -f board/stm32f429discovery.cfg -c "program firmware.elf verify reset exit"
