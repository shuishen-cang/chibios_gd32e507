
flash: all
	GD_Link_CLI.exe -commandfile gd.gdlink

flash2: build/ch.elf
	cp build/ch.elf /mnt/d/WSL/share/
	openocd.exe -f interface/cmsis-dap.cfg -f target/gd32e50x.cfg -c "program D:/WSL/share/ch.elf reset exit"
