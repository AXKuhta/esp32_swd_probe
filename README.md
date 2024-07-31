```sh
openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 200" -c "program firmware.elf verify reset exit"
```
