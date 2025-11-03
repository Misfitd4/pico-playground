add_executable(usb_serial_probe
        src/main.c
)

target_link_libraries(usb_serial_probe
        pico_stdlib
        m
)

pico_enable_stdio_usb(usb_serial_probe 1)
pico_enable_stdio_uart(usb_serial_probe 0)

pico_add_extra_outputs(usb_serial_probe)
