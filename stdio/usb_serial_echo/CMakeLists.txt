if (TARGET tinyusb_device)
    add_executable(usb_serial_echo
            usb_serial_echo.c
    )

    target_link_libraries(usb_serial_echo PRIVATE
            pico_stdlib
            pico_stdio_usb
    )

    pico_enable_stdio_usb(usb_serial_echo 1)
    pico_enable_stdio_uart(usb_serial_echo 0)

    pico_add_extra_outputs(usb_serial_echo)
endif ()
