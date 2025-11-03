# usb_serial_echo

Minimal USB CDC echo utility for sanity-checking host connectivity. It relies
on the Pico SDK’s built-in USB stdio backend, so the board enumerates with the
same descriptors as the official examples. Once flashed you can connect from a
host with:

```sh
screen /dev/tty.usbmodemXXXX 115200
```

Every character typed is echoed back and a prompt (`> `) is reprinted on
newlines. The onboard LED blinks quickly when the USB link is active and
slower otherwise, giving a quick visual cue that enumeration succeeded.
