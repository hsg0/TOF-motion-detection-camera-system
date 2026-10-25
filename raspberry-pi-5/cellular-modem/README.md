# Cellular modem

Hardware: SIMCom SIM7600G-H, plugged into a Pi USB port. No wires to the Uno Q or the Pico.

The kernel shows serial ports `ttyUSB0` through `ttyUSB4` and a data interface, `cdc-wdm0` with network device `wwan0`.

| Software | Role |
|---|---|
| `qmi_wwan` and `option` | Kernel drivers |
| ModemManager | Modem control, including voice calls |
| NetworkManager | Data session |
| `modem-manager-gui` | Desktop window on the Pi |

Wi-Fi stays the preferred route while it is connected. The cellular route is configured with a worse metric, so it carries traffic when Wi-Fi is down. The provider name, the APN, and any phone number are not written in this tree.

Voice is a ModemManager command, not a feature of the dashboard. The modem does not show up as a USB sound card. Call audio is on the modem board's headset jack.
