# loco-controller-v1a

The Loco Controller is designed to offer a simple upgrade of Dapol and other locomotives to battery power and wireless control. It is highly integrated, incorporating an ESP32-C6 dual core MCU (microcontroller Unit) module together with a motor driver, voltage booster, battery charger and input power conditioner all on a single double sided PCB. 

The controller is designed as an exact replacement for the factory fitted DCC decoder adaptor installed in newer Dapol locomotives and installation consists of sliding out the factory PCB and replacing it with the controller. On the 58XX and B4 models the battery also slides in underneath the controller and is connected directly to the controller PCB.

For non Dapol locomotives there are a range of signals and power sources available on 2.54mm spacing pin headers including motor drive and LED outputs. In these cases though some disassembly of the locomotive body and soldering will be required.

Although designed primarily as a controller for motorised devices the device can also function as a generic wireless interface for signals, turnouts and other accessories. In such a ‘fixed’ application the unit can operate without a connected battery being powered solely by externally connected power sources and can drive multiple LED or servo outputs.

The controller implements a Wi-Fi access point and Web server that serves a page of locomotive specific controls to any standard web browser on a phone or tablet 

The initial hardware and firmware release supports:-

• Connector compatible with Dapol 0 Gauge 58XX and B4 locomotives

• Aux connector for other models with power, motor, LED and servo drive

• Integral voltage boost from VBAT to 12 V for the motor at 1 A

• Integral battery charger with power in via track or USB and 2mm JST socket for a 1S Li-Po battery

• Track power input is compatible with DCC and can be up to +-50 VDC with protection against inductive
 spikes
• Battery voltage and USB voltage can be monitored in firmware

• Links provided to isolate track power and the battery

• Charging is indicated by firebox LEDs in Dapol models and also in UI

• Multiple levels of over-current/over-voltage/over-temperature protection

• Integral Wi-Fi access point and Web server

• Control by standard Web browsers (up to four simultaneous connections)

• Firmware is updatable via USB connection

