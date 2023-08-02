# pico_ServoCAN_v1.0
RasPi Pico RP2040 Sevo CAN Microcontroller Board

![RP2040_pico_ServoCAN Gimp picture](https://github.com/jebradshaw/pico_ServoCAN_v1.0/assets/5246863/c29b8548-cf97-40f7-bbad-c00d1fd12476)

This RasPi Pico RP2040 microcontroller board contains an MCP2515 SPI CAN bus transceiver and driver electronics, GPIO accessible ports, an I2C port with STEMMA and JST-XH connectors, two power FET driver ports, an on board 5 volt 2 amp DC-DC switching power supply, a TTL level serial UART, and jumper options.

Note that on version 1.0, the MCP2515 18-pin DIP package needs pin 17 (!RESET) tied high, usuaally through a solder jumper from pin 17 to pin 18 (Vcc) on the underside of the PCB.
  Also pin 8 (STBY) of the MCP2561 CAN transceiver should be tied LOW to ground through an external wire jumper.
  
![Solder_jumpers_backOfBoard_marked](https://github.com/jebradshaw/pico_ServoCAN_v1.0/assets/5246863/0b774c0b-8bcd-45a1-a45e-b61d4d0a5215)

![KiCAD_PCB_Artwork](https://github.com/jebradshaw/pico_ServoCAN_v1.0/assets/5246863/dace43ad-c81a-4756-9326-021f57ee2c05)

![3D_Render_of_PCB](https://github.com/jebradshaw/pico_ServoCAN_v1.0/assets/5246863/a911d34e-cfc2-46b8-be41-5ea7473034b4)

[pico_Kingfisher_2023.pdf](https://github.com/jebradshaw/pico_ServoCAN_v1.0/files/12242137/pico_Kingfisher_2023.pdf)
PDF Schematic above


