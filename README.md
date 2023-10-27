# pico_ServoCAN_v1.0
RasPi Pico RP2040 Sevo CAN Microcontroller Board

![RP2040_pico_ServoCAN Gimp picture](https://github.com/jebradshaw/pico_ServoCAN_v1.0/assets/5246863/c29b8548-cf97-40f7-bbad-c00d1fd12476)

This RasPi Pico RP2040 microcontroller board contains an MCP2515 SPI CAN bus transceiver and driver electronics, GPIO accessible ports, an I2C port with STEMMA and JST-XH connectors, two power FET driver ports, an on board 5 volt 2 amp DC-DC switching power supply, a TTL level serial UART, and jumper options.

Note that on version 1.0, the MCP2515 18-pin DIP package needs pin 17 (!RESET) tied high, usuaally through a solder jumper from pin 17 to pin 18 (Vcc) on the underside of the PCB.
  Also pin 8 (STBY) of the MCP2561 CAN transceiver should be tied LOW to ground through an external wire jumper.
  
![Solder_jumpers_backOfBoard_Marked](https://github.com/jebradshaw/pico_ServoCAN_v1.0/assets/5246863/46214bf9-9015-4f3b-b776-d11d8ca13050)


Also note that the board was originally designed using an OKI 78SR Series 5V 1.5A regulator (U4), which are now very difficult to come by.  So the part has been replaced with a VR20S05 5 volt 2 amp regulator which comes in a slightly larger, sealed, package (available from Digikey - https://www.xppower.com/portals/0/pdfs/SF_VR20.pdf).  To get this regulator to fit on the board, C6 (a .1uF capacitor) should not be soldered in place.

![KiCAD_PCB_Artwork](https://github.com/jebradshaw/pico_ServoCAN_v1.0/assets/5246863/dace43ad-c81a-4756-9326-021f57ee2c05)

![3D_Render_of_PCB](https://github.com/jebradshaw/pico_ServoCAN_v1.0/assets/5246863/a911d34e-cfc2-46b8-be41-5ea7473034b4)

KiCAD 3D render of PCB

![Schamatic](https://github.com/jebradshaw/pico_ServoCAN_v1.0/assets/5246863/dc132d68-9685-4170-b8d4-9f60317bf44c)

Schematic (PDF format) - [pico_Kingfisher_2023.pdf](https://github.com/jebradshaw/pico_ServoCAN_v1.0/files/12242137/pico_Kingfisher_2023.pdf)


  If using Adafruits Circuit Python (which I like for the RP2040), copy the lib folder contents to the pico drive under .lib to include the CAN libraries for the CAN bus controller.  An example program to test the CAN transmit and receive functionality is also included.

  Also added is an Arduino (which I also really like) sketch for the CAN interface and servo outs running on both cores overclocked to 250MHz (Core Temp ~ 31 deg C).
