# pico_ServoCAN_v1.0
RasPi Pico RP2040 Sevo CAN Microcontroller Board

This RasPi Pico RP2040 microcontroller board contains an MCP2515 SPI CAN bus transceiver and driver electronics, GPIO accessible ports, an I2C port with STEMMA and JST-XH connectors, two power FET driver ports, an on board 5 volt 2 amp DC-DC switching power supply, a TTL level serial UART, and jumper options.

Note that on version 1.0, the MCP2515 18-pin DIP package needs pin 17 (!RESET) tied high, usuaally through a solder jumper from pin 17 to pin 18 (Vcc) on the underside of the PCB.
  Also pin 8 (STBY) of the MCP2561 CAN transceiver shoud be tied LOW through an external wire jumper.

