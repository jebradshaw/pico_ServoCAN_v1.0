import time
import board
import busio
from digitalio import DigitalInOut
from adafruit_mcp2515.canio import Timer
from adafruit_mcp2515.canio import Message, RemoteTransmissionRequest
from adafruit_mcp2515 import MCP2515 as CAN

#CAN node ID
NODE_ID = 0x2234ABCD

# declare SPI pins for CAN MCP2515
clk_pin = board.GP2
mosi_pin = board.GP3
miso_pin = board.GP4
cs_can_pin = board.GP5

cs_can = DigitalInOut(cs_can_pin)
cs_can.switch_to_output()
spi = busio.SPI(clk_pin, mosi_pin, miso_pin)
can_bus = CAN(spi, cs_can, baudrate = 500000)#, silent=True)

t = Timer(timeout=5)
next_message = None
message_num = 0
i = 0

can_tx_time_delay = .1 # tranmit CAN message check every .1 seconds
can_tx_tm_last = 0

can_rx_time_delay = .01 # receive CAN message check every .01 seconds
can_rx_tm_last = 0

tm_display_last = 0
while True:
    tm = time.monotonic()        
    
    if tm > can_tx_tm_last + can_tx_time_delay:
        can_tx_tm_last = tm
        dataCANtx = (int(tm*1000000000)).to_bytes(8,'big', signed=False)       
#       dataCANtx = (0x004534521300AA).to_bytes(8,'big', signed=False)
        messageCANtx = Message(id=0x1234ABCD, data = dataCANtx, extended=True)        
        try:
            send_success = can_bus.send(messageCANtx)            
            text_can_rx_str = 'Txid:' + str(hex(messageCANtx.id)) + ' '
            for i in range(len(messageCANtx.data)):                        
                text_can_rx_str = text_can_rx_str + '{:x} '.format(messageCANtx.data[i])
            print('{:.2f} '.format(tm) + ' ' + text_can_rx_str)
        except:
            print("Send failure - Check for attached device!")
    
    if tm > can_rx_tm_last + can_rx_time_delay:
        with can_bus.listen() as listener:
            message_count = listener.in_waiting()
            if message_count > 0:
                for _i in range(message_count):
                    msg = listener.receive()                    
                    text_can_rx_str = 'Rxid:' + str(hex(msg.id)) + ' '
                    for i in range(len(msg.data)):                        
                        text_can_rx_str = text_can_rx_str + '{:x} '.format(msg.data[i])
                    print('{:.2f} '.format(tm) + ' ' + text_can_rx_str)
                        