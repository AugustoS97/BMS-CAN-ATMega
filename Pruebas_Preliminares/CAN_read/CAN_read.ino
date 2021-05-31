#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
struct can_frame canMsg1;
MCP2515 mcp2515(10);


void setup() {
  canMsg1.can_id  = 0x0F2;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x00;
  canMsg1.data[1] = 0x45;
  canMsg1.data[2] = 0x26;
  canMsg1.data[3] = 0xFD;
  canMsg1.data[4] = 0x17;
  canMsg1.data[5] = 0x4E;
  canMsg1.data[6] = 0xB5;
  canMsg1.data[7] = 0x81;
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);

  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
  
  //mcp2515.setFilterMask(MCP2515::MASK0, false, 0x7FF); //0x07FF0000 means accept any message with any value in id (for 11bit ID)
  //mcp2515.setFilterMask(MCP2515::MASK1, false, 0x7FF);

  //mcp2515.setFilter(MCP2515::RXF0, false, 9);
  //mcp2515.setFilter(MCP2515::RXF1, false, 0x614);
  //mcp2515.setFilter(MCP2515::RXF2, false, 9);
  //mcp2515.setFilter(MCP2515::RXF3, false, 9);
  //mcp2515.setFilter(MCP2515::RXF4, false, 9);

    mcp2515.setNormalMode();
}

void loop() {
  static uint8_t i=0;
  i +=1;
  canMsg1.data[0] = i;
  mcp2515.sendMessage(&canMsg1);
  delay (10);
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print(canMsg.can_id, HEX); // print ID
    Serial.print(" "); 
    Serial.print(canMsg.can_dlc, HEX); // print DLC
    Serial.print(" ");
    
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data
      Serial.print(canMsg.data[i],HEX);
      Serial.print(" ");
    }

    Serial.println();      
  }
 
}
