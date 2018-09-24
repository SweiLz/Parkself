#include <dynamixel.h>

//-------------------------------------------------------------------------------------------------------------------------------
//            Private Methods
//-------------------------------------------------------------------------------------------------------------------------------
void Dynamixel::writePacket(void)
{
    if (dynamixelSerial->readable())
        while (dynamixelSerial->readable())
            dynamixelSerial->getc();
    dynamixelDi->write(1);
    dynamixelSerial->putc(HEADER);
    dynamixelSerial->putc(HEADER);
    dynamixelSerial->putc(Instruction_Packet[0]);
    dynamixelSerial->putc(Instruction_Packet[1]);
    for (uint8_t i = 0; i < Instruction_Packet[1]; i++)
    {
        dynamixelSerial->putc(Instruction_Packet[2 + i]);
    }
    wait_us((Instruction_Packet[1] + 4) * 7);
    dynamixelDi->write(0);
}

unsigned int Dynamixel::readPacket(void)
{
    // wait_us(250);
    // uint8_t Counter = 0;
    // uint8_t InBuff[20];
    // uint8_t i = 0, j = 0, RxState = 0;
    // Status_Packet[0] = 0;
    // Status_Packet[1] = 0;
    // Status_Packet[2] = 0;
    // Status_Packet[3] = 0;
    // timer.start();
    // int bytes = 2;
    // int timeout = 0;
    // int plen = 0;
    // // while (timer.read_ms() < 3000)
    // // {
    // //     if (dynamixelSerial->readable())
    // //     {
    // //         InBuff[plen] = dynamixelSerial->getc();
    // //         plen++;
    // //     }
    // // }
    // while ((timeout < ((6 + bytes) * 10000)) && (plen < (6 + bytes)))
    // {

    //     if (dynamixelSerial->readable())
    //     {
    //         InBuff[plen] = dynamixelSerial->getc();
    //         plen++;
    //         timeout = 0;
    //     }

    //     // wait for the bit period
    //     wait_us(1);
    //     timeout++;
    // }
    // timer.stop();
    // for (int i = 0; i < plen; i++)
    // {
    //     Status_Packet[i] = InBuff[i];
    //     printf("0x%X,", Status_Packet[i]);
    // }
    // printf("\r\n");
    return 0x01;
}
//-------------------------------------------------------------------------------------------------------------------------------
// Public Methods
//-------------------------------------------------------------------------------------------------------------------------------
Dynamixel::Dynamixel(PinName tx, PinName rx, int baud, PinName di)
{
    dynamixelSerial = new Serial(tx, rx);
    dynamixelSerial->baud(baud);
    dynamixelDi = new DigitalOut(di);
    dynamixelDi->write(0);
}

Dynamixel::~Dynamixel(void)
{
    if (dynamixelSerial != NULL)
        delete dynamixelSerial;
}

void Dynamixel::reset(uint8_t ID)
{
    Instruction_Packet[0] = ID;
    Instruction_Packet[1] = RESET_LENGTH;
    Instruction_Packet[2] = COMMAND_RESET;
    Instruction_Packet[3] = ~(ID + RESET_LENGTH + COMMAND_RESET);
    writePacket();
}
void Dynamixel::setID(uint8_t ID, uint8_t nID)
{
    Instruction_Packet[0] = ID;
    Instruction_Packet[1] = SET_ID_LENGTH;
    Instruction_Packet[2] = COMMAND_WRITE_DATA;
    Instruction_Packet[3] = EEPROM_ID;
    Instruction_Packet[4] = nID;
    Instruction_Packet[5] = ~(ID + SET_ID_LENGTH + COMMAND_WRITE_DATA + EEPROM_ID + nID);
    writePacket();
}
void Dynamixel::setBaudrate(uint8_t ID, long Baud)
{
    Instruction_Packet[0] = ID;
    Instruction_Packet[1] = SET_BD_LENGTH;
    Instruction_Packet[2] = COMMAND_WRITE_DATA;
    Instruction_Packet[3] = EEPROM_BAUD_RATE;
    // Instruction_Packet[4] = (uint8_t)((2000000 / Baud) - 1);
    Instruction_Packet[4] = 0x22;
    // Instruction_Packet[5] = ~(ID + SET_BD_LENGTH + COMMAND_WRITE_DATA + EEPROM_BAUD_RATE + (uint8_t)((2000000 / Baud) - 1));
    Instruction_Packet[5] = ~(ID + SET_BD_LENGTH + COMMAND_WRITE_DATA + EEPROM_BAUD_RATE + 0x22);
    writePacket();
}
void Dynamixel::setMode(uint8_t ID, uint8_t Mode, uint16_t CW_limit, uint16_t CCW_limit)
{
    uint8_t L_CW = (uint8_t)(CW_limit & 0xFF);
    uint8_t H_CW = (uint8_t)(CW_limit >> 8);
    uint8_t L_CCW = (uint8_t)(CCW_limit & 0xFF);
    uint8_t H_CCW = (uint8_t)(CCW_limit >> 8);
    Instruction_Packet[0] = ID;
    Instruction_Packet[1] = SET_MODE_LENGTH;
    Instruction_Packet[2] = COMMAND_WRITE_DATA;
    Instruction_Packet[3] = EEPROM_CW_ANGLE_LIMIT_L;
    Instruction_Packet[4] = L_CW;
    Instruction_Packet[5] = H_CW;
    Instruction_Packet[6] = L_CCW;
    Instruction_Packet[7] = H_CCW;
    Instruction_Packet[8] = ~(ID + SET_MODE_LENGTH + COMMAND_WRITE_DATA + EEPROM_CW_ANGLE_LIMIT_L + L_CW + H_CW + L_CCW + H_CCW);
    writePacket();
}

void Dynamixel::setLed(uint8_t ID, bool State)
{
    Instruction_Packet[0] = ID;
    Instruction_Packet[1] = LED_LENGTH;
    Instruction_Packet[2] = COMMAND_WRITE_DATA;
    Instruction_Packet[3] = RAM_LED;
    Instruction_Packet[4] = State;
    Instruction_Packet[5] = ~(ID + LED_LENGTH + COMMAND_WRITE_DATA + RAM_LED + State);
    writePacket();
}

void Dynamixel::setPosition(uint8_t ID, uint16_t Position, uint16_t Speed = 0)
{
    uint8_t L_Position = (uint8_t)(Position & 0xFF);
    uint8_t H_Position = (uint8_t)(Position >> 8);
    uint8_t L_Speed = (uint8_t)(Speed & 0xFF);
    uint8_t H_Speed = (uint8_t)(Speed >> 8);
    Instruction_Packet[0] = ID;
    Instruction_Packet[1] = SERVO_GOAL_LENGTH;
    Instruction_Packet[2] = COMMAND_WRITE_DATA;
    Instruction_Packet[3] = RAM_GOAL_POSITION_L;
    Instruction_Packet[4] = L_Position;
    Instruction_Packet[5] = H_Position;
    Instruction_Packet[6] = L_Speed;
    Instruction_Packet[7] = H_Speed;
    Instruction_Packet[8] = ~(ID + SERVO_GOAL_LENGTH + COMMAND_WRITE_DATA + RAM_GOAL_POSITION_L + L_Position + H_Position + L_Speed + H_Speed);
    writePacket();
}

void Dynamixel::setWheelSpeed(uint8_t ID, bool Direction, uint16_t Speed)
{
    uint8_t L_Speed = (uint8_t)(Speed & 0xFF);
    uint8_t H_Speed = (uint8_t)(Direction ? (Speed >> 8) : (Speed >> 8) | 0x04);
    Instruction_Packet[0] = ID;
    Instruction_Packet[1] = WHEEL_LENGTH;
    Instruction_Packet[2] = COMMAND_WRITE_DATA;
    Instruction_Packet[3] = RAM_GOAL_SPEED_L;
    Instruction_Packet[4] = L_Speed;
    Instruction_Packet[5] = H_Speed;
    Instruction_Packet[6] = ~(ID + WHEEL_LENGTH + COMMAND_WRITE_DATA + RAM_GOAL_POSITION_L + L_Speed + H_Speed);
    writePacket();
}

void Dynamixel::setWheel3Speed(uint8_t ID[], bool Direction[], uint16_t Speed[])
{
    uint8_t L_Speed[3], H_Speed[3];
    L_Speed[0] = (uint8_t)(Speed[0] & 0xFF);
    H_Speed[0] = (uint8_t)(Direction[0] ? (Speed[0] >> 8) : (Speed[0] >> 8) | 0x04);
    L_Speed[1] = (uint8_t)(Speed[1] & 0xFF);
    H_Speed[1] = (uint8_t)(Direction[1] ? (Speed[1] >> 8) : (Speed[1] >> 8) | 0x04);
    L_Speed[2] = (uint8_t)(Speed[2] & 0xFF);
    H_Speed[2] = (uint8_t)(Direction[2] ? (Speed[2] >> 8) : (Speed[2] >> 8) | 0x04);

    Instruction_Packet[0] = 0xFE;
    Instruction_Packet[1] = SYNC_LOAD_LENGTH;
    Instruction_Packet[2] = COMMAND_SYNC_WRITE;
    Instruction_Packet[3] = RAM_GOAL_SPEED_L;
    Instruction_Packet[4] = SYNC_DATA_LENGTH;
    Instruction_Packet[5] = ID[0];
    Instruction_Packet[6] = L_Speed[0];
    Instruction_Packet[7] = H_Speed[0];
    Instruction_Packet[8] = ID[1];
    Instruction_Packet[9] = L_Speed[1];
    Instruction_Packet[10] = H_Speed[1];
    Instruction_Packet[11] = ID[2];
    Instruction_Packet[12] = L_Speed[2];
    Instruction_Packet[13] = H_Speed[2];
    Instruction_Packet[14] = ~(0xFE + SYNC_LOAD_LENGTH + COMMAND_SYNC_WRITE + RAM_GOAL_SPEED_L + SYNC_DATA_LENGTH + ID[0] + L_Speed[0] + H_Speed[0] + ID[1] + L_Speed[1] + H_Speed[1] + ID[2] + L_Speed[2] + H_Speed[2]);
    writePacket();
}
void Dynamixel::setWheelPreload(uint8_t ID, bool Direction, uint8_t Speed)
{
    uint8_t L_Speed, H_Speed;
    L_Speed = (uint8_t)(Speed & 0xFF);
    H_Speed = (uint8_t)(Direction ? (Speed >> 8) : (Speed >> 8) | 0x04);
    Instruction_Packet[0] = ID;
    Instruction_Packet[1] = WHEEL_LENGTH;
    Instruction_Packet[2] = COMMAND_REG_WRITE_DATA;
    Instruction_Packet[3] = RAM_GOAL_SPEED_L;
    Instruction_Packet[4] = L_Speed;
    Instruction_Packet[5] = H_Speed;
    Instruction_Packet[6] = ~(ID + WHEEL_LENGTH + COMMAND_REG_WRITE_DATA + RAM_GOAL_POSITION_L + L_Speed + H_Speed);
    writePacket();
}

void Dynamixel::action(uint8_t ID)
{
    Instruction_Packet[0] = ID;
    Instruction_Packet[1] = RESET_LENGTH;
    Instruction_Packet[2] = COMMAND_ACTION;
    Instruction_Packet[3] = ~(ID + ACTION_LENGTH + COMMAND_ACTION);
    writePacket();
}
// unsigned int Dynamixel::ping(uint8_t ID)
// {
//     Instruction_Packet[0] = ID;
//     Instruction_Packet[1] = PING_LENGTH;
//     Instruction_Packet[2] = COMMAND_PING;
//     Instruction_Packet[3] = ~(ID + PING_LENGTH + COMMAND_PING);
//     transmitInstructionPacket();
//     // if ((ID == 0xFE) || (Status_Return_Value != ALL))
//     //     return 0x00;
//     // else
//     // {
//     readStatusPacket();
//     return 0x01;
//     // if (Status_Packet[2] == 0)
//     // return (Status_Packet[0]);
//     // else
//     // return (Status_Packet[2] | 0xF000);
//     // }
// }
// unsigned int Dynamixel::getTemperature(uint8_t ID)
// {
//     Instruction_Packet[0] = ID;
//     Instruction_Packet[1] = READ_TEMP_LENGTH;
//     Instruction_Packet[2] = COMMAND_READ_DATA;
//     Instruction_Packet[3] = RAM_PRESENT_TEMPERATURE;
//     Instruction_Packet[4] = READ_ONE_BYTE_LENGTH;
//     Instruction_Packet[5] = ~(ID + READ_TEMP_LENGTH + COMMAND_READ_DATA + RAM_PRESENT_TEMPERATURE + READ_ONE_BYTE_LENGTH);

//     transmitInstructionPacket();
//     return readStatusPacket();
//     // return Instruction_Packet[5];
//     /*
//     if (Status_Packet[2] == 0)
//     { // If there is no status packet error return value
//         // return Status_Packet[3];
//         return 0;
//     }
//     else
//     {
//         return 1;
//         // return (Status_Packet[2] | 0xF000); // If there is a error Returns error value
//     }*/
// }
