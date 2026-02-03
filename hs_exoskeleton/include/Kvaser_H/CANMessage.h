#ifndef CAN_MESSAGE_H
#define CAN_MESSAGE_H

//C system headers

//C++ standard library headers
#include <cstdint>

//other libraries' headers

//project's headers

using std::int16_t;
using std::int32_t;
using std::uint8_t;
using std::uint16_t;

struct CANMessage
{
    long COB_ID;
    unsigned int DLC;
    uint8_t Byte[8];//�޷���8λ��������

    void int2byte(int32_t value, int offset = 0)
    {
        Byte[3 + offset] = value >> 24;
        Byte[2 + offset] = (value & 0x00FF0000) >> 16;
        Byte[1 + offset] = (value & 0x0000FF00) >> 8;
        Byte[0 + offset] = value & 0x000000FF;
    }

    void short2byte(int16_t value, int offset = 0)
    {
        Byte[1 + offset] = value >> 8;
        Byte[0 + offset] = value & 0x00FF;
    }

    void short2byteBigEndian(int16_t value, int offset = 0)
    {
        Byte[0 + offset] = value >> 8;
        Byte[1 + offset] = value & 0x00FF;
    }

    void float2byte(float fValue, int offset = 0)
    {
        int32_t iValue = *(int32_t*)&fValue;
        Byte[3 + offset] = (iValue >> 24);
        Byte[2 + offset] = ((iValue << 8) >> 24);
        Byte[1 + offset] = ((iValue << 16) >> 24);
        Byte[0 + offset] = ((iValue << 24) >> 24);
    }

    int byte2int(int offset = 0)
    {
        return (Byte[3 + offset] << 24 | Byte[2 + offset] << 16 | Byte[1 + offset] << 8 | Byte[0 + offset]);
    }

    int16_t byte2shortBigEndian(int offset = 0)
    {
        return (int16_t)((Byte[0 + offset] << 8) | (Byte[1 + offset]));
    }

    uint16_t byte2ushortBigEndian(int offset = 0)
    {
        return (uint16_t)((Byte[0 + offset] << 8) | (Byte[1 + offset]));
    }
};

#endif // !CAN_MESSAGE_H