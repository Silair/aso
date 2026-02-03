#ifndef KVASERBASE_H
#define KVASERBASE_H

//C system headers
#include <stdlib.h>

//C++ standard library headers
#include <iostream>
#include <cstring>

//other libraries' headers
#include "canlib.h"

//project's headers
#include "CANMessage.h"

class KvaserBase
{
    //enum ErrorType { INIT_ERROR = 1, CONNECT_ERROR = 2, DISCONNECT_ERROR = 3, SEND_MESSAGE_ERROR = 4, RELEASE_ERROR = 5 };
public:
    KvaserBase();
    KvaserBase(const KvaserBase& kvaser);
    KvaserBase(int channel_number, int canBitRate = canBITRATE_500K);
    KvaserBase(const char* sSerialNumber, int canBitRate = canBITRATE_500K);
    ~KvaserBase();
    int canInit(int channel_number, int canBitRate = canBITRATE_500K);
    int canInit(const char* sSerialNumber, int canBitRate = canBITRATE_500K);
    int searchKvaser(const char* sSerialNumber);
    int canRelease();
    int canSend(CANMessage& Tx);
    void canSendQuick(CANMessage& Tx);
    int canReceive(CANMessage& Rx);
    int checkStatus(const std::string& id);

    const int& ChannelNumber() const
    {
        return channel_number;
    }

    const canHandle& Handle() const
    {
        return handle;
    }

protected:
    int channel_number = -1;
    canHandle handle;
    canStatus status;
    CANMessage Tx = { 0x000,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00} };
    CANMessage Rx = { 0x000,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00} };
};

#endif // !KVASERBASE_H


