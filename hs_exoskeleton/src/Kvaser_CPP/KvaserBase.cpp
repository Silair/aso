#include "KvaserBase.h"
#include <cstdint>

/*构造*/
KvaserBase::KvaserBase()
{
}
KvaserBase::KvaserBase(const KvaserBase& kvaser) :channel_number(kvaser.ChannelNumber()), handle(kvaser.Handle())
{
}
KvaserBase::KvaserBase(int channel_number, int canBitRate)
{
    canInit(channel_number, canBitRate);
}
KvaserBase::KvaserBase(const char* sSerialNumber, int canBitRate)
{
    canInit(sSerialNumber, canBitRate);
}
/*析构*/
KvaserBase::~KvaserBase()
{
    if (channel_number != -1)canRelease();
}
/*初始化can通道*/
int KvaserBase::canInit(int channel_number, int canBitRate)
{
    this->channel_number = channel_number;
    // 初始化 can 驱动（重复调用只会调用一次）
    canInitializeLibrary();
    // 打开 can 通道
    handle = canOpenChannel(channel_number, canOPEN_ACCEPT_VIRTUAL);
    if (handle < 0)
    {
        std::cout << "can Open Channel error!" << std::endl;
        return EXIT_FAILURE;
    }
    // 若发送消息时出现 TimeOut 错误，调整波特率（一般是 canBITRATE_500K）
    status = canSetBusParams(handle, canBitRate, 0, 0, 0, 0, 0);
    if (checkStatus("canSetBusParams"))
    {
        std::cout << "can " << channel_number << " Set Bus Params error!\n";
        return EXIT_FAILURE;
    }
    // 打开 can 总线
    status = canBusOn(handle);
    if (checkStatus("canBusOn"))
    {
        std::cout << "can Bus On error!" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "can " << channel_number << " initialized!\n";
    return EXIT_SUCCESS;
}
int KvaserBase::canInit(const char* sSerialNumber, int canBitRate)
{
    return canInit(searchKvaser(sSerialNumber), canBitRate);
}
/*检索kvaser*/
int KvaserBase::searchKvaser(const char* sSerialNumber)
{
    // 初始化 can 驱动（重复调用只会调用一次）
    canInitializeLibrary();
    // 检索可用can通道数
    int channelCount = 0;
    canGetNumberOfChannels(&channelCount);
    if (channelCount == 0)
    {
        std::cout << "No CAN channel available!\n";
        return -1;
    }
    // 查找序列号对应的can通道
    uint32_t quad[2] = { 0 };
    uint32_t SerialNumber = atoi(sSerialNumber);
    for (int channel_number = 0; channel_number < channelCount; channel_number++)
    {
        canGetChannelData(channel_number, canCHANNELDATA_CARD_SERIAL_NO, quad, sizeof(quad));
        if (quad[0] == SerialNumber)
        {
            std::cout << "Kvaser S/N: " << SerialNumber << " channel " << channel_number << '\n';
            return channel_number;
        }
    }
    
    std::cout << "Kvaser S/N: " << SerialNumber << " channel " << channel_number << '\n';
    return -1;
}
/*关闭 can 通道*/
int KvaserBase::canRelease()
{
    status = canBusOff(handle);
    if (checkStatus("canBusOff"))
    {
        return EXIT_FAILURE;
    }
    status = canClose(handle);
    if (checkStatus("canClose"))
    {
        return EXIT_FAILURE;
    }
    std::cout << "can released!" << std::endl;
    return EXIT_SUCCESS;
}
/*发送 can 消息*/
int KvaserBase::canSend(CANMessage& Tx)
{
    status = canWriteWait(handle, Tx.COB_ID, Tx.Byte, Tx.DLC, 0, 100);
    if (checkStatus("canWriteWait"))
    {
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
void KvaserBase::canSendQuick(CANMessage& Tx)
{
    canWrite(handle, Tx.COB_ID, Tx.Byte, Tx.DLC, 0);
}
/*读取 can 通道返回数据*/
int KvaserBase::canReceive(CANMessage& Rx)
{
    if (canReadSpecific(handle, Rx.COB_ID, Rx.Byte, &(Rx.DLC), NULL, NULL) == canOK)
        return EXIT_SUCCESS;
    else
        return EXIT_FAILURE;
}
/*检查 can 通道状态*/
int KvaserBase::checkStatus(const std::string& id)
{
    if (status != canOK)
    {
        char buf[50];
        buf[0] = '\0';
        canGetErrorText(status, buf, sizeof(buf));
        std::cout << id << ": failed, stat = " << (int)status << " info: " << buf << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}