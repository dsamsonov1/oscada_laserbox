#ifndef EPILASER_H
#define EPILASER_H

#include <string>
#include <cstring>
#include <chrono>
#include "ftd2xx.h"
#include <utility>
#include <vector>
#include <unistd.h>
//#include <jsoncpp/json/json.h>

#define MAX_DEVICES	16 //max devices to scan
#define PACKET_OUT_LENGTH 6 //[Bytes]
#define PACKET_IN_LENGTH 25 //[Bytes]
#define GROUP_LENGTH 8 //[Bytes]
#define WORD_COUNT 4
#define WORD_LENGTH 2 //[Bytes]
#define SERIAL_LENGTH 16 //[Bytes]
#define BAUD_RATE 256000 //check later
#define UPDATE_PERIOD 500 //[ms]
#define MAX_POINTS 10000

struct Point{
    short none[WORD_COUNT];
    short red[WORD_COUNT];
    short blue[WORD_COUNT];
};

class EpiLaser {
public:
//    EpiLaser() = delete;
//    EpiLaser (std::string p_serialNo): m_serialNo(std::move(p_serialNo)) {};
    ~EpiLaser() ;
//    EpiLaser (const std::string & deviceInstance, const std::string & deviceType);

    bool initModule (const std::string &p_serialNo);
    bool destroyModule ();
    bool getState();
    void cyclicFunc();
    unsigned char Crc8(unsigned char *pcBlock, unsigned int len);
    void dumpBuffer(unsigned char *buffer, int elements);
    std::string exec(const char *cmd);

//protected:
//    bool handleGetData (Json::Value & resp);
    bool handleGetData (std::vector<Point> &p_points);
    bool handleStart();
    bool handleStop();
    bool handleClear();

private:
    bool m_online = false;
    std::chrono::time_point<std::chrono::high_resolution_clock> m_tStart;
    bool sendPacket(unsigned char* data);
    unsigned char m_packetStart[PACKET_OUT_LENGTH] = {0x01, 0x01, 0x00, 0x00, 0x00, 0xdb};
    unsigned char m_packetStop[PACKET_OUT_LENGTH] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x40};
//    char m_serialNo[SERIAL_LENGTH] = "AC30P4O1";
    std::string m_serialNo;
    const size_t m_realSerialLength = 8;
    const char* m_unloadFTDI = "rmmod ftdi_sio";
    const char* m_unloadUsbserial = "rmmod usbserial";
    FT_HANDLE m_ftHandle;
    unsigned char* m_readBuffer = NULL;
    unsigned char* m_tailBuffer = NULL;
    unsigned int m_tailLength = 0;
    FT_STATUS	m_ftStatus;
    union bytesToInt {
        WORD intVal;
        char chars[WORD_LENGTH];
    } m_converter;
    std::vector<Point> m_points;
    //std::vector<Point> m_avgPoints;
};


#endif //EPILASER_H
