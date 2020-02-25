#include "epiLaser.h"

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <array>
#include <cmath>

unsigned char EpiLaser::Crc8(unsigned char *pcBlock, unsigned int len) {
    unsigned char crc = 0xFF;
    unsigned int i;
    while (len--) {
        crc ^= *pcBlock++;
        for (i = 0; i < 8; i++)
            crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    }
    return crc;
}

bool EpiLaser::sendPacket(unsigned char *data) {
    DWORD dwBytesWritten;
    m_ftStatus = FT_Write(m_ftHandle, data, PACKET_OUT_LENGTH, &dwBytesWritten);
    if (m_ftStatus != FT_OK) {
        printf("Error FT_Write(%d)\n", (int) m_ftStatus);
        return false;
    }
    if (dwBytesWritten != (DWORD) PACKET_OUT_LENGTH) {
        printf("FT_Write only wrote %d (of %d) bytes\n",
               (int) dwBytesWritten,
               PACKET_OUT_LENGTH);
        return false;
    }
    return true;
}

void EpiLaser::dumpBuffer(unsigned char *buffer, int elements) {
    int j;

    printf(" [");
    for (j = 0; j < elements; j++) {
        if (j > 0)
            printf(", ");
        printf("0x%02X", (unsigned int) buffer[j]);
    }
    printf("]\n");
}


bool EpiLaser::initModule(const std::string &p_serialNo) {

    m_serialNo = p_serialNo;

    m_tStart = std::chrono::high_resolution_clock::now();

    char cBufLD[MAX_DEVICES][SERIAL_LENGTH];
    char *pcBufLD[MAX_DEVICES + 1]; //mem leak?
    //wtf? nullification?
    for (int i = 0; i < MAX_DEVICES; i++) {
        pcBufLD[i] = cBufLD[i];
    }
    pcBufLD[MAX_DEVICES] = NULL;


    int iNumDevs = 0;
    m_ftStatus = FT_ListDevices(pcBufLD, &iNumDevs, FT_LIST_ALL | FT_OPEN_BY_SERIAL_NUMBER);
    if (m_ftStatus != FT_OK) {
        printf("Error: FT_ListDevices(%d)\n", (int) m_ftStatus);
        return false;
    }

    bool deviceFound = false;
    for (int i = 0; i < iNumDevs; i++) {
        printf("Found device No = %d, Serial Number = %s\n", i, cBufLD[i]);
        bool serialMatch = true;
        for (int j = 0; j < m_realSerialLength; j++) {
            if (cBufLD[i][j] != m_serialNo[j]) {
                serialMatch = false;
                break;
            }
        }
        if (serialMatch) {
            deviceFound = true;
            break;
        }
    }
    if (deviceFound) {
        std::cout << "LaserBox found." << std::endl;
    } else {
        std::cout << "ERROR LaserBox not found." << std::endl;
        return false;
    }

    std::cout << "Unloading default FTDI modules.\nErrors mean they are already removed." << std::endl;
    exec(m_unloadFTDI);
    exec(m_unloadUsbserial);
    std::cout << "Default modules successfully unloaded." << std::endl;


    if ((m_ftStatus = FT_OpenEx((char*)m_serialNo.c_str(), FT_OPEN_BY_SERIAL_NUMBER, &m_ftHandle)) != FT_OK) {
        printf("Error FT_OpenEx(%d)\n", (int) m_ftStatus);
        return false;
    }

    if ((m_ftStatus = FT_SetBaudRate(m_ftHandle, BAUD_RATE)) != FT_OK) {
        printf("Error FT_SetBaudRate(%d)\n", (int) m_ftStatus);
        return false;
    }

    m_online = true;

    return sendPacket(m_packetStop);
}

bool EpiLaser::destroyModule() {
    m_online = false;
    sendPacket(m_packetStop);
    FT_Close(m_ftHandle);
    printf("Closed LaserBox\n");
    if (m_readBuffer)
        free(m_readBuffer);
    return true;
}

bool EpiLaser::getState() {
    DWORD EventDWord;
    DWORD RxBytes;
    DWORD TxBytes;
    m_ftStatus = FT_GetStatus(m_ftHandle, &RxBytes, &TxBytes, &EventDWord);
    if (m_ftStatus == FT_OK) {
        return true;
    }
    return false;
}

bool EpiLaser::handleGetData(std::vector<Point> &p_points) {

    p_points.clear();

    for (Point curr : m_points) {
        p_points.push_back(curr);
    }
    m_points.clear();

    return true;
}
/*
bool EpiLaser::handleGetData() {

    int count = 0;
    for (EpiLaser::Point curr : m_points) {
        ret["data"][0][count] = {}; // NONE
        for (int i = 0; i < WORD_COUNT; i++) {
            ret["data"][0][count][i] = curr.none[i];
        }
        ret["data"][1][count] = {}; // RED
        for (int i = 0; i < WORD_COUNT; i++) {
            ret["data"][1][count][i] = curr.red[i];
        }
        ret["data"][2][count] = {}; // BLUE
        for (int i = 0; i < WORD_COUNT; i++) {
            ret["data"][2][count][i] = curr.blue[i];
        }
        count++;
    }
    m_points.clear();
    return true;
}
*/

bool EpiLaser::handleStart() {

    if (m_online) {
        if (!handleStop()) {
            std::cout << "Failed to send stop cmd." << std::endl;
            return false;
        }
    }
    if (!sendPacket(m_packetStart)) {
        std::cout << "Failed to send start cmd." << std::endl;
        return false;
    }
    m_online = true;
    m_tStart = std::chrono::high_resolution_clock::now();
    return true;
}

bool EpiLaser::handleStop() {

    if (!sendPacket(m_packetStop)) {
        std::cout << "Failed to send stop cmd." << std::endl;
        return false;
    }

    m_online = false;
    return true;
}

bool EpiLaser::handleClear() {

    m_points.clear();
    return true;
}

void EpiLaser::cyclicFunc() {

    // Check 500 ms cycle
    if (m_online && (
            std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - m_tStart).count()
            > UPDATE_PERIOD)) {

        // Restart 500 ms timer
        m_tStart = std::chrono::high_resolution_clock::now();
        DWORD bytesReady = 0;
        DWORD bytesRead = 0;

        m_ftStatus = FT_GetQueueStatus(m_ftHandle, &bytesReady);

        if (m_ftStatus == FT_OK) {

            printf("\nGot %d bytes.\n", (int) bytesReady);

            if (bytesReady > 0) {

                DWORD buffLength = bytesReady + m_tailLength;
                m_readBuffer = (unsigned char *) realloc(m_readBuffer, buffLength);
                memcpy(m_readBuffer, m_tailBuffer, m_tailLength);
                memset(m_readBuffer + m_tailLength, 0xFF, bytesReady);

                m_ftStatus = FT_Read(m_ftHandle, m_readBuffer + m_tailLength, bytesReady, &bytesRead);
                m_tailLength = 0;

                if (m_ftStatus != FT_OK) {
                    printf("Error FT_Read(%d)\n", (int) m_ftStatus);
                    return;
                }

                if (bytesRead != bytesReady) {
                    printf("FT_Read only read %d (of %d) bytes\n", (int) bytesRead, (int) bytesReady);
                    return;
                }
                int bytesParsed = 0;

                while (bytesParsed <= buffLength - PACKET_IN_LENGTH) {
                    unsigned char *packet = (unsigned char *) malloc(PACKET_IN_LENGTH);
                    memcpy(packet, m_readBuffer + bytesParsed, PACKET_IN_LENGTH);
                    if (packet[PACKET_IN_LENGTH - 1] == Crc8(packet, PACKET_IN_LENGTH - 1)) {
                        Point newPoint;
                        //dumpBuffer(packet, PACKET_IN_LENGTH);
                        for (int group = 0; group < (PACKET_IN_LENGTH - 1) / GROUP_LENGTH; group++) {
                            for (int type = 0; type < 4; type++) {
                                m_converter.chars[0] = packet[group * GROUP_LENGTH + type * 2];
                                m_converter.chars[1] = packet[group * GROUP_LENGTH + type * 2 + 1];
                                DWORD shifted = m_converter.intVal;
                                if (m_converter.intVal >= 512) {
                                    shifted -= 1024;
                                }
                                switch (group) {
                                    case 0:
                                        newPoint.none[type] = shifted;
                                        break;
                                    case 1:
                                        newPoint.red[type] = shifted;
                                        break;
                                    case 2:
                                        newPoint.blue[type] = shifted;
                                        break;
                                    default:
                                        printf("WTF?");
                                        break;
                                }
                            }
                        }
                        if (m_points.size() >= MAX_POINTS) {
                            m_points.erase(m_points.begin(), m_points.begin() + (MAX_POINTS / 2));
                        }
                        m_points.push_back(newPoint);
                    } else {

                        printf("Warning! Corrupted packet.\n");
                    }
                    bytesParsed += PACKET_IN_LENGTH;
                }
                m_tailLength = buffLength - bytesParsed;
                std::cout << "tail = " << m_tailLength << std::endl;
                if (m_tailLength > 0) {
                    m_tailBuffer = (unsigned char *) realloc(m_tailBuffer, m_tailLength);
                    memcpy(m_tailBuffer, m_readBuffer + bytesParsed, m_tailLength);
                }

                std::cout << "Recieved points count: " << bytesParsed / PACKET_IN_LENGTH;
                std::cout << std::endl << std::endl;
            }
        } else {
            printf("\nFT_GetQueueStatus failed (%d).\n", (int) m_ftStatus);
        }
    }
}

std::string EpiLaser::exec(const char *cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}


EpiLaser::~EpiLaser() {

}

