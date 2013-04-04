//
// OBDLib.h
// OBDLib
//
// Created by Greg Paton on 3/3/2013
//
//

#define OBD_RECV_BUF_SIZE 48
#define OBD_BAUDRATE 38400

class OBDLib {
public:
    OBDLib();
    bool init();
    void sendCMD(unsigned char mode, unsigned char pid);
    bool getResultForPid(int &res, unsigned char pid);
    uint8_t PidToDec(uint8_t pid, char *res);
    bool waitForPid(uint8_t pid, uint8_t timeout = 0);
    bool waitForSerial(uint8_t timeout = 0);
private:
    
};