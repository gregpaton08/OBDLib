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
    void sendCMD(uint8_t mode, uint8_t pid);
    bool getResultForPid(float &res, uint8_t mode, uint8_t pid);
    float pidToDec(uint8_t pid, char *res);
    bool waitForPid(uint8_t pid, uint8_t timeout = 0);
    bool waitForSerial(uint8_t timeout = 0);
    void getSupportedPids();
    bool isPidSupported(uint8_t mode, uint8_t pid);
    
protected:
    unsigned int hex2uint16(const char *p);
    unsigned char hex2uint8(const char *p);

private:
    bool mode01Pids[160];
};