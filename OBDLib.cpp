/*! \file OBDLib.cpp
 *  \brief Arduino library for interfacing with ELM327 based OBD connection.
 *  \author Greg Paton
 *  \date 3/6/2013
 */
// Copyright (c) MPG 2013

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "OBDLib.h"

unsigned int hex2uint16(const char *p)
{
	char c = *p;
	unsigned int i = 0;
	for (char n = 0; c && n < 4; c = *(++p)) {
		if (c >= 'A' && c <= 'F') {
			c -= 7;
		} else if (c>='a' && c<='f') {
			c -= 39;
        } else if (c == ' ') {
            continue;
        } else if (c < '0' || c > '9') {
			break;
        }
		i = (i << 4) | (c & 0xF);
		n++;
	}
	return i;
}

unsigned char hex2uint8(const char *p)
{
	unsigned char c1 = *p;
	unsigned char c2 = *(p + 1);
	if (c1 >= 'A' && c1 <= 'F')
		c1 -= 7;
    else if (c1 >='a' && c1 <= 'f')
        c1 -= 39;
	else if (c1 < '0' || c1 > '9')
		return 0;
    
	if (c2 >= 'A' && c2 <= 'F')
		c2 -= 7;
    else if (c2 >= 'a' && c2 <= 'f')
        c2 -= 39;
	else if (c2 < '0' || c2 > '9')
		return 0;
    
	return c1 << 4 | (c2 & 0xf);
}

/*! \brief Class constructor.
 *
 * \author Greg Paton
 * \date 3/6/2013
 */
OBDLib::OBDLib() {
    
}

/*! \brief Method to initialize connection to OBD.
 *
 * \return True if connected. False otherwise.
 *
 * \author Greg Paton
 * \date 3/6/2013
 */
bool OBDLib::init() {
    
    bool prompted;
    unsigned char n;
    char buffer[OBD_RECV_BUF_SIZE];
    char c;
    
    // set up ELM327
    // reset
    Serial.write("ATZ\r\n");
    while (false == Serial.find(">"));
    // turn spaces off
    Serial.write("ATS0\r\n");
    while (false == Serial.find(">"));
    // turn line feeds off
    Serial.write("ATL0\r\n");
    while (false == Serial.find(">"));
    // turn echo off
    Serial.write("ATE0\r\n");
    while (false == Serial.find(">"));
    // set timeout to 152ms
    Serial.write("ATST26\r\n");
    while (false == Serial.find(">"));
    // aggressive adaptive timeout
    Serial.write("ATAT2\r\n");
    while (false == Serial.find(">"));
    
    return false;
}

/*! \brief Method to send an OBD command.
 *
 * \param mode IN: OBD Mode.
 * \param pid IN: OBD PID.
 *
 * \return void
 *
 * \author Greg Paton
 * \date 3/6/2013
 */
void OBDLib::sendCMD(unsigned char mode, unsigned char pid) {
	char cmd[8];
	sprintf(cmd, "%02X%02X 1\r", mode, pid);
	Serial.write(cmd);
}

/*! \brief Method to get the result for an OBD PID.
 *
 * \param res OUT: Result from OBD PID.
 * \param pid IN: OBD PID.
 *
 * \return True if successful. False otherwise.
 *
 * \author Greg Paton
 * \date 3/6/2013
 */
bool OBDLib::getResultForPid(int &res, unsigned char pid) {
    //sendCMD(pid);
    return false;
}

/*! \brief Method to convert mode 01 PID result to decimal.
 *
 * \param pid IN: OBD PID.
 * \param res IN: Result from OBD PID.
 *
 * \return Decimal value of PID result.
 *
 * \author Greg Paton
 * \date 4/3/2013
 */
uint8_t OBDLib::PidToDec(uint8_t pid, char *res) {
    switch (pid) {
        case 0x0C:
            return hex2uint16(res) / 4;
            break;
        case 0x0D:
            return hex2uint8(res) * 0.621371;
            break;
        case 0x10:
            return hex2uint16(res) / 100;
            break;
        case 0x2F:
            return hex2uint8(res) * 100 / 255;
            break;
        default:
            return 0;
    }
}

/*! \brief Method to wait for PID result to become available.
 *
 * \param pid IN: OBD PID.
 * \param timeout IN: Time (in ms) to wait. Set to 0 for no timeout.
 *
 * \return True if available. False otherwise.
 *
 * \author Greg Paton
 * \date 4/4/2013
 */
bool OBDLib::waitForPid(uint8_t pid, uint8_t timeout) {
    // format PID to char array
    uint8_t strPidSize = 4;
    char strPid[strPidSize];
    String strPidtemp = String(pid, HEX);
    strPidtemp.toUpperCase();
    if (strPidtemp.length() == 1) {
        strPidtemp = "0" + strPidtemp;
    }
    strPidtemp.toCharArray(strPid, strPidSize);
    
    uint8_t stime = millis();
    while (millis() - stime > timeout || timeout == 0) {
        if (Serial.find(strPid))
            return true;
        else if (Serial.find("NODATA"))
            return false;
        else if (Serial.find("STOPPED"))
            return false;
    }
    return false;
}

/*! \brief Method to wait for Serial data to become available.
 *
 * \param timeout IN: Time (in ms) to wait. Set to 0 for no timeout.
 *
 * \return True if available. False otherwise.
 *
 * \author Greg Paton
 * \date 4/4/2013
 */
bool OBDLib::waitForSerial(uint8_t timeout) {
    uint8_t stime = millis();
    while (millis() - stime > timeout || timeout == 0) {
        if (Serial.available() > 0)
            return true;
    }
    return false;
}