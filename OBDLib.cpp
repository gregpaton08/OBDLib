/*! \file OBDLib.cpp
 *  \brief Arduino library for interfacing with ELM327 based OBD connection.
 *  \author Greg Paton
 *  \date 3/6/2013
 */
// Copyright (c) MPG 2013

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "OBDLib.h"


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
bool OBDLib::init(File &file) {
    
    // initialize mode01Pids
//    for (uint8_t i = 0; i < 160; ++i) {
//        mode01Pids[i] = false;
//    }
    
    // wait for prompt
    while (false == Serial.find(">"));
    
    // set up ELM327
    // reset
    Serial.write("ATZ\r");
    while (false == Serial.findUntil("ELM327", "?"));
    while (false == Serial.find(">"));
    
    // turn echo off
    Serial.write("ATE0\r");
    while (false == Serial.findUntil("OK", "?"));
    while (false == Serial.find(">"));
    
    // turn spaces off
    Serial.write("ATS0\r");
    while (false == Serial.findUntil("OK", "?"));
    while (false == Serial.find(">"));
    
    // turn line feeds off
    Serial.write("ATL0\r");
    while (false == Serial.findUntil("OK", "?"));
    while (false == Serial.find(">"));
    
    // set timeout to 152ms
    Serial.write("ATST26\r");
    while (false == Serial.findUntil("OK", "?"));
    while (false == Serial.find(">"));
    
    // aggressive adaptive timeout
    Serial.write("ATAT2\r");
    while (false == Serial.findUntil("OK", "?"));
    while (false == Serial.find(">"));
    
    
    return true;
}

/*! \brief Method to send an OBD command.
 *
 * \param mode IN: OBD Mode.
 * \param pid IN: OBD PID.
 *
 * \return Void.
 *
 * \author Greg Paton
 * \date 3/6/2013
 */
void OBDLib::sendCMD(uint8_t mode, uint8_t pid) {
	char cmd[8];
	sprintf(cmd, "%02X%02X1\r", mode, pid);
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
bool OBDLib::getResultForPid(float &res, uint8_t mode, uint8_t pid) {
    uint8_t len = 0;
    uint8_t pidResSize = 10;
    char pidRes[pidResSize];
    
    if (false == isPidSupported(mode, pid))
        return false;
    
    // Query PID
    sendCMD(mode, pid);
    
    // Wait until data is available
    if (false == waitForPid(pid, 200))
        return false;
    
    // loop until new line character found
    while (len < pidResSize) {
        unsigned char c = Serial.read();
        if (c == (unsigned char)-1) continue;
        if (c == '\n' || c == '\r') break;
        //OBDLog.write(c);
        pidRes[len] = c;
        ++len;
    }
    
    // Get result
    res = pidToDec(pid, pidRes);
    
    return true;
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
float OBDLib::pidToDec(uint8_t pid, char *res) {
    switch (pid) {
        // RPM
        case 0x0C:
            return hex2uint16(res) / 4.0;
            break;
        // Speed (MPH)
        case 0x0D:
            return hex2uint8(res) * 0.621371;
            break;
        // MAF
        case 0x10:
            return hex2uint16(res) / 100.0;
            break;
        // Fuel Level
        case 0x2F:
            return hex2uint8(res) * 100.0 / 255.0;
            break;
        default:
            return 0.0;
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

/*! \brief Method to find which PIDs are supported
 *
 * \return Void.
 *
 * \author Greg Paton
 * \date 4/4/2013
 */
void OBDLib::getSupportedPids() {
    uint8_t mask = 1;
    uint8_t zeroCnt = 0;
    long unsigned int supPidInt;
    uint8_t len = 0;
    uint8_t supPidSize = 12;
    char supPid[supPidSize];
    
    do {
        sendCMD(0x01, 0x00);
    } while (false);
        
    while (len < supPidSize) {
        char c = Serial.read();
        if (c == (char)-1)
            continue;
        if (zeroCnt < 2) {
            if (c == '0')
                ++zeroCnt;
            continue;
        }
        if (c == '\r' || c == '\n' || c == '>')
            break;
        supPid[len] = c;
        ++len;
    }
    
    supPidInt = hex2uint16(supPid + 2) + (65536 * hex2uint16(supPid));
    
    for (uint8_t i = 0; i < 4; ++i) {
        mask = 1;
        for (uint8_t j = 0; j < 8; ++j) {
            mode01Pids[i*j] = supPid[i] & mask == 1;
            mask = mask << 1;
        }
    }
}

/*! \brief Method to find if a PID is supported.
 *
 * \param pid IN: PID.
 *
 * \return True if supported. False otherwise. 
 *
 * \author Greg Paton
 * \date 4/4/2013
 */
bool OBDLib::isPidSupported(uint8_t mode, uint8_t pid) {
    if (mode == 0x01) {
        if (pid < 0 || pid > 159)
            return false;
        
        return mode01Pids[pid];
    }
    
    return false;
}




/*! \brief Method to convert a hex string of length 4 to decimal uint16.
 *
 * \param p IN: Pointer to hex string.
 *
 * \return Decimal value of hex string.
 *
 * \author Greg Paton
 * \date 4/4/2013
 */
unsigned int OBDLib::hex2uint16(const char *p)
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

/*! \brief Method to convert a hex string of length 2 to decimal uint8.
 *
 * \param p IN: Pointer to hex string.
 *
 * \return Decimal value of hex string.
 *
 * \author Greg Paton
 * \date 4/4/2013
 */
unsigned char OBDLib::hex2uint8(const char *p)
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
