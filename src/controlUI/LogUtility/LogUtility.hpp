/**
 * @defgroup LogUtility LogUtility
 * @brief	 This code handles logging to file/console
 */

/*****************************************************************************************
 * LogUtility.hpp
 *
 *     Created on: 15-May-2016
 *  Last Modified: 25-Aug-2016
 *         Author: Sona Praneeth Akula
 *        Project: Data_Structures_and_Algorithms
 *        Description: Code to handle logging
 *
 * Date				Author							Modification
 * 25-Aug-2016	Sona Praneeth Akula	* Added detailed comment
 * 										* Added SET_LOG_TO_FILE(level, filename)
 * 										* Renamed SET_LOG_TO_FILE(filename) to SET_LOG_FILE(filename)
 *****************************************************************************************/

#ifndef LOGUTILITY_LOGUTILITY_HPP_
#define LOGUTILITY_LOGUTILITY_HPP_
#pragma once


#include "../Headers.h"

extern bool LOG_ACTIVATE;
extern int LOG_LEVEL;
extern string logFile;
extern bool logFileOpen;
extern ofstream logOutFile;
extern stringstream logMessage;
extern stringstream logExceptionMessage;

#define LOG_MSG if(LOG_ACTIVATE) logMessage << "[ LOG] " 

#define SET_LOG setLog()
#define SET_LOG_LEVEL(level) setLog(level)
#define SET_LOG_TO_FILE(level, filename) setLog(level); setLog(filename)
#define SET_LOG_FILE(filename) setLog(filename)
#define LOG_INFO(level, logMessage) printLogMessage(level, logMessage)


/**
 * @brief Clear the log stringstream
 *
 * @param logMessageToPrint
 *
 */
void clearLog(stringstream &logMessageToPrint);

/**
 * @brief Activate logging for the program
 *
 */
void setLog();

/**
 * @brief Set the log level for logging purpose
 *
 * @param [int] level - 1,2,3.
 * 				1 - highest preference message. 3 - least preference message
 *
 */
void setLog(int level);

/**
 * @brief Set the log level for logging purpose
 *
 * @param [string] filename - Name of the file to which log statements have to be printed
 *
 */
void setLog(string filename);

/**
 * @brief Prints log message (stringstream) to file/console
 *
 * @param [int] level
 * @param [stringstream] logMessageToPrint
 *
 */
void printLogMessage(int level, stringstream &logMessageToPrint);

/**
 * @brief Prints log message (string) to file/console
 *
 * @param [int] level
 * @param [string] logMessageToPrint
 */
void printLogMessage(int level, string logMessageToPrint);

/**
 * @brief Deactivate logging for the program
 *
 */
void unsetLog();

#endif /* LOGUTILITY_LOGUTILITY_HPP_ */
