/**
 * @defgroup DebugUtility DebugUtility
 * @brief	 This code handles debugging to file/console
 */

/*****************************************************************************************
 * DebugUtility.hpp
 *
 *     Created on: 15-May-2016
 *  Last Modified: 25-Aug-2016
 *         Author: Sona Praneeth Akula
 *        Project: Data_Structures_and_Algorithms
 *        Description: Code to handle debugging
 *
 * Date				Author							Modification
 * 25-Aug-2016	Sona Praneeth Akula		* Added detailed comment
 * 										* Added SET_DEBUG_TO_FILE(level, filename)
 * 										* Renamed SET_DEBUG_TO_FILE(filename) to SET_DEBUG_FILE(filename)
 *****************************************************************************************/

#ifndef DEBUGUTILITY_DEBUGUTILITY_HPP_
#define DEBUGUTILITY_DEBUGUTILITY_HPP_
#pragma once

#include "../Headers.h"

extern stringstream debugMessage;
extern stringstream debugExceptionMessage;
extern bool DEBUG_ACTIVATE;
extern int DEBUG_LEVEL;
extern string debugFile;
extern bool debugFileOpen;
extern ofstream debugOutFile;

#define DEBUG_MSG if(DEBUG_ACTIVATE) debugMessage

#define SET_DEBUG setDebug()
#define SET_DEBUG_LEVEL(level) setDebug(level)
#define SET_DEBUG_FILE(filename) setDebug(filename)
#define SET_DEBUG_TO_FILE(level, filename) setDebug(level); setDebug(filename)
#define DEBUG_PRINT(level, debugMessage) printDebugMessage(level, debugMessage)


/**
 * @brief Clear the debug stringstream
 *
 * @param debugMessageToPrint
 *
 */
void clearDebug(stringstream &debugMessageToPrint);

/**
 * @brief Activate debugging for the program
 *
 */
void setDebug();

/**
 * @brief Set the debug level for debugging purpose
 *
 * @param [int] level - 1,2,3.
 * 				1 - highest preference message. 3 - least preference message
 *
 */
void setDebug(int level);

/**
 * @brief Set the debug level for debugging purpose
 *
 * @param [string] filename - Name of the file to which debug statements have to be printed
 *
 */
void setDebug(string filename);

/**
 * @brief Prints debug message (stringstream) to file/console
 *
 * @param [int] level
 * @param [stringstream] debugMessageToPrint
 *
 */
void printDebugMessage(int level, stringstream &debugMessageToPrint);

/**
 * @brief Prints debug message (string) to file/console
 *
 * @param [int] level
 * @param [string] debugMessageToPrint
 */
void printDebugMessage(int level, string debugMessageToPrint);

/**
 * @brief Deactivate debugging for the program
 *
 */
void unsetDebug();

#endif /* DEBUGUTILITY_DEBUGUTILITY_HPP_ */
