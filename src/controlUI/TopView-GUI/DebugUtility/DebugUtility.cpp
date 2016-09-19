/**
 * @addtogroup DebugUtility
 */

/*****************************************************************************************
 * DebugUtility.cpp
 *
 *     Created on: 15-May-2016
 *  Last Modified: 25-Aug-2016
 *         Author: Sona Praneeth Akula
 *        Project: Data_Structures_and_Algorithms
 *        Description: Code to handle debugging
 *
 * Date				Author							Modification
 * 25-Aug-2016	Sona Praneeth Akula	* Added closing of ofstream to unsetDebug function
*  										* Pushed brackets from side to bottom for every function
 *****************************************************************************************/

#include "DebugUtility.hpp"

bool DEBUG_ACTIVATE = false; 			/*!< boolean variable to start debugging. DEFAULT: "false" */
int DEBUG_LEVEL = 1;			/*!< level of debugging messages to be printed. DEFAULT: "0" */
string debugFile = "log/myDebug.txt"; /*!< If debugging is set to file. DEFAULT: "log/myDebu.txt" */
bool debugFileOpen = false; 	/*!< boolean variable to check if debug file is open. DEFAULT: "false" */
ofstream debugOutFile;			/*!< stream operator for writing to debug file */
stringstream debugMessage; 		/*!< stream for storing debug messages */
stringstream debugExceptionMessage; 		/*!< stream for storing exception messages */

void setDebug()
{
	DEBUG_ACTIVATE = true; debugMessage.str(string());
	printDebugMessage(1, "[ DEBUG] DEBUG IS ACTIVATED.\n");
}

void setDebug(int level)
{
	DEBUG_ACTIVATE = true; debugMessage.str(string());
	DEBUG_LEVEL = level;
	DEBUG_MSG << "[ DEBUG] DEBUG IS ACTIVATED WITH LEVEL " << DEBUG_LEVEL << ".\n";
	printDebugMessage(1, debugMessage);
}

void setDebug(string filename)
{
	DEBUG_ACTIVATE = true; debugFile = filename;
	debugFileOpen = true;
	debugMessage.str(string());
	debugOutFile.open(debugFile.c_str(), ofstream::out);
	DEBUG_MSG << "[ DEBUG] DEBUG TO FILE \"" << debugFile << "\" ACTIVATED.\n";
	cout << "[ DEBUG] DEBUG TO FILE \"" << debugFile << "\" ACTIVATED.\n";
	printDebugMessage(1, debugMessage);
}

void printDebugMessage( int level,
						stringstream &debugMessageToPrint)
{
	if (DEBUG_ACTIVATE && !debugFileOpen && level <= DEBUG_LEVEL)
	{
		cout << debugMessageToPrint.str();
		clearDebug(debugMessageToPrint);
	}
	else if (DEBUG_ACTIVATE && debugFileOpen && level <= DEBUG_LEVEL)
	{
		streambuf *coutbuf = cout.rdbuf();
		cout.rdbuf(debugOutFile.rdbuf());
		cout << debugMessageToPrint.str();
		clearDebug(debugMessageToPrint);
		cout.rdbuf(coutbuf);
	}
	else
	{
		clearDebug(debugMessageToPrint);
	}
}

void printDebugMessage( int level,
						string debugMessageToPrint)
{
	if( !(level>=1 && level<=3) )
	{
		debugExceptionMessage.str(string());
		debugExceptionMessage << "\nEXCEPTION::Wrong debug level " << level << " used.\n";
		debugExceptionMessage << "Please use debug level between 1 and 3.\n";
		debugExceptionMessage << "Highest preference message has level 1.\n";
		throw debugExceptionMessage.str();
	}
	if (DEBUG_ACTIVATE && !debugFileOpen && level <= DEBUG_LEVEL)
	{
		cout << debugMessageToPrint;
		debugMessageToPrint = "";
	}
	else if (DEBUG_ACTIVATE && debugFileOpen && level <= DEBUG_LEVEL)
	{
		streambuf *coutbuf = cout.rdbuf();
		cout.rdbuf(debugOutFile.rdbuf());
		cout << debugMessageToPrint;
		debugMessageToPrint = "";
		cout.rdbuf(coutbuf);
	}
	else
	{
		debugMessageToPrint = "";
	}
}

void clearDebug(stringstream &debugMessageToPrint)
{
	debugMessageToPrint.str(string());
}

void unsetDebug()
{
	DEBUG_ACTIVATE = false;
	printDebugMessage(1, "[ DEBUG] DEBUG DEACTIVATED.\n");
	if(debugOutFile.is_open())
	{
		debugOutFile.close();
	}
}

