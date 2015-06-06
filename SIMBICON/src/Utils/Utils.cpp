/*
	Simbicon 1.5 Controller Editor Framework, 
	Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
	All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

	This file is part of the Simbicon 1.5 Controller Editor Framework.

	Simbicon 1.5 Controller Editor Framework is free software: you can 
	redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Simbicon 1.5 Controller Editor Framework is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even the 
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#include <TCL/tcl.h>
#include <TCL/tk.h>
#include <sstream>
#include "utils.h"
#include "AppGUI\Globals.h"
#include <iostream>

/**
 * Output the message to a file...
 */
void logPrint(char *format, ...){
    static char message[1024];
    va_list vl;

	va_start(vl, format);   
	vsprintf(message, format, vl);
	va_end(vl);

	//in case the folde rdon't exist yet
	static bool first_time = true;
	if (first_time){
		system("mkdir out");
		first_time = false;
	}

	//TODO change this
	static FILE *fp = fopen("out\\log.txt", "wt");

	if (fp != NULL){
		fprintf(fp, "%s", message);
		fflush(fp);
	}

    
}


void debugLog(std::string msg){
	//TODO change this
	static FILE *fp = fopen("out\\debugLog.txt", "a");

	fprintf(fp, "%s", msg.c_str());
	fflush(fp);
}


/**
	Converts a list of strings to a unique TCL string list
	The string is allocated using Tcl_Alloc, so it should be managed by TCL from now on
	Pass true to "enQuote" to put extra quotes around the generated string
*/
char* stringListToTclList( DynamicArray<const char*> stringList, bool enQuote ) {

	uint storageSize = 1;  // For terminating null
	if( enQuote ) storageSize = 3; // For "\""  and  "\"" and terminating null

	for( uint i=0; i < stringList.size(); ++i )
		storageSize += 3 + strlen( stringList[i] );  // For "{string} "

	char* buffer = Tcl_Alloc( storageSize );
	int bufferIdx = 0;
	if( enQuote ) buffer[bufferIdx++] = '\"';
	for( uint i=0; i < stringList.size(); ++i ) {
		buffer[bufferIdx++] = '{';
		strcpy( buffer+bufferIdx, stringList[i] );
		bufferIdx += strlen(stringList[i]);
		buffer[bufferIdx++] = '}';
		buffer[bufferIdx++] = ' ';
	}
	if( enQuote ) buffer[bufferIdx++] = '\"';
	buffer[bufferIdx++] = 0;

	return buffer;

}


/**
those method are helper to split a string
*/
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
	std::stringstream ss(s);
	std::string item;
	//split the string
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
	}

	if (!elems.empty()){
		//remove the endl from the last element
		std::stringstream ss2(elems.back());
		elems.pop_back();
		while (std::getline(ss2, item, '\n')) {
			elems.push_back(item);
		}
	}
	return elems;
}


std::vector<std::string> split(const std::string &s, char delim) {
	std::vector<std::string> elems;
	split(s, delim, elems);
	return elems;
}


/**
this function interpret the path (using the global configuration data path
*/
std::string interpret_path(std::string path){
	std::vector<std::string> splited_path;
	std::ostringstream oss;
	bool config_folder_reached;
	bool first_passage;

	splited_path = split(path, '/');
	//now we look for the configuration folder in the vector and we create the real path
	config_folder_reached = false;
	for (int i = 0; i < (int)splited_path.size(); ++i){
		//we ignore anything before the data folder
		if (!config_folder_reached){
			if (splited_path[i] == "configuration_data"){
				config_folder_reached = true;
				oss << Globals::data_folder_path;
				first_passage = true;
			}
			continue;
		}

		//so if we reach here we have only the things following the data folder
		if (first_passage){
			first_passage = false;
		}
		else{
			oss << "/";
		}
		oss << splited_path[i];
	}
	return oss.str();
}



