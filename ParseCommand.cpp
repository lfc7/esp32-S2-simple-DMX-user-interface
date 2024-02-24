/**
 * ParseCommand - A Wiring/Arduino library to tokenize and parse commands
 * received over a serial port.
 * 
 * Copyright (C) 2012 Stefan Rado
 * Copyright (C) 2011 Steven Cogswell <steven.cogswell@gmail.com>
 *                    http://husks.wordpress.com
 * 
 * Version 20120522 
 * 
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "ParseCommand.h"

/**
 * Constructor makes sure some things are set.
 */
ParseCommand::ParseCommand()
  : commandList(NULL),
    commandCount(0),
    defaultHandler(NULL),
    term('\n'),           // default terminator for commands, newline character
    last(NULL)
{
  strcpy(delim, " "); // strtok_r needs a null-terminated string
  clearBuffer();
}

/**
 * Adds a "command" and a handler function to the list of available commands.
 * This is used for matching a found token in the buffer, and gives the pointer
 * to the handler function to deal with it.
 */
void ParseCommand::addCommand(const char *command, void (*function)()) {
  #ifdef PARSECOMMAND_DEBUG
    Serial.print("Adding command (");
    Serial.print(commandCount);
    Serial.print("): ");
    Serial.println(command);
  #endif

  commandList = (ParseCommandCallback *) realloc(commandList, (commandCount + 1) * sizeof(ParseCommandCallback));
  strncpy(commandList[commandCount].command, command, PARSECOMMAND_MAXCOMMANDLENGTH);
  commandList[commandCount].function = function;
  commandCount++;
}

/**
 * This sets up a handler to be called in the event that the receveived command string
 * isn't in the list of commands.
 */
void ParseCommand::setDefaultHandler(void (*function)(const char *)) {
  defaultHandler = function;
}


/**
 * This checks the Serial stream for characters, and assembles them into a buffer.
 * When the terminator character (default '\n') is seen, it starts parsing the
 * buffer for a prefix command, and calls handlers setup by addCommand() member
 */
void ParseCommand::readCommand(const char * sbuffer) {
	
      snprintf(buffer,sizeof(buffer),"%s",sbuffer);
      #ifdef PARSECOMMAND_DEBUG
        Serial.print("Received: ");
        Serial.println(buffer);
      #endif

      char *command = strtok_r(buffer, delim, &last);   // Search for command at start of buffer
      if (command != NULL) {
        boolean matched = false;
        for (int i = 0; i < commandCount; i++) {
          #ifdef PARSECOMMAND_DEBUG
            Serial.print("Comparing [");
            Serial.print(command);
            Serial.print("] to [");
            Serial.print(commandList[i].command);
            Serial.println("]");
          #endif

          // Compare the found command against the list of known commands for a match
          if (strncmp(command, commandList[i].command, PARSECOMMAND_MAXCOMMANDLENGTH) == 0) {
            #ifdef PARSECOMMAND_DEBUG
              Serial.print("Matched Command: ");
              Serial.println(command);
            #endif

            // Execute the stored handler function for the command
            (*commandList[i].function)();
            matched = true;
            break;
          }
        }
        if (!matched && (defaultHandler != NULL)) {
          (*defaultHandler)(command);
        }
      }
      clearBuffer();
}
 
/*
 * Clear the input buffer.
 */
void ParseCommand::clearBuffer() {
  buffer[0] = '\0';
  bufPos = 0;
}

/**
 * Retrieve the next token ("word" or "argument") from the command buffer.
 * Returns NULL if no more tokens exist.
 */
char *ParseCommand::next() {
  return strtok_r(NULL, delim, &last);
}
