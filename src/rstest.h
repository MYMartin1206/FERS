/// rstest.h - Message support functions and test levels
/// Marc Brooker, 20 March 2006
/// Edited by Yaaseen Martin, 27 August 2019

#ifndef __RS_TEST_H
#define __RS_TEST_H

#include <string>

// Macro which calls the test print function with the current file and line
#define TEST_PRINT(level, str) rsTest::print(level, str, __FILE__, __LINE__)

// Formatting is as per the C printf function, with all the format specifiers supported
// RS_INFORMATIVE - Messages which may be informative to the user
// RS_IMPORTANT
// RS_CRITICAL - Stuff that is very important
// RS_EXTREMELY_CRITICAL - Very important messages that must be printed

namespace rsTest {

	enum Level {
        RS_VERY_VERBOSE, // Messages which are only useful for testging
        RS_VERBOSE, // Messages which are unlikely to prove important
        RS_INFORMATIVE, // Messages which may be informative to the user
        RS_IMPORTANT, // Important messages
        RS_CRITICAL, // Critical messages, such as errors which may lead to incorrect results
        RS_EXTREMELY_CRITICAL // Extremely important messages
	};

	// Print the current test message, file and line, as long as level is greater or equal to the current test level
    void print(const Level level, const std::string &str, const std::string file, const int line);

	// Print a formatted test message at the current level
	void printf(const Level level, const char *format, ...);

	// Overloaded printf which takes format string as std::string
	void printf(const Level level, const std::string &format, ...);

	// Set the current test level
	void setTestLevel(Level level);
}

#endif
