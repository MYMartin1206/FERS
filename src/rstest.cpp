/// rstest.cpp - Implementation of rstest.h (test messaging)
/// Marc Brooker, 20 March 2006
/// Edited by Yaaseen Martin, 27 August 2019

#include <iostream>
#include <sstream>
#include <cstdio>
#include <cstdarg>
#include <boost/thread/mutex.hpp> // For boost::mutex
#include "rstest.h"

rsTest::Level test_level = rsTest::RS_VERY_VERBOSE; // The current test level

// We use a mutex for log information printing, to stop messages from getting mangled
boost::mutex testMutex;

// Print out a test message including the line and file name
void rsTest::print(const rsTest::Level level, const std::string &str, const std::string file, const int line) {
    if (level >= test_level) {
        boost::mutex::scoped_lock lock(testMutex); // Lock the mutex
        std::ostringstream oss;
        oss << "[" << file << " " << line << "] ";
        oss << str << "\n";
        std::cerr << oss.str();
        // Mutex will automatically be unlocked here by scoped_lock
    }
}

// Formatted print of the current test level, doesn't include filename and line
// Uses the cstdarg variable arguments system and the vfprintf function to handle the arguments
// If your system does not have the standard vfprintf function in it's library, you will have to make a pla
void rsTest::printf(const rsTest::Level level, const char *format, ...)
{
	if (level >= test_level) {
		boost::mutex::scoped_lock lock(testMutex); // Lock the mutex
		va_list ap;
		va_start(ap, format);
		vfprintf(stderr, format, ap);
		va_end(ap);
		// Mutex will automatically be unlocked here by scoped_lock
	}
}

// See comments for printf(Level, char *)
void rsTest::printf(const rsTest::Level level, const std::string &format, ...) {
	if (level >= test_level) {
		boost::mutex::scoped_lock lock(testMutex); // Lock the mutex
		va_list ap;
		va_start(ap, format);
		vfprintf(stderr, format.c_str(), ap);
		va_end(ap);
		// Mutex will automatically be unlocked here by scoped_lock
	}
}

// Set the current test level
void rsTest::setTestLevel(rsTest::Level level) {
	if (level <= rsTest::RS_EXTREMELY_CRITICAL)
		test_level = level;
}


