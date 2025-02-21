
#include "xunit.hpp"
#include <iostream>
#include <fstream>

void XUnit::writeOutput() {

	int failures = 0;

	if (isFailure()) {
		failures = 1;
	}

	// Try and open file.
	std::ofstream file(path);

	// Write verdict output using XUnit XML structure.
	file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << std::endl;
	file << "<testsuites name=\"test\" errors=\"0\" failures=\"" << failures << "\">" << std::endl;
		file << "<testsuite name=\"test\" errors=\"0\" failures=\"" << failures << "\">" << std::endl;
			file << "<testcase name=\"test\" time=\"0\">" << std::endl;
				if (isFailure()) {
					file << "<failure message=\"" << printVerdict() << "\"/>" << std::endl;
				}
			file << "</testcase>" << std::endl;
		file << "</testsuite>" << std::endl;
	file << "</testsuites>" << std::endl;

	// Close
	file.close();

};
