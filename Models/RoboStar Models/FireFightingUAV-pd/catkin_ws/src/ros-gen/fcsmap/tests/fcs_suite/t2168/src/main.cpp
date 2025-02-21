#include <memory>
#include "ros/ros.h"
#include "etbuilder.hpp"
#include "events.hpp"
#include "traceexecutor.hpp"
#include "message.hpp"
#include "verdict.hpp"
#include "tester.hpp"
#include "types.hpp"
#include "primitivetypes.hpp"
#include "tracechecker.hpp"

#include "xunit.hpp"
#include <regex>
#include <boost/program_options.hpp>

const double TIMEOUT_SEC = 0.5;
const double FINAL_TIMEOUT = 0.5;

std::vector<std::string> splitGTest(const std::string& input) {
    std::regex re(":");
    std::sregex_token_iterator
        first{input.begin(), input.end(), re, -1},
        last;
    return {first, last};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "t2168");
	
	// Tester
	Tester test;
	
	// Setup rostest
	XUnit xunit;
	bool gtestEnabled = false;
	bool text = false;
	bool noReadyWait = false;
	std::string xunitPath;
	
	namespace po = boost::program_options;
	po::options_description desc("Options");
	desc.add_options()
		("gtest_output", po::value<std::string>(&xunitPath))
		("text", po::bool_switch(&text))
		("noready", po::bool_switch(&noReadyWait));
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);
	
	if (vm.count("gtest_output")) {
		std::vector<std::string> strs = splitGTest(xunitPath);
		if (strs.size() == 2) {
			#ifdef DEBUG
			ROS_INFO_STREAM("gtest_output at: " << strs[1]);
			#endif
			xunit.setPath(strs[1]);
			gtestEnabled = true;
		}
	}
	
	// Setup tester
	EventTraceBuilder builder;
	builder.setTester(&test);
	auto trace = builder.then(E(fcActivation,std::tuple<>{})).then(E(fcActivationRetIN,true)).then(E(fcControlAuth,std::tuple<>{})).then(E(fcControlAuthRetIN,true)).then(E(navCommandIN,NavCommand::GoHome())).then(E(fcMissionWPUpload,std::tuple<std::vector<Position>>{{0,3}}));
	
	// Test loop
	if (!noReadyWait) {
		#ifdef DEBUG
			ROS_INFO_STREAM("Waiting for connections to be ready.");
		#endif
		
		test.ready();
		#ifdef DEBUG
			ROS_INFO_STREAM("Ready now.");
		#endif
	}
	
	test.setTrace(trace.trace);
	
	// Spin at least number of possible concurrent callbacks being handled at the same time.
	ros::MultiThreadedSpinner spinner(26);
	
	// Test loop
	#ifdef DEBUG
		ROS_INFO_STREAM("Spining.");
	#endif
	spinner.spin();
	
	std::cout << "Stopped ROS node. Checking trace." << std::endl;
	
	TraceChecker checker;
	checker = checker.initially(test.getMessages()).then(E(fcActivation,std::tuple<>{})).then(E(fcActivationRetIN,true)).then(E(fcControlAuth,std::tuple<>{})).then(E(fcControlAuthRetIN,true)).pass(E(navCommandIN,NavCommand::GoHome())).forbid(E(fcMissionWPUpload,std::tuple<std::vector<Position>>{{0,3}}));
	
	if (gtestEnabled) {
		xunit.setVerdict(checker.getVerdict());
		xunit.writeOutput();
	}
	
	switch (checker.getVerdict()) {
	    case Verdict::Inconclusive:
	      std::cout << "Verdict: inconclusive" << std::endl;
	      return 0;
	    case Verdict::Pass:
	      std::cout << "Verdict: pass" << std::endl;
	      return 0;
	    case Verdict::Fail:
	      std::cout << "Verdict: fail" << std::endl;
	      return -1;
	    default:
	      break;
	}
	
	return 0;
}

