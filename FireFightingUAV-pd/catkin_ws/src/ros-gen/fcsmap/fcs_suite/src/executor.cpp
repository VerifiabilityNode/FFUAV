
#include "event.hpp"
#include "traceexecutor.hpp"

bool TraceExecutor::validEventTrace() {
      return currEvent < eventTrace.end();
};

void TraceExecutor::setTrace(std::vector<std::shared_ptr<Event>> _vec) {
    m = 0;
    published = false;
    handled = true;
    eventTrace = _vec;
    currEvent = eventTrace.begin();
    finalEvent = eventTrace.end();
};
   
// // call/response?
// bool next() {
//   ROS_INFO_STREAM("Next called.");
//   if (validEventTrace()) {
//     bool result = (*currEvent)->publish();
//     ROS_INFO_STREAM("Called publish.");
//     ++currEvent;
//     return result;
//   } else {
//     return true;
//   }
// };

/*
 * This method puts a new message in the messageTrace vector and
 * sorts the vector according to the time stamps.
 *
 * This method is to be called by any call back handling topics.
 */
void TraceExecutor::put(std::shared_ptr<Message> msg) {

    std::lock_guard<std::mutex> locker(lock);
    ROS_INFO_STREAM("Got lock, saving message and sorting.");
    messageTrace.push_back(msg);
    std::sort(messageTrace.begin(),messageTrace.end(),LTHAN);
    // After a new message is received into this 'buffer',
    // we should be delaying the 'get' by a fixed amount.
    //
    // -> A timer thread will periodically call a method
    //    of this class and notify waiting threads that
    //    they can be updated. A client needs to get hold
    //    of the lock mutex in order to manipulate the
    //    vector, but how do we ensure it reads the latest value?

};

void TraceExecutor::printMessageTrace() {
    for (auto i = messageTrace.begin(); i < messageTrace.end(); i++) {
    ROS_INFO_STREAM("mT: " << (*i)->getFullName());
    }
}

/*
    * This method waits until a message of a compatible type is
    * the current message in messageTrace vector. Then, it returns
    * 
    */
void TraceExecutor::getSMessage(std::shared_ptr<Message> input) {

    while (true) {
		std::unique_lock<std::mutex> locker(lock);
		if (_shutdown || ros::isShuttingDown()) {
			ROS_INFO_STREAM("Shutting down. Returning from getSMessage.");
			return;
		}
		cv.wait(locker);
		// Each client call gets to wait here.
		ROS_INFO_STREAM("Woke up from get SMessage.");

		if (_shutdown || ros::isShuttingDown()) {
			ROS_INFO_STREAM("Shutting down. Returning from getSMessage.");
			return;
		}
		// After being notified, we pick the current position in the vector
		// and check whether the 'msg' at that position corresponds to the input.
		try {
			auto msg = messageTrace.at(m);
			ROS_INFO_STREAM("Head SMessage is: " << msg->getName() << " and input is " << input->getName());
			auto in = msg->match(input);

			if (in) {
				#ifdef DEBUG
				if (validEventTrace()) {
					ROS_INFO_STREAM("comparing current input message " << input->getName() << " with current event " << (*currEvent)->getName());
				}
				#endif

				if (validEventTrace() && input->compare(*currEvent)) {
				ROS_INFO_STREAM("Compared and handled");
				// If the current trace message and the input match, then we step.

				// This is fine for a request, but for a response, we need to get the value.
				currEvent++;
				m++;
				handled = true;
				return;
				}
			}
			// if (in) {
			//   ROS_INFO_STREAM("Message matches.");
			//   // If the current trace message and the input match, then we replace it.
			//   //messageTrace.at(m) = in.second;
			// }
		} catch (std::out_of_range const& exc) {

		}
    }
};

void TraceExecutor::getSTMessage(std::shared_ptr<Message> input) {
    // But then we return an Event<X> of an unknown type? Can't do that,
    // but we know all possible types to be returned per each ServiceMessage
    // type?
    while (true) {
		std::unique_lock<std::mutex> locker(lock);

		if (_shutdown || ros::isShuttingDown()) {
			ROS_INFO_STREAM("Shutting down. Returning from getSTMessage.");
			return;
		}

		cv.wait(locker);
		// Each client call gets to wait here.
		ROS_INFO_STREAM("Woke up from get STMessage.");

		if (_shutdown || ros::isShuttingDown()) {
			ROS_INFO_STREAM("Shutting down. Returning from getSTMessage.");
			return;
		}

		// After being notified, we pick the current position in the vector
		// and check whether the 'msg' at that position corresponds to the input.
		try {
			auto msg = messageTrace.at(m);
			ROS_INFO_STREAM("Head STMessage is: " << msg->getName() << " and input is " << input->getName());
			auto in = msg->match(input);

			if (in && validEventTrace() && input->compare(*currEvent)) {
			ROS_INFO_STREAM("Compared and handled");
			messageTrace.at(m) = in.value();
			// If the current trace message and the input match, then we step.
			handled = true;
			return;
			}
		} catch (std::out_of_range const& exc) {
			if (validEventTrace() && input->compare(*currEvent)) { // We are beyond the end of the messageTrace, but the input message matches the trace.
			ROS_INFO_STREAM("New matching message.");
			// messageTrace.push_back(input);
			// m++;
			handled = true;
			return;
			}
		}
    }
};

void TraceExecutor::getTTMessage(std::shared_ptr<Message> input) {
    // But then we return an Event<X> of an unknown type? Can't do that,
    // but we know all possible types to be returned per each ServiceMessage
    // type?
    while (true) {
		std::unique_lock<std::mutex> locker(lock);

		if (_shutdown || ros::isShuttingDown()) {
			ROS_INFO_STREAM("Shutting down. Returning from getTTMessage.");
			return;
		}

		cv.wait(locker);
		// Each client call gets to wait here.
		ROS_INFO_STREAM("Woke up from get TTMessage.");

		if (_shutdown || ros::isShuttingDown()) {
			ROS_INFO_STREAM("Shutting down. Returning from getTTMessage.");
			return;
		}
		// After being notified, we pick the current position in the vector
		// and check whether the 'msg' at that position corresponds to the input.
		try {
			auto msg = messageTrace.at(m);
			ROS_INFO_STREAM("Head TTMessage is: " << msg->getName() << " and input is " << input->getName());
			auto in = msg->match(input);

			if (in && validEventTrace()) {
				ROS_INFO_STREAM("(getTTMessage): in && validEventTrace()");
				messageTrace.at(m) = input;

				if (input->compare(*currEvent)) {
					ROS_INFO_STREAM("(getTTMessage): event compares OK");
					currEvent++;
					m++;
					handled = true;
					return;
				}
			// && input->compare(*currEvent)) {
			// ROS_INFO_STREAM("Compared and handled");
			// messageTrace.at(m) = input;//in.value();
			// // If the current trace message and the input match, then we step.
			}
		} catch (std::out_of_range const& exc) {
			// if (input->compare(*currEvent)) { // We are beyond the end of the messageTrace, but the input message matches the trace.
			// ROS_INFO_STREAM("New matching message.");
			// // messageTrace.push_back(input);
			// // m++;
			// handled = true;
			// return;
			// }
		}
    }
};

std::vector<std::shared_ptr<Message>> TraceExecutor::getMessages() {
    return messageTrace;
}

void TraceExecutor::shutdown() {

    std::lock_guard<std::mutex> locker(lock);
    ROS_INFO_STREAM("Got lock, shutting down.");

    _shutdown = true;
    ROS_INFO_STREAM("Notifying thread clients of shutdown.");
	cv.notify_all();
	ROS_INFO_STREAM("Notified all thread clients of shutdown.");
	ros::requestShutdown();
}

// This should be called periodically
void TraceExecutor::update() {

    std::unique_lock<std::mutex> locker(lock);
    printMessageTrace();

    if (_shutdown || ros::isShuttingDown()) {
		ROS_INFO_STREAM("It's shutting down.");
		ros::requestShutdown();
		return;
	}

    if (handled) {
		if (validEventTrace() && (*currEvent)->isSUTInput()) {
			ROS_INFO_STREAM("Publishing current event: " << (*currEvent)->getName());
			(*currEvent)->publish();
		}
		handled = false;
		ROS_INFO_STREAM("Handled false.");
    } else {
		ROS_INFO_STREAM("Notifying thread clients.");
		cv.notify_all();

    // Check whether we can proceed with trace here.
    // try {
    //   auto msg = messageTrace.at(m);

    //   ROS_INFO_STREAM("Head is: " << msg->getName());

    //   if (msg->compare(*currEvent)) {
    //     ROS_INFO_STREAM("Compared ok.");
    //     // If the current trace message and the input match, then we step.
    //     currEvent++;
    //     m++;
    //     handled = true;
    //   }
    // } catch (std::out_of_range const& exc) {
        
    // }
    }
      // get the lock
      // change flag
      // notify_all()

      // Check if the flag has been changed, if so, it means
      // at least one message was 'handled'. If not, we proceed
      // through the trace, if we can!
};
