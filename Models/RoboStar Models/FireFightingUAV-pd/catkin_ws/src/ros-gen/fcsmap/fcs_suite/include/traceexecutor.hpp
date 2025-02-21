#ifndef _H_TRACE_EXECUTOR
#define _H_TRACE_EXECUTOR

#include <memory>
#include <vector>
#include <mutex>
#include <condition_variable>
#include "event.hpp"
#include "executor.hpp"
#include "message.hpp"

class TraceExecutor : public TExecutor {
  protected:
	bool _shutdown = false;

	// The test trace to be executed, in full.
	std::vector<std::shared_ptr<Event>> eventTrace;
	std::vector<std::shared_ptr<Event>>::iterator currEvent;
	std::vector<std::shared_ptr<Event>>::iterator finalEvent;

	// The trace of messages seen so far.
	std::vector<std::shared_ptr<Message>> messageTrace;

	// This will be used to keep track of the current position in
	// the message vector. We cannot use iterators given that the
	// vector will be sorted.
	std::vector<std::shared_ptr<Message>>::size_type m = 0;

	// Variables regarding execution of an event trace.
	bool published;

	// Mutex to keep access to vectors synchronised.
	std::mutex lock;
	std::condition_variable cv;

	bool handled;

	// Methods
	bool validEventTrace();
  public:
	void setTrace(std::vector<std::shared_ptr<Event>> _vec);
	void put(std::shared_ptr<Message> msg);
	void printMessageTrace();
	void getSMessage(std::shared_ptr<Message> input);
	void getSTMessage(std::shared_ptr<Message> input);
	void getTTMessage(std::shared_ptr<Message> input);
	void update();
	void shutdown();
	std::vector<std::shared_ptr<Message>> getMessages();
};

#endif /* _H_TRACE_EXECUTOR */
