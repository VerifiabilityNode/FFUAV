#ifndef _H_EXECUTOR
#define _H_EXECUTOR

#include <memory>
#include "message.hpp"

class TExecutor {
  public:
	virtual void put(std::shared_ptr<Message> msg) {};
	virtual void printMessageTrace() {};
	virtual void getSMessage(std::shared_ptr<Message> input) {};
	virtual void getSTMessage(std::shared_ptr<Message> input) {};
	virtual void update() {};
};

#endif /* _H_EXECUTOR */
