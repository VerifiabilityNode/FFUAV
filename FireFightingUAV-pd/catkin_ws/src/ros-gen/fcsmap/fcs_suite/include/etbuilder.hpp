
#ifndef _H_ETBUILDER
#define _H_ETBUILDER

#include <memory>
#include "event.hpp"
#include "tester.hpp"

#define E(type,...) std::make_shared<type>(__VA_ARGS__)

class EventTraceBuilder {
  protected:

	Tester* tester;

	EventTraceBuilder add(std::shared_ptr<Event> ev) {
	  trace.push_back(ev);
	  return EventTraceBuilder{trace,tester};
	};
  public:
	std::vector<std::shared_ptr<Event>> trace;

	EventTraceBuilder() {};
	EventTraceBuilder(std::vector<std::shared_ptr<Event>> _trace) { trace = _trace; };
	EventTraceBuilder(std::vector<std::shared_ptr<Event>> _trace, Tester* _tester) { trace = _trace; tester = _tester; };

	EventTraceBuilder then(std::shared_ptr<Event> ev) {
	  if (tester != nullptr) {
		  tester->setup(ev);
	  }
	  return add(ev);
	};

	void setTester(Tester* t) {
		tester = t;
	};
};

#endif /* _H_ETBUILDER */
