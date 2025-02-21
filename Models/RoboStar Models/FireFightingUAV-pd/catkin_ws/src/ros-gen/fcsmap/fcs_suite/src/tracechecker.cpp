
#include <vector>
#include <memory>
#include "event.hpp"
#include "message.hpp"
#include "verdict.hpp"
#include "tracechecker.hpp"

TraceChecker::TraceChecker() {};
TraceChecker::TraceChecker(std::vector<std::shared_ptr<Message>>::iterator _it,
			 std::vector<std::shared_ptr<Message>>::iterator _end,
			 Verdict _verdict) {
  it = _it;
  end = _end;
  verdict = _verdict;
};
TraceChecker TraceChecker::initially(std::vector<std::shared_ptr<Message>> vec) {
  // Initially the verdict is inconclusive.
  return TraceChecker { vec.begin(), vec.end(), Verdict::Inconclusive };
};
TraceChecker TraceChecker::then(std::shared_ptr<Event> ev) {
  if (valid() && (*it)->compare(ev)) { // It's not the end of the trace and the event matches
	return TraceChecker { ++it, end, Verdict::Inconclusive };
  } else { // It is the end of the trace or event does not match
	return TraceChecker { end, end, verdict };
  }
};
TraceChecker TraceChecker::pass(std::shared_ptr<Event> ev) {
  if (valid() && (*it)->compare(ev)) { // It's not the end of the trace and the event matches
	return TraceChecker { ++it, end, Verdict::Pass };
  } else { // It is the end of the trace or event does not match
	return TraceChecker { end, end, verdict };
  }
};
TraceChecker TraceChecker::pass() { // Set pass manually.
  return TraceChecker { it, end, Verdict::Pass };
};
TraceChecker TraceChecker::forbid(std::shared_ptr<Event> ev) {
  if (valid() && (*it)->compare(ev)) { // It's not the end of the trace and the event matches
	return TraceChecker { ++it, end, Verdict::Fail };
  } else { // It is the end of the trace or event does not match
	return TraceChecker { end, end, verdict };
  }
};
Verdict TraceChecker::getVerdict() {
  return verdict;
};
