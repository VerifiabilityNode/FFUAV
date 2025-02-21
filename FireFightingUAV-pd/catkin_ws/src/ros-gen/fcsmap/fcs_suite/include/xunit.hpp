
#ifndef _H_XUNIT
#define _H_XUNIT

#include "verdict.hpp"
#include <string>

class XUnit {

  private:
	std::string path;
	Verdict verdict;
  protected:
	bool isFailure() {
		return verdict == Verdict::Fail;
	};
	std::string printVerdict() {
		switch (verdict) {
			case Verdict::Inconclusive:
			  return "Verdict: inconclusive";
			case Verdict::Pass:
			  return "Verdict: pass";
			case Verdict::Fail:
			  return "Verdict: fail";
			default:
			  return "Unknown";
		}
	};
  public:
	void setVerdict(Verdict _verdict) {
		verdict = _verdict;
	};
	void setPath(std::string _path) {
		path = _path;
	};
	void writeOutput();

};

#endif /* !_H_XUNIT */
