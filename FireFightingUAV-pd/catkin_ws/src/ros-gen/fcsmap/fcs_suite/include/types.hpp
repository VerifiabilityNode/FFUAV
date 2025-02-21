#ifndef _H_TYPE_DECLS
#define _H_TYPE_DECLS

#include <tuple>
#include <variant>

namespace NavCommand {
	
	class TakeOff {
	  public:
		
		friend bool operator ==(const TakeOff& lhs, const TakeOff& rhs) {
			return std::tie() == std::tie();
		}
	};
	class GoHome {
	  public:
		
		friend bool operator ==(const GoHome& lhs, const GoHome& rhs) {
			return std::tie() == std::tie();
		}
	};
	class Land {
	  public:
		
		friend bool operator ==(const Land& lhs, const Land& rhs) {
			return std::tie() == std::tie();
		}
	};
	
	typedef std::variant<NavCommand::TakeOff,NavCommand::GoHome,NavCommand::Land> NavCommand;
}
class Battery {
  public:
  	unsigned int percentage;
  	
  	friend bool operator ==(const Battery& lhs, const Battery& rhs) {
  		return std::tie(lhs.percentage) == std::tie(rhs.percentage);
  	}
};
namespace M600Task {
	
	class GoHome {
	  public:
		
		friend bool operator ==(const GoHome& lhs, const GoHome& rhs) {
			return std::tie() == std::tie();
		}
	};
	class Land {
	  public:
		
		friend bool operator ==(const Land& lhs, const Land& rhs) {
			return std::tie() == std::tie();
		}
	};
	class TakeOff {
	  public:
		
		friend bool operator ==(const TakeOff& lhs, const TakeOff& rhs) {
			return std::tie() == std::tie();
		}
	};
	
	typedef std::variant<M600Task::GoHome,M600Task::Land,M600Task::TakeOff> M600Task;
}
namespace M600Action {
	
	class Start {
	  public:
		
		friend bool operator ==(const Start& lhs, const Start& rhs) {
			return std::tie() == std::tie();
		}
	};
	class Stop {
	  public:
		
		friend bool operator ==(const Stop& lhs, const Stop& rhs) {
			return std::tie() == std::tie();
		}
	};
	class Pause {
	  public:
		
		friend bool operator ==(const Pause& lhs, const Pause& rhs) {
			return std::tie() == std::tie();
		}
	};
	class Resume {
	  public:
		
		friend bool operator ==(const Resume& lhs, const Resume& rhs) {
			return std::tie() == std::tie();
		}
	};
	
	typedef std::variant<M600Action::Start,M600Action::Stop,M600Action::Pause,M600Action::Resume> M600Action;
}

#endif /* _H_TYPE_DECLS */
