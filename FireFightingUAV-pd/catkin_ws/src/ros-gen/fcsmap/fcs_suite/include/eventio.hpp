#ifndef _H_EVENTIO
#define _H_EVENTIO

#include "event.hpp"
#include "inputadapter.hpp"
#include <memory>

template <typename V>
class TypedIO {
  protected:
	V value;
  public:
	TypedIO(V _value) { value = _value; };
};

template <typename M>
class InputEvent : public Event {
  protected:
	std::shared_ptr<InputAdapter<M>> _adapter;
  public:
	using Event::getName;
	void setAdapter(std::shared_ptr<InputAdapter<M>> adapter) {
		_adapter = adapter;
	};
	void publish() override {
		if (_adapter != nullptr) {
			_adapter->provideInput(getROSMessage());
		}
	};
	bool isSUTInput() const override { return true; }; // Could also use type system instead.
	virtual M getROSMessage() = 0;
};

template <typename V, typename M>
class TypedInputEvent : public TypedIO<V>, public InputEvent<M> {
  public:
	// Constructors
    using TypedIO<V>::TypedIO;
    using Event::getName;
};

template <typename V>
class TypedResponseInputEvent : public TypedIO<V>, public Event {
  public:
	// Constructors
	using TypedIO<V>::TypedIO;
	using Event::getName;
};

class OutputEvent : public Event {

};

template <typename V>
class TypedOutputEvent : public TypedIO<V>, public Event {
  public:
	// Constructors
  	using TypedIO<V>::TypedIO;
  	using Event::getName;
};

#endif /* !_H_EVENTIO */
