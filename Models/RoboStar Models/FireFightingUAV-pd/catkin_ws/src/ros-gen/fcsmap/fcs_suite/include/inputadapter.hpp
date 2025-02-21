#ifndef _H_INPUTADAPTER
#define _H_INPUTADAPTER

#include "ros/ros.h"
#include "executor.hpp"
#include <thread>

template <typename M>
class InputAdapter {
  public:
    virtual bool provideInput(M input) { return false; };
};

template <typename M>
class TopicInputAdapter : public InputAdapter<M> {
  protected:
    ros::Publisher *_publisher;
  public:
    TopicInputAdapter(ros::Publisher *publisher) {
        _publisher = publisher;
    };
    bool provideInput(M p) {
        _publisher->publish(p);
        return true;
    };
};

template <typename C, typename R>
class ServiceInputAdapter : public InputAdapter<C> {
  protected:
    ros::ServiceClient *_service;
    TExecutor* _texec;
  public:
    ServiceInputAdapter(ros::ServiceClient *service, TExecutor* texec) {
        _service = service;
        _texec = texec;
    };
    bool provideInput(C request) override {
      // The default implementation is blocking. This should not be used.
      R response;
      return _service->call(request,response);
    };
};

template <typename C, typename R, typename M>
class AsyncServiceResponseInputAdapter : public ServiceInputAdapter<C,R> {
  public:
    using ServiceInputAdapter<C,R>::ServiceInputAdapter;

    bool provideInput(C request) override {
      std::thread { [this,request]() {
          R response; // FIXME: Allocate on the heap?

          if (this->_service != nullptr) {
            if (this->_service->call(request,response)) {
              std::shared_ptr<M> msg = std::make_shared<M>(&response);
              #ifdef DEBUG
                ROS_INFO_STREAM("Called service, now pending response.");
              #endif
              this->_texec->getTTMessage(msg);
            } else {
              #ifdef DEBUG
                ROS_INFO_STREAM("Failure in calling service.");
              #endif
            }
          }
        }
      }.detach();
      return true;
    }
};

template <typename C, typename R>
class AsyncServiceInputAdapter : public ServiceInputAdapter<C,R> {
  public:
    using ServiceInputAdapter<C,R>::ServiceInputAdapter;

    bool provideInput(C request) override {
      std::thread { [this,request]() {
          R response; // FIXME: Allocate on the heap?

          if (this->_service != nullptr) {
            if (this->_service->call(request,response)) {
              #ifdef DEBUG
                ROS_INFO_STREAM("Called callService, forget response.");
              #endif
            } else {
              #ifdef DEBUG
                ROS_INFO_STREAM("Failure in calling service.");
              #endif
            }
          }
        }
      }.detach();
      return true;
    }
};

#endif /* !_H_INPUTADAPTER */
