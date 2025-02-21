
#include <iostream>
#include "message.hpp"

bool LTHAN(std::shared_ptr<Message> m1, std::shared_ptr<Message> m2) {
//  std::cout << "m1: " << m1->getTime().value().sec << ":" << m1->getTime().value().nsec << " m2: " << m2->getTime().value().sec << ":" << m2->getTime().value().nsec << std::endl;
//  if ((!(m2->getTime().has_value())) || (m1->getTime() < m2->getTime())) {
//	  std::cout << "LTHAN: true" << std::endl;
//  } else {
//	  std::cout << "LTHAN: false" << std::endl;
//  }
  return (!(m2->getTime().has_value())) || (m1->getTime() < m2->getTime());
};
