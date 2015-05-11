/*
 * error.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: antonio
 */

#include "exception.h"

namespace mas {

exception::exception(std::string msg, std::exception_ptr cause) {
    this->_msg = msg;
    this->_cause = cause;
}

const std::exception_ptr exception::cause() const {
    return _cause;
}

const char* exception::what() const noexcept(true) {
    return _msg.c_str();
}

exception::~exception() noexcept {
}


} /* namespace mas */
