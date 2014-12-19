#ifndef MAS_EXCEPTION_H_
#define MAS_EXCEPTION_H_

#include <string>
#include <exception>

namespace mas {

class exception: public std::exception {
private:
    std::string _msg;
    std::exception_ptr _cause;

public:
    exception(std::string msg, std::exception_ptr cause = nullptr);
    const std::exception_ptr cause() const;
    virtual const char* what() const noexcept(true);
    virtual ~exception() noexcept;
};

} // mas

#endif
