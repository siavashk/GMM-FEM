#ifndef MEX_HANDLE
#define MEX_HANDLE

// Define types
#ifdef _MSC_VER
typedef __int32 int32_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int64 uint64_t;
#else
#include <stdint.h>
#endif
#include <utility>

// Thanks to Oliver Wood and discussions:
// http://www.mathworks.com/matlabcentral/newsreader/view_thread/278243

namespace mex {

template<class base> class class_handle: public base
{
    public:
	template<typename... Args>
        class_handle(uint32_t sig, Args&&... args): base(std::forward<Args>(args)...) { signature = sig; }
        ~class_handle() { signature = 0; }
        bool isValid(uint32_t sig) { return (signature == sig); }
    
    private:
        uint32_t signature;
};

template<class base> inline mxArray *get_mex_handle(class_handle<base> *ptr)
{
    mxArray *out = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
    *((uint64_t *)mxGetData(out)) = reinterpret_cast<uint64_t>(ptr);
    return out;
}

template<class base> inline class_handle<base> *get_class_handle(uint32_t sig, const mxArray *in)
{
    if (mxGetNumberOfElements(in) != 1 || mxGetClassID(in) != mxUINT64_CLASS || mxIsComplex(in))
        mexErrMsgTxt("Input must be a real uint64 scalar.");
    class_handle<base> *ptr = reinterpret_cast<class_handle<base> *>(*((uint64_t *)mxGetData(in)));
    if (!ptr->isValid(sig))
        mexErrMsgTxt("Handle not valid.");
    return ptr;
}

template<class base> class class_holder
{
public:
     class_holder(uint32_t sig, const base &data) {
		signature = sig;
		this->data = data;
     }
     ~class_holder() { signature = 0; }
     bool isValid(uint32_t sig) { return (signature == sig); }
     base &getData() {
    	 return data;
     }

    private:
        uint32_t signature;
        base data;
};

template<class base> inline mxArray *get_mex_handle(class_holder<base> *ptr)
{
    mxArray *out = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
    *((uint64_t *)mxGetData(out)) = reinterpret_cast<uint64_t>(ptr);
    return out;
}

template<class base> inline class_holder<base> *get_class_holder(uint32_t sig, const mxArray *in)
{
    if (mxGetNumberOfElements(in) != 1 || mxGetClassID(in) != mxUINT64_CLASS || mxIsComplex(in))
        mexErrMsgTxt("Input must be a real uint64 scalar.");
    class_holder<base> *ptr = reinterpret_cast<class_holder<base> *>(*((uint64_t *)mxGetData(in)));
    if (!ptr->isValid(sig))
        mexErrMsgTxt("Handle not valid.");
    return ptr;
}

}

#endif
