#ifndef DRIVING_COMMON_EXCEPTION_H_
#define DRIVING_COMMON_EXCEPTION_H_

#include <string>

struct InvalidArgumentEx {};
struct ZeroPointerArgumentEx {};

struct IOEx {};
struct FileIOEx {};
struct SocketIOEx {};

class BaseException {
 public:
  BaseException(const std::string& error = std::string())
      : error_message_(error) {}

  virtual ~BaseException() {}

  const std::string& errorMessage() const { return error_message_; }
  const std::string& what() const { return error_message_; }

 protected:
  std::string error_message_;
};

struct EmptyEx {};

template <class t1 = EmptyEx, class t2 = EmptyEx, class t3 = EmptyEx>
struct Ex : public Ex<t2, t3, EmptyEx> {
  Ex(const std::string& error = std::string()) : Ex<t2, t3>(error) {}
};

template <>
struct Ex<EmptyEx, EmptyEx, EmptyEx> : public BaseException {
  Ex(const std::string& error = std::string()) : BaseException(error) {}
};

template <class t1>
struct Ex<t1, EmptyEx, EmptyEx> : public Ex<EmptyEx, EmptyEx, EmptyEx> {
  Ex(const std::string& error = std::string()) : Ex<>(error) {}
};

template <class t1, class t2>
struct Ex<t1, t2, EmptyEx> : public Ex<t2, EmptyEx, EmptyEx> {
  Ex(const std::string& error = std::string()) : Ex<t2>(error) {}
};

#endif
