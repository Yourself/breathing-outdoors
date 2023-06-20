#ifndef FORMAT_H_
#define FORMAT_H_

#include <cstdint>
#include <cstdio>
#include <utility>

class Formatter {
public:
  Formatter(char *buf, std::size_t size) : buf_(buf), size_(size) {}

  Formatter(const Formatter &) = delete;
  Formatter &operator=(const Formatter &) = delete;

  template <std::size_t N> Formatter(char (&buf)[N]) : Formatter(buf, N) {}

  operator bool() const { return !failed(); }

  template <class... Args> Formatter &append(const char *fmt, Args &&...args) {
    if (failed_) {return *this;}
    int result = std::snprintf(buf_, size_, fmt, std::forward<Args>(args)...);
    if (0 <= result && result < size_) {
      buf_ += result;
      size_ -= result;
    } else {
      failed_ = true;
    }
    return *this;
  }

  char *peek() const { return buf_; }

  bool failed() const { return failed_; }

private:
  char *buf_;
  std::size_t size_;
  bool failed_ = false;
};

#endif