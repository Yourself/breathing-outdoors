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

  template <class... Args> int append(const char *fmt, Args &&...args) {
    int result = std::snprintf(buf_, size_, fmt, std::forward<Args>(args)...);
    if (result < 0)
      return -1;
    if (result >= size_)
      return -1;
    buf_ += result;
    size_ -= result;
    return result;
  }

  char *peek() const { return buf_; }

private:
  char *buf_;
  std::size_t size_;
};

#endif