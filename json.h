#ifndef JSON_H_
#define JSON_H_

#include <initializer_list>

#include "format.h"

class JsonArray;
class JsonObject;

class JsonFormatter {
  friend class JsonArray;
  friend class JsonObject;

public:
  JsonFormatter(char *buf, std::size_t len);
  template <std::size_t N> explicit JsonFormatter(char (&buf)[N]) : f_(buf) {}

  JsonArray array();
  JsonObject object();

  Formatter &formatter();
  const Formatter &formatter() const;

private:
  Formatter f_;
  bool array_;
  bool object_;
};

class JsonArray {
  friend class JsonFormatter;
  friend class JsonObject;

public:
  ~JsonArray();

  template <class... Args> void push(Args &&...args) {
    std::initializer_list<int> _{(push(std::forward<Args>(args)), 0)...};
  }

  void push(int x);
  void push(double x);
  void push(double x, int precision);
  void push(const char *s);
  void push(bool x);
  void push(std::nullptr_t);
  JsonArray pushArray();
  JsonObject pushObject();

private:
  explicit JsonArray(JsonFormatter &json);

  void appendComma();

  JsonFormatter &json_;
  bool first_ = true;
};

class JsonObject {
  friend class JsonFormatter;
  friend class JsonArray;

public:
  ~JsonObject();

  void addMember(const char *key, int x);
  void addMember(const char *key, double x);
  void addMember(const char *key, double x, int precision);
  void addMember(const char *key, const char *s);
  void addMember(const char *key, bool x);
  void addMember(const char *key, std::nullptr_t);
  JsonArray addArrayMember(const char *key);
  JsonObject addSubobject(const char *key);

private:
  explicit JsonObject(JsonFormatter &json);

  void appendComma();

  JsonFormatter &json_;
  bool first_ = true;
};

#endif