#include "json.h"

JsonFormatter::JsonFormatter(char *buf, std::size_t len) : f_(buf, len) {}

JsonArray JsonFormatter::array() {
  f_.append("[");
  return JsonArray(*this);
}
JsonObject JsonFormatter::object() {
  f_.append("{");
  return JsonObject(*this);
}

Formatter &JsonFormatter::formatter() { return f_; }
const Formatter &JsonFormatter::formatter() const { return f_; }

JsonArray::~JsonArray() { json_.f_.append("]"); }

void JsonArray::push(int x) {
  appendComma();
  json_.f_.append("%d", x);
}

void JsonArray::push(double x) {
  appendComma();
  json_.f_.append("%f", x);
}

void JsonArray::push(double x, int precision) {
  appendComma();
  json_.f_.append("%.*f", precision, x);
}

void JsonArray::push(const char *s) {
  appendComma();
  json_.f_.append("\"%s\"", s);
}

void JsonArray::push(bool x) {
  appendComma();
  json_.f_.append(x ? "true" : "false");
}

void JsonArray::push(std::nullptr_t) {
  appendComma();
  json_.f_.append("null");
}

JsonArray JsonArray::pushArray() {
  appendComma();
  json_.f_.append("[");
  return JsonArray(json_);
}

JsonObject JsonArray::pushObject() {
  appendComma();
  json_.f_.append("{");
  return JsonObject(json_);
}

JsonArray::JsonArray(JsonFormatter &json) : json_(json) {}

void JsonArray::appendComma() {
  if (!first_) {
    json_.f_.append(",");
  }
  first_ = false;
}

JsonObject::~JsonObject() { json_.f_.append("}"); }

void JsonObject::addMember(const char *key, int x) {
  appendComma();
  json_.f_.append("\"%s\":%d", key, x);
}

void JsonObject::addMember(const char *key, double x) {
  appendComma();
  json_.f_.append("\"%s\":%f", key, x);
}

void JsonObject::addMember(const char *key, double x, int precision) {
  appendComma();
  json_.f_.append("\"%s\":%.*f", key, precision, x);
}

void JsonObject::addMember(const char *key, const char *s) {
  appendComma();
  json_.f_.append("\"%s\":%s", key, s);
}

void JsonObject::addMember(const char *key, bool x) {
  appendComma();
  json_.f_.append(x ? "\"%s\":true" : "\"%s\":false", key);
}

void JsonObject::addMember(const char *key, std::nullptr_t) {
  appendComma();
  json_.f_.append("\"%s\":null", key);
}

JsonArray JsonObject::addArrayMember(const char *key) {
  appendComma();
  json_.f_.append("\"%s\":[", key);
  return JsonArray(json_);
}

JsonObject JsonObject::addSubobject(const char *key) {
  appendComma();
  json_.f_.append("\"%s\":{", key);
  return JsonObject(json_);
}

JsonObject::JsonObject(JsonFormatter &json) : json_(json) {}

void JsonObject::appendComma() {
  if (!first_) {
    json_.f_.append(",");
  }
  first_ = false;
}
