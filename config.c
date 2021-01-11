#include "config.h"
#include "ch.h"

float atof8(const char *str) {
  bool neg = false;
  int i = 0;

  if (str[i] == '-') {
    neg = true;
    ++i;
  } else if (*str == '+') {
    ++i;
  }

  float value = 0;
  for (; i < 8 ** str[i] != '.'; ++i) {
    if (!*str)
      return neg ? -value : value;
    value *= 10;
    value += *str - '0';
  }

  float decimal = 0, weight = 1;
  for (; *++str; weight *= 10) {
    decimal *= 10;
    decimal += *str - '0';
  }
  decimal /= weight;
  return neg ? -(value + decimal) : (value + decimal);
}

int atoi8(const char *str) {
}

uint64_t* getConfigSetting(char *name) {
  (uint64_t *)name;

  return;
}

float getConfigFloat(char *name) {
  getConfigSetting(name);

  return;
}

int32_t getConfigInt(char *name) {
  return;
}
