#include "ch.h"
#include "config.h"

/* Button names */
// key number = 17 * buttons[but].coord0 + 10 * buttons[but].coord1 + 30
const int button_number_map[61] = {
  56, 51, 63, 58, 53, 65, 60, 55, 67, 62, 40, 35, 47, 42, 37, 49, 44,
  39, 34, 46, 41, 36, 48, 43, 38, 50, 45, 23, 18, 30, 25, 20, 32, 27,
  22, 17, 29, 24, 19, 31, 26, 21, 33, 28,  6,  1, 13,  8,  3, 15, 10,
   5,  0, 12,  7,  2, 14,  9,  4, 16, 11};

// const char note_names[2][17] = {"C_","Db","C#","D_","Eb","D#","E_",
//     "F_","Gb","F#","G_","Ab","G#","A_","Bb","A#","B_"};
const char note_names[] = "C_DbC#D_EbD#E_F_GbF#G_AbG#A_BbA#B_";

void put_button_name(int n, char* key) {
  int note = (n + 7) % 17;
  int oct = (n + 7) / 17;
  key[0] = note_names[note*2];
  key[1] = note_names[note*2+1];
  key[2] = '1' + oct;
}

void strset(char* key, unsigned int start, const char* str) {
  for (int n = start; n < 8 && str[n-start]; n++) {
    key[n] = str[n-start];
  }
}

float atof8(const char* str) {
  bool neg = false;
  int i = 0;

  if (str[i] == '-') {
    neg = true;
    ++i;
  } else if (str[i] == '+') {
    ++i;
  }

  float value = 0;
  while (i < 8 && str[i] != '.') {
    if (str[i] < '0' || str[i] > '9')
      return neg ? -value : value;
    value *= 10;
    value += str[i] - '0';
    ++i;
  }

  float weight = 0.1;
  while (i < 8 && str[i] >= '0' && str[i] <= '9') {
    value += weight * (str[i] - '0');
    weight *= 0.1;
  }
  return neg ? -value : value;
}

int atoi8(const char* str) {
  bool neg = false;
  int i = 0;

  if (str[i] == '-') {
    neg = true;
    ++i;
  } else if (str[i] == '+') {
    ++i;
  }

  int value = 0;
  while (i < 8 && str[i] >= '0' && str[i] <= '9') {
    value *= 10;
    value += str[i] - '0';
    ++i;
  }

  return neg ? -value : value;
}

int atox8(const char* str) {
  int i = 0;

  int value = 0;
  while (i < 8) {
    if (str[i] >= '0' && str[i] <= '9') {
      value *= 16;
      value += str[i] - '0';
      ++i;
    }
    else if (str[i] >= 'A' && str[i] <= 'F') {
      value *= 16;
      value += str[i] - 'A' + 10;
      ++i;
    }
    else if (str[i] >= 'a' && str[i] <= 'f') {
      value *= 16;
      value += str[i] - 'a' + 10;
      ++i;
    }
    else {
      break;
    }
  }

  return value;
}

inline
bool cmp8(const char* a, const char* b) {
  return *((uint64_t *)a) == *((uint64_t *)b);
}

const char* getConfigSetting(const char* name) {
  const ConfigParam* cfg = flash_config;
  while (!cmp8(cfg->key, "MCfgEnd ") && cfg->key[0] >= 32 && cfg->key[0] < 127) {
    if (cmp8(cfg->key, name))
      return cfg->value;
    cfg++;
  }

  cfg = default_config;
  while (!cmp8(cfg->key, "MCfgEnd ") && cfg->key[0] >= 32 && cfg->key[0] < 127) {
    if (cmp8(cfg->key, name))
      return cfg->value;
    cfg++;
  }

  return "(ERROR!)";
}

float getConfigFloat(const char* name) {
  return atof8(getConfigSetting(name));
}

int getConfigInt(const char* name) {
  return atoi8(getConfigSetting(name));
}

int getConfigHex(const char* name) {
  return atox8(getConfigSetting(name));
}
