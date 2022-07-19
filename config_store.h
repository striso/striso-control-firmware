#ifndef _CONFIGSTORE_H_
#define _CONFIGSTORE_H_

#include "ch.h"

#define CONFIG_UNDEFINED 100000000

typedef struct {
    char key[8];
    char value[8];
} ConfigParam;
extern const ConfigParam default_config[];
// extern ConfigParam* const flash_config;
#define flash_config ((ConfigParam*)0x08020000)

const char* getConfigSetting(const char* name);
float getConfigFloat(const char* name);
int getConfigInt(const char* name);
unsigned int getConfigHex(const char* name);
void strset(char* key, unsigned int start, const char* str);
bool cmp8(const char* a, const char* b);

extern const int button_number_map[61];

void put_button_name(int n, char* key);

#endif