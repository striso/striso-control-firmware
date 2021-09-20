#ifndef _CONFIGSTORE_H_
#define _CONFIGSTORE_H_

#include "ch.h"

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
int getConfigHex(const char* name);
void strset(char* key, unsigned int start, const char* str);

extern const int button_number_map[61];

void put_button_name(int n, char* key);

#endif