#ifndef OPTION_LIST_H
#define OPTION_LIST_H
#include "list.h"

typedef struct
{
    char *key;
    char *val;
    int used;
} kvp;


int read_option(char *s, clist *options);
void option_insert(clist *l, char *key, char *val);
char *option_find(clist *l, char *key);
float option_find_float(clist *l, char *key, float def);
float option_find_float_quiet(clist *l, char *key, float def);
void option_unused(clist *l);

#endif
