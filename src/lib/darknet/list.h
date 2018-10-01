#ifndef LIST_H
#define LIST_H
#include "darknet.h"

clist *make_list();
int list_find(clist *l, void *val);

void list_insert(clist *, void *);


void free_list_contents(clist *l);

#endif
