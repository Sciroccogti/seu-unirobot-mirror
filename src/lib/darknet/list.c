#include <stdlib.h>
#include <string.h>
#include "list.h"

clist *make_list()
{
	clist *l = malloc(sizeof(clist));
	l->size = 0;
	l->front = 0;
	l->back = 0;
	return l;
}


void *list_pop(clist *l){
    if(!l->back) return 0;
    node *b = l->back;
    void *val = b->val;
    l->back = b->prev;
    if(l->back) l->back->next = 0;
    free(b);
    --l->size;
    
    return val;
}

void list_insert(clist *l, void *val)
{
	node *new = malloc(sizeof(node));
	new->val = val;
	new->next = 0;

	if(!l->back){
		l->front = new;
		new->prev = 0;
	}else{
		l->back->next = new;
		new->prev = l->back;
	}
	l->back = new;
	++l->size;
}

void free_node(node *n)
{
	node *next;
	while(n) {
		next = n->next;
		free(n);
		n = next;
	}
}

void free_list(clist *l)
{
	free_node(l->front);
	free(l);
}

void free_list_contents(clist *l)
{
	node *n = l->front;
	while(n){
		free(n->val);
		n = n->next;
	}
}

void **list_to_array(clist *l)
{
    void **a = calloc(l->size, sizeof(void*));
    int count = 0;
    node *n = l->front;
    while(n){
        a[count++] = n->val;
        n = n->next;
    }
    return a;
}
