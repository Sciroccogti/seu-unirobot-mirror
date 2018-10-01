#ifndef DATA_H
#define DATA_H
#include <pthread.h>

#include "darknet.h"
#include "matrix.h"
#include "list.h"
#include "image.h"
#include "tree.h"

void get_random_batch(data d, int n, float *X, float *y);
data get_data_part(data d, int part, int total);

#endif
