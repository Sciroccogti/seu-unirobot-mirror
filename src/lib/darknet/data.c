#include "data.h"
#include "utils.h"
#include "image.h"
#include "cuda.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

clist *get_paths(char *filename)
{
    char *path;
    FILE *file = fopen(filename, "r");

    if (!file)
    {
        file_error(filename);
    }

    clist *lines = make_list();

    while ((path = fgetl(file)))
    {
        list_insert(lines, path);
    }

    fclose(file);
    return lines;
}

char **get_labels(char *filename)
{
    clist *plist = get_paths(filename);
    char **labels = (char **)list_to_array(plist);
    free_list(plist);
    return labels;
}

void free_data(data d)
{
    if (!d.shallow)
    {
        free_matrix(d.X);
        free_matrix(d.y);
    }
    else
    {
        free(d.X.vals);
        free(d.y.vals);
    }
}

matrix concat_matrix(matrix m1, matrix m2)
{
    int i, count = 0;
    matrix m;
    m.cols = m1.cols;
    m.rows = m1.rows + m2.rows;
    m.vals = calloc(m1.rows + m2.rows, sizeof(float *));

    for (i = 0; i < m1.rows; ++i)
    {
        m.vals[count++] = m1.vals[i];
    }

    for (i = 0; i < m2.rows; ++i)
    {
        m.vals[count++] = m2.vals[i];
    }

    return m;
}

void get_random_batch(data d, int n, float *X, float *y)
{
    int j;

    for (j = 0; j < n; ++j)
    {
        int index = rand() % d.X.rows;
        memcpy(X + j * d.X.cols, d.X.vals[index], d.X.cols * sizeof(float));
        memcpy(y + j * d.y.cols, d.y.vals[index], d.y.cols * sizeof(float));
    }
}

void get_next_batch(data d, int n, int offset, float *X, float *y)
{
    int j;

    for (j = 0; j < n; ++j)
    {
        int index = offset + j;
        memcpy(X + j * d.X.cols, d.X.vals[index], d.X.cols * sizeof(float));

        if (y)
        {
            memcpy(y + j * d.y.cols, d.y.vals[index], d.y.cols * sizeof(float));
        }
    }
}

data get_data_part(data d, int part, int total)
{
    data p = {0};
    p.shallow = 1;
    p.X.rows = d.X.rows * (part + 1) / total - d.X.rows * part / total;
    p.y.rows = d.y.rows * (part + 1) / total - d.y.rows * part / total;
    p.X.cols = d.X.cols;
    p.y.cols = d.y.cols;
    p.X.vals = d.X.vals + d.X.rows * part / total;
    p.y.vals = d.y.vals + d.y.rows * part / total;
    return p;
}

