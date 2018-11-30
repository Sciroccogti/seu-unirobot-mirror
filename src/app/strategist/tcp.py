#!/usr/bin/env python3
#coding: utf-8

class task_type:
    TASK_NONE = 0
    TASK_WALK = 1
    TASK_ACT = 2
    TASK_LOOK = 3
    TASK_LED = 4

class tcp_cmd_type:
    REG_DATA = 2
    WM_DATA = 15
    TASK_DATA = 20

class tcp_data_dir:
    DIR_BOTH = 0
    DIR_APPLY = 1
    DIR_SUPPLY = 2

class tcp_size:
    enum_size = 4
    int_size = 4
    float_size = 4
    bool_size = 1
    head_size = 9

tcp_cmd_fmt = {
    tcp_cmd_type.WM_DATA: '=i?Ifffff',
    tcp_cmd_type.TASK_DATA: '=i?Ii'
}