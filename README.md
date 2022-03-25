[Brief]
    This repository contains the source code to build/parse these of type formats (including control frame and type data frame)
[Usage]
    Just build only a file main.c with GCC

Examples:
    Control frame:
    | 0xA55A | 0xA1 | 0x0001 | 0xF1 | 0xAF68 |
    Data frame::
    | 0xA55A | 0xA2 | 0x0005 | 0x68 0x65 0x6C 0x6C 0x6F | 0x94E3 |


[1] https://github.com/lammertb/libcrc