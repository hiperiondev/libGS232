#!/bin/bash

valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --verbose  Release/libGS232 -g function-within-function-bugfix.tb
