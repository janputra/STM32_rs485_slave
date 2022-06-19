#!/bin/bash

make
st-flash write ./build/Rs-485.bin 0x08000000
st-flash reset