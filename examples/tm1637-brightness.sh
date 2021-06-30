#!/bin/sh

# If LEVEL in 0..7 the display turns on with this level of brightness
# In another case the display turns off
LEVEL=$1
DEVICE=/dev/tm1637/0

if [ -z ""${LEVEL} ] || [ ${LEVEL} -lt 0 ] || [ ${LEVEL} -gt 7 ]; then
    # Octal 0200 equals to hexadecimal 0x80 (turn off)
    printf "\200" > ${DEVICE}
else
    # Octal 021n equals to hexadecimal 0x88 + n (turn on and set brightness n)
    printf "\21${LEVEL}" > ${DEVICE}
fi
