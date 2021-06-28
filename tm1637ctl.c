/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (C) 2021, Alexander Mishin
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/types.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "tm1637d.h"

static void
usage()
{
    fprintf(stderr, "usage: %s -t <0|1> -l <0..7> [-d <device>] [-h]\n\n",
	getprogname());
    fprintf(stderr,
    "Optioqns:\n"
	"    -d, --device <device>\n"
	"                        Define a tm1637-device name for control;\n"
	"    -t, --turn <0|1>    Turn a display on or off;\n"
	"    -l, --brightness <0..7>\n"
	"                        Set a brightness level;\n"
	"    -h, --help          Print this help.\n"
    );
}

int
main(int argc, char **argv)
{
    static int dev;
    static char *dev_tm1637 = "/dev/tm1637";

    static int ch, long_index = 0;
    static unsigned long value;
    static uint8_t turn = 255, level = 255;
    extern char *optarg, *end;

    static struct option long_options[] = {
	{"device",     required_argument, 0, 'd' },
	{"turn",       required_argument, 0, 't' },
	{"brightness", required_argument, 0, 'l' },
	{0, 0, 0, 0}
    };

    while ((ch = getopt_long(argc, argv, "t:l:d:h", long_options, &long_index)) != -1) {
	switch(ch) {
	case 'd':
	    dev_tm1637 = optarg;
	    break;
	case 't':
	    value = strtoul(optarg, &end, 0);
	    if (value <= 1)
		turn = value;
	    break;
	case 'l':
	    value = strtoul(optarg, &end, 0);
	    if (value <= 7)
		level = value;
	    break;
	case 'h':
	    /* FALLTHROUGH */
	default:
	    usage();
	    exit(EXIT_SUCCESS);
	}
    }

    dev = open(dev_tm1637, O_WRONLY | O_DIRECT);
    if (dev < 0) {
	perror("opening tm1637 device");
	exit(EXIT_FAILURE);
    }

    if (level <= 7)
	ioctl(dev, TM1637IOC_SET_BRIGHTNESS, &level);

    if (turn == 1)
	ioctl(dev, TM1637IOC_ON);
    else if (turn == 0)
	ioctl(dev, TM1637IOC_OFF);

    close(dev);
}
