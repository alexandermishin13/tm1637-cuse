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
#include <stdlib.h>
#include <signal.h>
#include <syslog.h>
#include <err.h>
#include <sysexits.h>

#include <libgpio.h>
#include <cuse.h>
#include <libutil.h>

#define tm1637_errx(code, fmt, ...) do {	\
    syslog(LOG_ERR, "tm1637d: " fmt "\n",##	\
    __VA_ARGS__);				\
    exit(code);					\
} while (0)

/* tm1637_dev device struct */
struct tm1637_dev_t {
};

static int tm1637_cuse_open(struct cuse_dev *, int);
static int tm1637_cuse_close(struct cuse_dev *, int);
static int tm1637_cuse_write(struct cuse_dev *, int, const void *, int);

static struct cuse_methods tm1637_cuse_methods = {
    .cm_open = tm1637_cuse_open,
    .cm_close = tm1637_cuse_close,
    .cm_write = tm1637_cuse_write
};

static void
tm1637_work_exec_hup(int dummy)
{

}

static void *
tm1637_work(void *arg)
{
    signal(SIGHUP, tm1637_work_exec_hup);

    while (1) {
	if (cuse_wait_and_process() != 0)
	    break;
    }

    /* we are done */
    exit(0);

    return (NULL);
}

static int
tm1637_cuse_open(struct cuse_dev *cdev, int fflags)
{
    return (0);
}

static int
tm1637_cuse_close(struct cuse_dev *cdev, int fflags)
{
    return (0);
}

static int
tm1637_cuse_write(struct cuse_dev *cdev, int fflags, const void *peer_ptr, int len)
{
    struct tm1637_dev_t *tmd = cuse_dev_get_priv0(cdev);

    return (0);
}

int
main(int argc, char **argv)
{
    if (cuse_init() != 0) {
	tm1637_errx(EX_USAGE, "Could not open /dev/cuse. Did you kldload cuse4bsd?");
    }

    for(;;) {
	tm1637_work(NULL);
    }
}
