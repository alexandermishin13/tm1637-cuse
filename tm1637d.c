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
#include <stdio.h>
#include <signal.h>
#include <syslog.h>
#include <err.h>
#include <errno.h>
#include <sysexits.h>

#include <libgpio.h>
#include <cuse.h>
#include <libutil.h>

#define tm1637_errx(code, fmt, ...) do {	\
    syslog(LOG_ERR, "tm1637d: " fmt "\n",##	\
    __VA_ARGS__);				\
    exit(code);					\
} while (0)

#define CDEV_UID			0
#define CDEV_GID			0
#define CDEV_MODE			666

#define	UNIT_MAX			1
#define TM1637_CUSE_DEFAULT_DEVNAME	"tm1637"

#define MAX_DIGITS			4
#define MAX_CHARS			MAX_DIGITS+1

#define CHR_SPACE			0x00
#define CHR_HYPHEN			0x40

struct pidfh *pfh;
static char *pid_file = NULL;
static const uint8_t char_code[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f };

/* tm1637_dev device struct */
struct tm1637_dev_t {
    uint8_t	 codes[MAX_DIGITS];
    char	 digits[MAX_DIGITS];
    char	 buf_text[MAX_CHARS];
    size_t	 buf_len;
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
    int error;

    if ((len == 0) || (len > MAX_CHARS))
	return (CUSE_ERR_INVALID);

//    TM1637_UNLOCK(ec);
    error = cuse_copy_in(peer_ptr, tmd->buf_text, len);
    tmd->buf_len = len;
//    TM1637_LOCK(ec);

    if (error)
	return (error);

    printf("%.*s\n", tmd->buf_len, tmd->buf_text);

    return (len);
}

static void
tm1637_exit(void)
{
    if (pfh != NULL) {
	pidfile_remove(pfh);
	pfh = NULL;
    }
    closelog();
}

/* Daemonize wrapper */
static void
daemonize(void)
{
    pid_t otherpid;

    /* Try to create a pidfile */
    pfh = pidfile_open(pid_file, 0600, &otherpid);
    if (pfh == NULL) {
	if (errno == EEXIST)
	    tm1637_errx (EXIT_FAILURE, "Daemon already running, pid: %jd.", (intmax_t)otherpid);

	/* If we cannot create pidfile from other reasons, only warn. */
	warn ("Cannot open or create pidfile");
	/*
	* Even though pfh is NULL we can continue, as the other pidfile_*
	* function can handle such situation by doing nothing except setting
	* errno to EDOOFUS.
	*/
    }

    /* Try to demonize the process */
    if (daemon(0, 0) == -1) {
	pidfile_remove(pfh);
	tm1637_errx (EXIT_FAILURE, "Cannot daemonize");
    }

    pidfile_write(pfh);
}

static void
tm1637_create(void)
{
    int id;
    unsigned int n = 0;
    int unit_num[UNIT_MAX] = { -1 };
    struct cuse_dev *pdev;

    struct tm1637_dev_t *tmd;

    for (n = 0; n < UNIT_MAX; n++) {

	if ((tmd = malloc(sizeof(*tmd))) != NULL) {

	    if (unit_num[id] < 0) {
		if (cuse_alloc_unit_number_by_id(&unit_num[id], CUSE_ID_DEFAULT(id)) != 0)
		    tm1637_errx(1, "Cannot allocate uniq unit number");
	    }

again:
	    pdev = cuse_dev_create(&tm1637_cuse_methods, tmd, 0,
		CDEV_UID, CDEV_GID, CDEV_MODE,
		"%s.%d", TM1637_CUSE_DEFAULT_DEVNAME, unit_num[id]);

	    /*
	     * Resolve device naming conflict with new
	     * kernel evdev module:
	     */
	    if (pdev == NULL &&
		cuse_alloc_unit_number_by_id(&unit_num[id], CUSE_ID_DEFAULT(id)) == 0)
		    goto again;

	    syslog(LOG_INFO, "Creating /dev/%s.%d\n", TM1637_CUSE_DEFAULT_DEVNAME, unit_num[id]);
	}
    }
}

int
main(int argc, char **argv)
{

    openlog("tm1637d", LOG_NDELAY | LOG_PERROR | LOG_PID, LOG_DAEMON);

    if (cuse_init() != 0) {
	tm1637_errx(EX_USAGE, "Could not open /dev/cuse. Did you kldload cuse4bsd?");
    }

    tm1637_create();

    for(;;) {
	tm1637_work(NULL);
    }
}
