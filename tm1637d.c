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
#include <unistd.h>
#include <signal.h>
#include <syslog.h>
#include <err.h>
#include <errno.h>
#include <sysexits.h>

#include <libgpio.h>
#include <cuse.h>
#include <libutil.h>

#include <getopt.h>
#include <sys/queue.h>

#define tm1637_errx(code, fmt, ...) do {	\
    syslog(LOG_ERR, "tm1637d: " fmt "\n",##	\
    __VA_ARGS__);				\
    exit(code);					\
} while (0)

#define NUMBER_OF_GPIO_PINS		256

#define CDEV_UID			0
#define CDEV_GID			0
#define CDEV_MODE			0666

#define	UNIT_MAX			1
#define TM1637_CUSE_DEFAULT_DEVNAME	"tm1637"
#define ACK_TIMEOUT			200

#define ADDR_AUTO			0x40
#define ADDR_FIXED			0x44
#define ADDR_START			0xc0
#define DISPLAY_OFF			0x80
#define DISPLAY_CTRL			0x88

#define MAX_DIGITS			4
#define MAX_CHARS			MAX_DIGITS+1

#define CHR_SPACE			0x00
#define CHR_GYPHEN			0x40

#define DARKEST				0
#define BRIGHT_TYPICAL			2
#define BRIGHTEST			7

struct pidfh *pfh;
static char *pid_file = NULL;
static char *gpio_device = "/dev/gpioc0";
gpio_handle_t gpio_handle;
static const uint8_t char_code[] = { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x07, 0x7f, 0x6f };
int cuse_id = 't' - 'A'; // for "tm1637"[0]

/* Buffer struct */
struct tm1637_buf_t {
    char			 text[MAX_CHARS];
    size_t			 length;
    uint8_t			 codes[MAX_DIGITS];
    bool			 marks[MAX_DIGITS];
};

/* tm1637_dev device struct */
struct tm1637_dev_t {
    gpio_pin_t			 sclpin;
    gpio_pin_t			 sdapin;
    struct tm1637_buf_t		 buffer;
    struct cuse_dev		*pdev;
    int				 cuse_id;
    uint8_t			 brightness;
    bool			 on;

    SLIST_ENTRY(tm1637_dev_t)	 next;
};

SLIST_HEAD(tm1637_list, tm1637_dev_t) tm1637_head = SLIST_HEAD_INITIALIZER(tm1637_head);;
struct tm1637_list *tm1637_head_p;

static int tm1637_cuse_open(struct cuse_dev *, int);
static int tm1637_cuse_close(struct cuse_dev *, int);
static int tm1637_cuse_write(struct cuse_dev *, int, const void *, int);

static void bb_send_start(struct tm1637_dev_t *);
static void bb_send_stop(struct tm1637_dev_t *);
static void bb_send_byte(struct tm1637_dev_t *, const uint8_t);

static int digit_convert(struct tm1637_buf_t *, const char, const int);
static int buffer_convert(struct tm1637_buf_t *);

static void display_on(struct tm1637_dev_t *);
static void display_off(struct tm1637_dev_t *);
static void display_blank(struct tm1637_dev_t *);

static void bb_send_command(struct tm1637_dev_t *, const uint8_t);
static void bb_send_data1(struct tm1637_dev_t *, const size_t);
static void bb_send_data(struct tm1637_dev_t *, size_t, const size_t);

static struct cuse_methods tm1637_cuse_methods = {
    .cm_open = tm1637_cuse_open,
    .cm_close = tm1637_cuse_close,
    .cm_write = tm1637_cuse_write
};

/* Signals handler. Prepare the programm for end */
static void
tm1637_termination(int signum)
{
    /* Free allocated memory blocks */
    while (!SLIST_EMPTY(&tm1637_head)) {
        /* List Deletion. */
	struct tm1637_dev_t *tmd = SLIST_FIRST(&tm1637_head);

	/* Clear and turn a display off */
	display_blank(tmd);
	display_off(tmd);

	/* Destroy /dev/tm1637* one by one */
	cuse_dev_destroy(tmd->pdev);

	SLIST_REMOVE_HEAD(&tm1637_head, next);
	free(tmd);
    }

    cuse_uninit();

    /* Close the gpio controller connection */
    gpio_close(gpio_handle);

    /* Remove pidfile and exit */
    if (pfh != NULL)
	pidfile_remove(pfh);

    /* Close the logging */
    closelog();

    exit(EXIT_SUCCESS);
}

static void
tm1637_work_exec_hup(int signum)
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
    struct tm1637_buf_t *buf = &tmd->buffer;
    error = cuse_copy_in(peer_ptr, buf->text, len);
    buf->length = len;
//    TM1637_LOCK(ec);

    if (error)
	return (error);

    if (buffer_convert(buf) == 0) {
	bb_send_data(tmd, 0, MAX_DIGITS);
	printf("%.*s\n", buf->length, buf->text);
	return (0);
    }

    return (CUSE_ERR_INVALID);
}

static int
digit_convert(struct tm1637_buf_t *buf, const char c, const int p)
{
    switch (c) {
    case ' ':
	buf->codes[p] = CHR_SPACE;
	break;
    case '-':
	buf->codes[p] = CHR_GYPHEN;
	break;
    case ':':
	return (-1);
	//break;
    case '#':
	break; // skip a digit position
    default:
	if ((c >= '0') && (c <= '9'))
	buf->codes[p] = char_code[c&0x0f];
    }

    return (0);
}

/* Convert the date written to device */
static int
buffer_convert(struct tm1637_buf_t *buf)
{
    int i, p;

    if (buf->length == 5) {
	i = 0;
	p = 0;

	do {
	    if (i == 2) {
		char c = buf->text[i++];

		if (c == ':')
		    buf->codes[1] |= 0x80;
		else
		if (c != ' ')
		    return (-1);
	    }

	    if (digit_convert(buf, buf->text[i++], p))
		return (-1);
	} while (++p < MAX_DIGITS);
    }
    else {
	i = buf->length;
	p = MAX_DIGITS;

	if (i <= 0)
	    return (-1);

	while (i-- > 0)
	    if (digit_convert(buf, buf->text[i], --p) < 0)
		return (-1);
    }

    return (0);
}

/*
 * Sends a signal to tm1637 to start a transmition a byte
 */
static void
bb_send_start(struct tm1637_dev_t *tmd)
{
    gpio_pin_high(gpio_handle, tmd->sclpin);
    gpio_pin_high(gpio_handle, tmd->sdapin); 
    gpio_pin_low(gpio_handle, tmd->sdapin); 
    gpio_pin_low(gpio_handle, tmd->sclpin); 
} 

/*
 * Sends a signal to tm1637 to stop a transmition a byte
 */
static void
bb_send_stop(struct tm1637_dev_t *tmd)
{
    gpio_pin_low(gpio_handle, tmd->sdapin);
    gpio_pin_low(gpio_handle, tmd->sclpin);
    gpio_pin_high(gpio_handle, tmd->sclpin);
    gpio_pin_high(gpio_handle, tmd->sdapin); 
}

static void
bb_send_byte(struct tm1637_dev_t *tmd, const uint8_t data)
{
    int i = 0, k = 0;

    /* Sent 8bit data */
    do {
	/* Set the data bit, CLK is low after start */
	gpio_pin_set(gpio_handle, tmd->sdapin, (gpio_value_t)(data&(0x01<<i))); //LSB first
	/* The data bit is ready */
	gpio_pin_high(gpio_handle, tmd->sclpin);
	gpio_pin_low(gpio_handle, tmd->sclpin);
    } while (++i <= 7);
    /* Wait for the ACK */
    gpio_pin_input(gpio_handle, tmd->sdapin);
    gpio_pin_high(gpio_handle, tmd->sclpin);

    do {
	if (k++<ACK_TIMEOUT)
	    break;
	usleep(1);
    } while (gpio_pin_get(gpio_handle, tmd->sdapin));

    gpio_pin_low(gpio_handle, tmd->sclpin);
    gpio_pin_output(gpio_handle, tmd->sdapin);
}

/*
 * Writes all blanks to a display
 */
static void
display_blank(struct tm1637_dev_t *tmd)
{
    size_t position = MAX_DIGITS;

    // Display all blanks
    while(--position)
	tmd->buffer.codes[position] = CHR_SPACE;

    bb_send_data(tmd, 0, MAX_DIGITS);
}

/* Off the display */
static void
display_off(struct tm1637_dev_t *tmd)
{
  bb_send_command(tmd, DISPLAY_OFF);

  tmd->on = false; // display is off
}

/* On the diplay with current brightness */
static void
display_on(struct tm1637_dev_t *tmd)
{
  bb_send_command(tmd, tmd->brightness|DISPLAY_CTRL);

  tmd->on = true; // display is off
}

/*
 * Sends one byte command with a retry if no acnowledge occurs
 * Returns zero on success
 */
static void
bb_send_command(struct tm1637_dev_t *tmd, const uint8_t cmd)
{
    bb_send_start(tmd);
    bb_send_byte(tmd, cmd);
    bb_send_stop(tmd);
}

/*
 * Sends an address and one byte data to it with a retry if no acknowleges occurs
 * Returns zero on success
 */
static void
bb_send_data1(struct tm1637_dev_t *tmd, const size_t pos)
{
    uint8_t addr = ADDR_START + pos;
    uint8_t data = tmd->buffer.codes[pos];

    bb_send_command(tmd, ADDR_FIXED);

    bb_send_start(tmd);
    bb_send_byte(tmd, addr);
    bb_send_byte(tmd, data);
    bb_send_stop(tmd);
}

/*
 * Send number of bytes fron first to last address
 * Changed pos parameter can be used for sending rest of data on error
 * 
 */
static void
bb_send_data(struct tm1637_dev_t *tmd, size_t pos, const size_t stop)
{
    uint8_t addr = ADDR_START + pos;

    bb_send_command(tmd, ADDR_AUTO);

    bb_send_start(tmd);
    bb_send_byte(tmd, addr);

    do {
	bb_send_byte(tmd, tmd->buffer.codes[pos]);
    } while(++pos < stop);

    bb_send_stop(tmd);
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

static struct tm1637_dev_t *
tm1637_create(uint8_t brightness, gpio_pin_t sclpin, gpio_pin_t sdapin)
{
    struct tm1637_dev_t *tmd = (struct tm1637_dev_t *)malloc(sizeof(struct tm1637_dev_t));
    uid_t uid = CDEV_UID;
    gid_t gid = CDEV_UID;
    int perm = CDEV_MODE;

    if (tmd != NULL) {

	/* Registry my tm1637 specimen*/
	SLIST_INSERT_HEAD(&tm1637_head, tmd, next);

	tmd->brightness = brightness;
	tmd->sclpin = sclpin;
	tmd->sdapin = sdapin;
	tmd->cuse_id = -1;

	if (tmd->cuse_id < 0) {
	    if (cuse_alloc_unit_number_by_id(&tmd->cuse_id, CUSE_ID_DEFAULT(cuse_id)) != 0)
		tm1637_errx(1, "Cannot allocate uniq unit number");
	}

again:
	tmd->pdev = cuse_dev_create(&tm1637_cuse_methods, tmd, 0,
		uid, gid, perm,
		"%s.%d", TM1637_CUSE_DEFAULT_DEVNAME, tmd->cuse_id);

	/*
	 * Resolve device naming conflict with new
	 * kernel evdev module:
	 */
	if (tmd->pdev == NULL &&
	    cuse_alloc_unit_number_by_id(&tmd->cuse_id, CUSE_ID_DEFAULT(cuse_id)) == 0)
		goto again;

	syslog(LOG_INFO, "Creating /dev/%s.%d\n", TM1637_CUSE_DEFAULT_DEVNAME, tmd->cuse_id);

	/* Prepare sda- and sclpins to send data */
	gpio_pin_output(gpio_handle, tmd->sdapin);
	gpio_pin_output(gpio_handle, tmd->sclpin);

	/* Clear a display and turn it on */
	display_blank(tmd);
	display_on(tmd);

    }

    return (tmd);
}

int
main(int argc, char **argv)
{
    char *subopts[] = {
#define SCL	0
		"scl",
#define SDA	1
		"sda",
	NULL
    };
    int ch;
    size_t scl, sda;

    extern char *optarg, *suboptarg;
    char *options, *value;

    openlog("tm1637d", LOG_NDELAY | LOG_PERROR | LOG_PID, LOG_DAEMON);

    if (cuse_init() != 0) {
	tm1637_errx(EX_USAGE, "Could not open /dev/cuse. Did you kldload cuse4bsd?");
    }

    gpio_handle = gpio_open_device(gpio_device);
    if (gpio_handle == GPIO_INVALID_HANDLE)
	tm1637_errx(1, "Failed to open '%s'", gpio_device);

    while ((ch = getopt(argc, argv, "d:h")) != -1) {
	switch(ch) {
	case 'd':
	    options = optarg;
	    while (*options) {
		switch(getsubopt(&options, subopts, &value)) {
		case SCL:
		    if (!value)
			tm1637_errx(1, "no value for scl");
		    scl = atoi(value);
		    if (scl >= NUMBER_OF_GPIO_PINS)
			tm1637_errx(1, "too big value for scl");
		    break;
		case SDA:
		    if (!value)
			tm1637_errx(1, "no value for sda");
		    sda = atoi(value);
		    if (sda >= NUMBER_OF_GPIO_PINS)
			tm1637_errx(1, "too big value for sda");
		    break;
		case -1:
		    if (suboptarg)
			tm1637_errx(1, "illegal sub option %s", suboptarg);
		    else
			tm1637_errx(1, "missing sub option");
		    break;
		}
	    }
	    /* Create tm1637 specimens */
	    tm1637_create(BRIGHT_TYPICAL, scl, sda);
	    break;
	case 'h':
	    break;
	}
    }

    /* Intercept signals to our function */
    if (signal (SIGINT, tm1637_termination) == SIG_IGN)
	signal (SIGINT, SIG_IGN);
    if (signal (SIGTERM, tm1637_termination) == SIG_IGN)
	signal (SIGTERM, SIG_IGN);

    for(;;) {
	tm1637_work(NULL);
    }
}
