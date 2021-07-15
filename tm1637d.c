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
#include <pthread.h>

#include <libgpio.h>
#include <cuse.h>
#include <libutil.h>

#include <getopt.h>
#include <sys/queue.h>

#include "tm1637d.h"

#define tm1637_errx(code, fmt, ...) do {	\
    syslog(LOG_ERR, "tm1637d: " fmt "\n",##	\
    __VA_ARGS__);				\
    exit(code);					\
} while (0)

#define MIN(x,y)			((x) < (y) ? (x) : (y))
#define NUMBER_OF_GPIO_PINS		256

#define CDEV_UID			0
#define CDEV_GID			0
#define CDEV_MODE			0666

#define TM1637_CUSE_DEFAULT_DEVNAME	"tm1637"
#define ACK_TIMEOUT			200

#define MAX_DIGITS			4
#define MAX_CHARS			MAX_DIGITS+2

#define DARKEST				0
#define BRIGHT_DARK			1
#define BRIGHT_TYPICAL			2
#define BRIGHTEST			7

struct pidfh *pfh;
static char *pid_file = NULL;
static char *gpio_device = "/dev/gpioc0";
static gpio_handle_t gpio_handle;
static bool background = false;
static const int cuse_id = 't' - 'A'; // for "tm1637"[0]

static const uint8_t char_code[10] = {
    CHR_0, CHR_1, CHR_2, CHR_3, CHR_4,
    CHR_5, CHR_6, CHR_7, CHR_8, CHR_9
};

/* Buffer struct */
struct tm1637_buf_t {
    unsigned char		 text[MAX_CHARS];
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
    uid_t			 uid;
    gid_t			 gid;
    int				 perm;
    pthread_mutex_t		 lock;
    uint8_t			 brightness;
    bool			 on;

    SLIST_ENTRY(tm1637_dev_t)	 next;

#define	TM1637_LOCK(s)		pthread_mutex_lock(&(s)->lock)
#define	TM1637_UNLOCK(s)	pthread_mutex_unlock(&(s)->lock)
};

SLIST_HEAD(tm1637_list, tm1637_dev_t) tm1637_head = SLIST_HEAD_INITIALIZER(tm1637_head);;
struct tm1637_list *tm1637_head_p;

static int tm1637_cuse_open(struct cuse_dev *, int);
static int tm1637_cuse_close(struct cuse_dev *, int);
static int tm1637_cuse_write(struct cuse_dev *, int, const void *, int);
static int tm1637_cuse_ioctl(struct cuse_dev *, int, unsigned long, void *);

static void bb_send_start(struct tm1637_dev_t *);
static void bb_send_stop(struct tm1637_dev_t *);
static void bb_send_byte(struct tm1637_dev_t *, const uint8_t);

static int digit_convert(uint8_t *, const unsigned char);
static int buffer_convert(struct tm1637_buf_t *);

static void tm1637_display_on(struct tm1637_dev_t *);
static void tm1637_display_off(struct tm1637_dev_t *);
static void tm1637_display_blank(struct tm1637_dev_t *);
static void tm1637_set_brightness(struct tm1637_dev_t *, const uint8_t);
static void tm1637_set_clock(struct tm1637_dev_t *, const struct tm1637_clock_t);

static void bb_send_command(struct tm1637_dev_t *, const uint8_t);
static void bb_send_data1(struct tm1637_dev_t *, const size_t);
static void bb_send_data(struct tm1637_dev_t *, size_t, const size_t);

static uint8_t is_raw_command(struct tm1637_dev_t *);

static struct cuse_methods tm1637_cuse_methods = {
    .cm_open = tm1637_cuse_open,
    .cm_close = tm1637_cuse_close,
    .cm_write = tm1637_cuse_write,
    .cm_ioctl = tm1637_cuse_ioctl
};

static void
usage()
{
    fprintf(stderr, "usage: %s -d code=<code>,pin=<number> ... "
	"[-b] [-h]\n\n",
	getprogname());
    fprintf(stderr,
    "Optioqns:\n"
	"    -d, --device scl=<pin>,sda=<pin>\n"
	"                        Create a device with defined 'scl' and 'sda'\n"
	"                        pins. Can be used multiple times;\n"
	"    -b,                 Run in background as a daemon;\n"
	"    -h, --help          Print this help.\n"
    );
}

/* Signals handler. Prepare the programm for end */
static void
tm1637_termination(int signum)
{
    /* Free allocated memory blocks */
    while (!SLIST_EMPTY(&tm1637_head)) {
        /* List Deletion. */
	struct tm1637_dev_t *tmd = SLIST_FIRST(&tm1637_head);

	/* Clear and turn a display off */
	tm1637_display_blank(tmd);
	tm1637_display_off(tmd);

	pthread_mutex_destroy(&tmd->lock);

	/* Destroy /dev/tm1637* one by one */
	cuse_dev_destroy(tmd->pdev);
	syslog(LOG_INFO, "Destroyed /dev/%s/%d\n", TM1637_CUSE_DEFAULT_DEVNAME, tmd->cuse_id);

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

    if ((is_raw_command(tmd)) == 0)
	return (len);

    if (buffer_convert(buf) == 0) {
	bb_send_data(tmd, 0, MAX_DIGITS);
	return (len);
    }

    return (CUSE_ERR_INVALID);
}

static int
tm1637_cuse_ioctl(struct cuse_dev *cdev, int fflags,
    unsigned long cmd, void *peer_data)
{
    struct tm1637_dev_t *tmd = cuse_dev_get_priv0(cdev);
    int error = 0;
    struct tm1637_clock_t time;
    uint8_t brightness;

    switch (cmd) {
    case TM1637IOC_CLEAR:
	tm1637_display_blank(tmd);
	break;
    case TM1637IOC_OFF:
	tm1637_display_off(tmd);
	break;
    case TM1637IOC_ON:
	tm1637_display_on(tmd);
	break;
    case TM1637IOC_SET_BRIGHTNESS:
	error = cuse_copy_in(peer_data, &brightness, sizeof(brightness));
	if (error != CUSE_ERR_NONE)
	    break;
	tm1637_set_brightness(tmd, brightness);
	break;
    case TM1637IOC_SET_CLOCK:
	error = cuse_copy_in(peer_data, &time, sizeof(time));
	if (error != CUSE_ERR_NONE)
	    break;
	tm1637_set_clock(tmd, time);
	break;
    default:
	/* Not supported. */
	return (CUSE_ERR_INVALID);
    }

    return (error);
}

static uint8_t
is_raw_command(struct tm1637_dev_t *tmd)
{
    size_t start, stop, length = tmd->buffer.length;
    uint8_t *text = &tmd->buffer.text[0];
    uint8_t tmp;

    switch (text[0]) {
    /* Send one byte at a fixed position */
    case ADDR_FIXED:
	if ((length < 3) ||
	   ((tmp = text[1]^ADDR_START) >= MAX_DIGITS))
	    return (-1);

	tmd->buffer.codes[tmp] = text[2];
	bb_send_data1(tmd, tmp);
	break;
    /* Send up to 4 bytes at an autoincremented position */
    case ADDR_AUTO:
	if ((length < 3) ||
	   ((tmp = text[1]^ADDR_START) >= MAX_DIGITS))
	    return (-1);

	start = tmp;
	stop = MIN(length-2, MAX_DIGITS);
	for (size_t i=2; tmp<stop; tmp++)
	    tmd->buffer.codes[tmp] = text[i++];

	bb_send_data(tmd, start, stop);
	break;
    /* Send one byte command to turn display off */
    case DISPLAY_OFF:
	tm1637_display_off(tmd);
	break;
    default:
	/* Send one byte command to light display with the bright level 0..7 */
	if ((tmp = (text[0]^DISPLAY_CTRL)) <= BRIGHTEST) {
	    tmd->brightness = tmp;
	    tm1637_display_on(tmd);
	    break;
	}
	return (-1);
    }

    return (0);
}

static int
digit_convert(uint8_t *code, const unsigned char c)
{
    switch (c) {
    case ' ':
	*code = CHR_SPACE;
	break;
    case '-':
	*code = CHR_GYPHEN;
	break;
    case ':':
	return (-1);
	//break;
    case '#':
	break; // skip a digit position
    default:
	if ((c >= '0') && (c <= '9'))
	    *code = char_code[c&0x0f];
	else
	    return (-1);
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
		unsigned char c = buf->text[i++];

		if (c == ':')
		    buf->codes[1] |= 0x80;
		else
		if (c == ' ')
		    buf->codes[1] &= 0x7f;
		else
		    return (-1);
	    }

	    if (digit_convert(&buf->codes[p], buf->text[i++]))
		return (-1);
	} while (++p < MAX_DIGITS);
    }
    else {
	i = buf->length;
	p = MAX_DIGITS;

	if (i <= 0)
	    return (-1);

	while (i-- > 0)
	    if (digit_convert(&buf->codes[--p], buf->text[i]) < 0)
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
tm1637_display_blank(struct tm1637_dev_t *tmd)
{
    size_t position = MAX_DIGITS;
    uint8_t *codes = &tmd->buffer.codes[0];

    // Display all blanks
    while(--position)
	codes[position] = CHR_SPACE;

    bb_send_data(tmd, 0, MAX_DIGITS);
}

/* Off the display */
static void
tm1637_display_off(struct tm1637_dev_t *tmd)
{
    bb_send_command(tmd, DISPLAY_OFF);

    tmd->on = false; // display is off
}

/* On the diplay with current brightness */
static void
tm1637_display_on(struct tm1637_dev_t *tmd)
{
    if (!tmd->on) {
	/* First set flag for send data correctly */
	tmd->on = true;
	bb_send_data(tmd, 0, MAX_DIGITS);
    }
    /* Light the display anyway */
    bb_send_command(tmd, tmd->brightness|DISPLAY_CTRL);
}

/*
 * Sets a display on with a brightness value as parameter
 */
static void
tm1637_set_brightness(struct tm1637_dev_t *tmd, const uint8_t brightness)
{
    /* If brightness is really changed */
    if ((brightness != tmd->brightness) &&
        (brightness <= BRIGHTEST))
    {
	tmd->brightness = brightness;
	/* Only change a variable if a display is not on */
	if(tmd->on)
	    bb_send_command(tmd, tmd->brightness|DISPLAY_CTRL);
    }
}

/*
 * Sets time to the display
 */
static void
tm1637_set_clock(struct tm1637_dev_t *tmd, const struct tm1637_clock_t clock)
{
    int t;
    uint8_t *codes = &tmd->buffer.codes[0];

    /* Four clock digits */
    t = clock.tm_hour / 10;
    codes[0] = char_code[t&0x0f];
    t = clock.tm_hour % 10;
    codes[1] = char_code[t];
    t = clock.tm_min / 10;
    codes[2] = char_code[t&0x0f];
    t = clock.tm_min % 10;
    codes[3] = char_code[t];

    /* Clockpoint */
    if (clock.tm_colon)
	codes[1] |= 0x80;

    bb_send_data(tmd, 0, MAX_DIGITS);
}

/*
 * Sends one byte command with a retry if no acnowledge occurs
 * Returns zero on success
 */
static void
bb_send_command(struct tm1637_dev_t *tmd, const uint8_t cmd)
{
    TM1637_LOCK(tmd);

    bb_send_start(tmd);
    bb_send_byte(tmd, cmd);
    bb_send_stop(tmd);

    TM1637_UNLOCK(tmd);
}

/*
 * Sends an address and one byte data to it with a retry if no acknowleges occurs
 * Returns zero on success
 */
static void
bb_send_data1(struct tm1637_dev_t *tmd, const size_t pos)
{
    uint8_t addr = ADDR_START|pos;
    uint8_t data = tmd->buffer.codes[pos];

    if (tmd->on) {
	TM1637_LOCK(tmd);

	bb_send_start(tmd);
	bb_send_byte(tmd, ADDR_FIXED);
	bb_send_stop(tmd);

	bb_send_start(tmd);
	bb_send_byte(tmd, addr);
	bb_send_byte(tmd, data);
	bb_send_stop(tmd);

	TM1637_UNLOCK(tmd);
    }
}

/*
 * Send number of bytes fron first to last address
 * Changed pos parameter can be used for sending rest of data on error
 * 
 */
static void
bb_send_data(struct tm1637_dev_t *tmd, size_t pos, const size_t stop)
{
    uint8_t addr = ADDR_START|pos;
    uint8_t *codes = &tmd->buffer.codes[0];

    if (tmd->on) {
	TM1637_LOCK(tmd);

	bb_send_start(tmd);
	bb_send_byte(tmd, ADDR_AUTO);
	bb_send_stop(tmd);

	bb_send_start(tmd);
	bb_send_byte(tmd, addr);
	for( ;pos < stop; pos++)
	    bb_send_byte(tmd, codes[pos]);
	bb_send_stop(tmd);

	TM1637_UNLOCK(tmd);
    }
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
tm1637_register(gpio_pin_t sclpin, gpio_pin_t sdapin, uint8_t brightness)
{
    struct tm1637_dev_t *tmd = (struct tm1637_dev_t *)malloc(sizeof(struct tm1637_dev_t));

    if (tmd != NULL) {
	tmd->sclpin = sclpin;
	tmd->sdapin = sdapin;
	tmd->cuse_id = -1;
	tmd->uid = CDEV_UID;
	tmd->gid = CDEV_UID;
	tmd->perm = CDEV_MODE;
	tmd->brightness = brightness;

	/* Register my tm1637 specimen*/
	SLIST_INSERT_HEAD(&tm1637_head, tmd, next);
    }

    return (tmd);
}

static struct tm1637_dev_t *
tm1637_create(struct tm1637_dev_t *tmd)
{
    if (tmd != NULL) {

	if (tmd->cuse_id < 0) {
	    if (cuse_alloc_unit_number_by_id(&tmd->cuse_id, CUSE_ID_DEFAULT(cuse_id)) != 0)
		tm1637_errx(1, "Cannot allocate uniq unit number");
	}

again:
	tmd->pdev = cuse_dev_create(&tm1637_cuse_methods, tmd, 0,
		tmd->uid, tmd->gid, tmd->perm,
		"%s/%d", TM1637_CUSE_DEFAULT_DEVNAME, tmd->cuse_id);

	/*
	 * Resolve device naming conflict with new
	 * kernel evdev module:
	 */
	if (tmd->pdev == NULL &&
	    cuse_alloc_unit_number_by_id(&tmd->cuse_id, CUSE_ID_DEFAULT(cuse_id)) == 0)
		goto again;

	syslog(LOG_INFO, "Created /dev/%s/%d\n", TM1637_CUSE_DEFAULT_DEVNAME, tmd->cuse_id);
    }

    return (tmd);
}

int
main(int argc, char **argv)
{
    int ch, long_index = 0;
    size_t scl, sda, brightness = BRIGHT_TYPICAL;
    extern char *optarg, *suboptarg;
    char *options, *value, *end;
    struct tm1637_dev_t *tmd;

    char *subopts[] = {
#define SCL		0
			"scl",
#define SDA		1
			"sda",
#define BRIGHTNESS	2
			"brightness",
	NULL
    };

    static struct option long_options[] = {
	{"device", required_argument, 0, 'd' },
	{"help",   required_argument, 0, 'h' },
	{0, 0, 0, 0}
    };

    openlog("tm1637d", LOG_NDELAY | LOG_PERROR | LOG_PID, LOG_DAEMON);

    while ((ch = getopt_long(argc, argv, "bd:h", long_options, &long_index)) != -1) {
	switch(ch) {
	case 'b':
	    background = true;
	    break;
	case 'd':
	    options = optarg;
	    while (*options) {
		switch(getsubopt(&options, subopts, &value)) {
		case SCL:
		    if (!value)
			tm1637_errx(EXIT_FAILURE, "no value for scl");
		    scl = strtoul(value, &end, 0);
		    if (scl >= NUMBER_OF_GPIO_PINS)
			tm1637_errx(EXIT_FAILURE, "too big value for scl");
		    break;
		case SDA:
		    if (!value)
			tm1637_errx(EXIT_FAILURE, "no value for sda");
		    sda = strtoul(value, &end, 0);
		    if (sda >= NUMBER_OF_GPIO_PINS)
			tm1637_errx(EXIT_FAILURE, "too big value for sda");
		    break;
		case BRIGHTNESS:
		    if (value) {
			brightness = strtoul(value, &end, 0);
			if (brightness > BRIGHTEST)
			    tm1637_errx(EXIT_FAILURE, "too big value for brightness");
		    }
		case -1:
		    if (suboptarg)
			tm1637_errx(1, "illegal sub option %s", suboptarg);
		    else
			tm1637_errx(1, "missing sub option");
		    break;
		}
	    }
	    if (scl == sda)
		tm1637_errx(EXIT_FAILURE, "Pins 'scl' and 'sda' cannot be same");
	    /* Create tm1637 specimens */
	    tmd = tm1637_register(scl, sda, brightness);
	    break;
	case 'h':
	    /* FALLTHROUGH */
	default:
	    usage();
	    exit(EXIT_SUCCESS);
	}
    }

    if (cuse_init() != 0) {
	tm1637_errx(EX_USAGE, "Could not open '/dev/cuse'. Did you kldload cuse4bsd?");
    }

    gpio_handle = gpio_open_device(gpio_device);
    if (gpio_handle == GPIO_INVALID_HANDLE)
	tm1637_errx(1, "Failed to open '%s'", gpio_device);

    /* Create devices and initialize them */
    SLIST_FOREACH(tmd, &tm1637_head, next) {
	tm1637_create(tmd);

	if (pthread_mutex_init(&tmd->lock, NULL))
	    tm1637_errx(1, "pthread_mutex_init failed: %m");

	/* Prepare sda- and sclpins to send data */
	gpio_pin_output(gpio_handle, tmd->sdapin);
	gpio_pin_output(gpio_handle, tmd->sclpin);

	/* Clear a display and turn it on */
	tm1637_display_blank(tmd);
	tm1637_display_on(tmd);
    }

    /* Unbinds from terminal if '-b' */
    if (background)
	daemonize();

    /* Intercept signals to our function */
    if (signal (SIGINT, tm1637_termination) == SIG_IGN)
	signal (SIGINT, SIG_IGN);
    if (signal (SIGTERM, tm1637_termination) == SIG_IGN)
	signal (SIGTERM, SIG_IGN);

    for(;;) {
	tm1637_work(NULL);
    }
}
