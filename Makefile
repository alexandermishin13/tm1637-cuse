# $FreeBSD$

PREFIX=		/usr/local
MK_DEBUG_FILES=	no

PROGS=			tm1637d tm1637ctl
BINDIR_tm1637d=		${PREFIX}/sbin
BINDIR_tm1637ctl=	${PREFIX}/bin

SCRIPTS=		tm1637d.sh
SCRIPTSNAME_tm1637d.sh=	tm1637d
SCRIPTSDIR_tm1637d.sh=	${PREFIX}/etc/rc.d

INCS=			tm1637d.h
INCSDIR=		${PREFIX}/include

MAN_tm1637d=		tm1637d.8
MAN_tm1637ctl=		tm1637ctl.1
MANDIR=			${PREFIX}/man/man

LDADD_tm1637d=		-L/usr/local/lib -lutil -lgpio -lcuse -lpthread
LDADD_tm1637ctl=	-L/usr/local/lib

uninstall:
	rm ${BINDIR_tm1637d}/tm1637d
	rm ${BINDIR_tm1637ctl}/tm1637ctl
	rm ${MANDIR}1/${MAN_tm1637ctl}.gz
	rm ${MANDIR}8/${MAN_tm1637d}.gz
	rm ${PREFIX}/etc/rc.d/tm1637d

check:
	cppcheck \
	    --enable=all \
	    --force \
	    -I/usr/local/include \
	    ./

.include <bsd.progs.mk>
