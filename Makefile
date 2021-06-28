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

MAN=			tm1637d.8
MANDIR=			${PREFIX}/man/man

LDADD_tm1637d=		-L/usr/local/lib -lutil -lgpio -lcuse
LDADD_tm1637ctl=	-L/usr/local/lib

uninstall:
	rm ${BINDIR}/tm1637d
	rm ${MANDIR}8/${MAN}.gz
	rm ${PREFIX}/etc/rc.d/tm1637d

check:
	cppcheck \
	    --enable=all \
	    --force \
	    -I/usr/local/include \
	    ./

.include <bsd.progs.mk>
