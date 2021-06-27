# $FreeBSD$

PREFIX=		/usr/local
MK_DEBUG_FILES=	no

PROG=		tm1637d
BINDIR=		${PREFIX}/sbin

SCRIPTS=		${PROG}.sh
SCRIPTSNAME_${PROG}.sh=	${PROG}
SCRIPTSDIR_${PROG}.sh=	${PREFIX}/etc/rc.d

MAN=		${PROG}.8
MANDIR=		${PREFIX}/man/man

LDADD=		-L/usr/local/lib -lutil -lgpio -lcuse

uninstall:
	rm ${BINDIR}/${PROG}
	rm ${MANDIR}8/${MAN}.gz
	rm ${PREFIX}/etc/rc.d/${PROG}

check:
	cppcheck \
	    --enable=all \
	    --force \
	    -I/usr/local/include \
	    ./

.include <bsd.prog.mk>
