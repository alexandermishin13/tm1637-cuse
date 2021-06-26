# $FreeBSD$

PREFIX=		/usr/local
MK_DEBUG_FILES=	no

PROG=		tm1637d
BINDIR=		${PREFIX}/sbin

MAN=		${PROG}.8
MANDIR=		${PREFIX}/man/man

LDADD=		-L/usr/local/lib -lutil -lgpio -lcuse

.include <bsd.prog.mk>
