AC_INIT([Juju utils], [0.0], [linux1394-devel@lists.sourceforge.net],
	[jujuutils], [http://code.google.com/p/jujuutils/])
AC_CONFIG_SRCDIR([src/lsfirewire.in])
AM_INIT_AUTOMAKE([foreign no-define silent-rules])

# enable silent rules by default
AS_IF([test "${enable_silent_rules+set}" != set],
      [AM_DEFAULT_VERBOSITY=0])

AC_PROG_CC

AC_CHECK_HEADERS_ONCE([
asm/byteorder.h
linux/firewire-cdev.h
linux/firewire-constants.h
])

AS_IF([test "$ac_cv_header_asm_byteorder_h" != yes],
      [AC_MSG_ERROR([Linux kernel headers not found])])

JUJU=1
test "$ac_cv_header_linux_firewire_cdev_h" = yes || JUJU=
test "$ac_cv_header_linux_firewire_constants_h" = yes || JUJU=
AS_IF([test -z "$JUJU"],
      [AC_MSG_ERROR([required Linux FireWire headers not found])])

AC_OUTPUT([
README
Makefile
src/Makefile
src/lsfirewire
src/lsfirewire.8
src/firewire-request.8
])
