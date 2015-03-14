AC_INIT([Linux FireWire utilities], [0.3], [linux1394-devel@lists.sourceforge.net],
	[linux-firewire-utils], [https://github.com/cladisch/linux-firewire-utils])
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

AC_CHECK_DECL([FW_CDEV_EVENT_REQUEST2], [], [],
	      [#include <linux/firewire-cdev.h>])
AC_MSG_CHECKING([for cdev ABI version 4])
juju4=$ac_cv_have_decl_FW_CDEV_EVENT_REQUEST2
AC_MSG_RESULT([$juju4])
AS_IF([test "$juju4" = yes],
      [AC_DEFINE([HAVE_CDEV_4], [1])])
AM_CONDITIONAL([HAVE_CDEV_4],
	       [test "$juju4" = yes])

AC_OUTPUT([
Makefile
src/lsfirewire
src/lsfirewire.8
src/lsfirewirephy.8
src/firewire-request.8
src/firewire-phy-command.8
])

AS_IF([test "$juju4" != yes],
      [AC_MSG_WARN([to enable all utilities, install Linux 2.6.36 headers])])
