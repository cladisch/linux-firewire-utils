AC_INIT([Juju utils], [0.0], [linux1394-devel@lists.sourceforge.net],
	[jujuutils], [http://code.google.com/p/jujuutils/])
AC_CONFIG_SRCDIR([src/lsfirewire.in])
AM_INIT_AUTOMAKE([foreign no-define silent-rules])

# enable silent rules by default
AS_IF([test "${enable_silent_rules+set}" != set],
      [AM_DEFAULT_VERBOSITY=0])

AC_OUTPUT([
README
Makefile
src/Makefile
src/lsfirewire
src/lsfirewire.8
])
