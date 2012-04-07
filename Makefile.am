AUTOMAKE_OPTIONS := subdir-objects

bin_SCRIPTS = src/lsfirewire
bin_PROGRAMS = src/firewire-request

pkglibexec_SCRIPTS = src/crpp

man_MANS = src/lsfirewire.8 src/firewire-request.8

if HAVE_CDEV_4
bin_PROGRAMS += src/lsfirewirephy src/firewire-phy-command
man_MANS += src/lsfirewirephy.8 src/firewire-phy-command.8
endif

EXTRA_DIST = README src/crpp
