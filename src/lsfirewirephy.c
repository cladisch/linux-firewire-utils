/*
 * lsfirewirephy.c - list the PHYs on a bus
 *
 * Copyright 2010-2011 Clemens Ladisch <clemens@ladisch.de>
 *
 * licensed under the terms of version 2 of the GNU General Public License
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <getopt.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/firewire-cdev.h>
#include <linux/firewire-constants.h>

#define PHY_REMOTE_ACCESS_PAGED(phy_id, page, port, reg) \
	(((phy_id) << 24) | (5 << 18) | ((page) << 15) | ((port) << 11) | ((reg) << 8))
#define PHY_REMOTE_REPLY_PAGED(phy_id, page, port, reg, data) \
	(((phy_id) << 24) | (7 << 18) | ((page) << 15) | ((port) << 11) | ((reg) << 8) | (data))

typedef __u8 u8;
typedef __u32 u32;
typedef __u64 u64;

typedef u32 u24;

static const struct vendor {
	u24 oui;
	const char *name;
	const struct phy {
		u24 id;
		const char *name;
	} *phys;
} vendors[] = {
	{
		.oui  = 0x00000e,
		.name = "Fujitsu",
		.phys = (const struct phy[]) {
			{ 0x086613, "MB86613" },
			{}
		}
	},
	{
		.oui  = 0x00004c,
		.name = "NEC",
		.phys = (const struct phy[]) {
			{}
		}
	},
	{
		.oui  = 0x00053d,
		.name = "Agere Systems",
		.phys = (const struct phy[]) {
			{ 0x06430a, "FW643E" },
			{}
		}
	},
	{
		.oui  = 0x001018,
		.name = "Broadcom",
		.phys = (const struct phy[]) {
			{}
		}
	},
	{
		/*
		 * This OUI actually belongs to System S.p.A.;
		 * VIA's OUI is 0x004063.
		 */
		.oui  = 0x001163,
		.name = "VIA Technologies",
		.phys = (const struct phy[]) {
			{ 0x306001, "VT630x" },
			{}
		}
	},
	{
		.oui  = 0x001454,
		.name = "Symwave",
		.phys = (const struct phy[]) {
			{}
		}
	},
	{
		.oui  = 0x001b8c,
		.name = "JMicron",
		.phys = (const struct phy[]) {
			{ 0x038100, "JMB38x" },
			{}
		}
	},
	{
		.oui  = 0x00601d,
		.name = "Lucent Technologies",
		.phys = (const struct phy[]) {
			{ 0x032361, "FW323" },
			{ 0x080201, "FW802" },
			{}
		}
	},
	{
		.oui  = 0x00c02d,
		.name = "Fujifilm",
		.phys = (const struct phy[]) {
			{ 0x303562, "MD8405B" },
			{ 0x303565, "MD8405E" },
			{}
		}
	},
	{
		.oui  = 0x080028,
		.name = "Texas Instruments",
		.phys = (const struct phy[]) {
			{ 0x42308a, "TSB41LV02A" },
			{ 0x424296, "TSB41AB1/2" },
			{ 0x424499, "TSB43AB22(A)" },
			{ 0x424729, "XIO2200A" },
			{ 0x434195, "TSB41AB3" },
			{ 0x434615, "TSB43CB43A" },
			{ 0x46318a, "TSB41LV06A" },
			{ 0x831304, "TSB81BA3(A)" },
			{ 0x831306, "TSB81BA3D" },
			{ 0x831307, "TSB81BA3E/XIO2213" },
			{ 0x833005, "TSB41BA3D" },
			{}
		},
	},
	{}
};

static char *device_file_name;
static int list_phy_id = -1;
static int fd;
static bool any_unknown_phys;
struct fw_cdev_get_info get_info;
struct fw_cdev_event_bus_reset bus_reset;

static void help(void)
{
	fputs("Usage: lsfirewirephy [options] [devicenode [phyid]]\n"
	      "Options:\n"
	      " -h, --help      show this message and exit\n"
	      " -V, --version   show version number and exit\n"
	      "\n"
	      "Report bugs to <" PACKAGE_BUGREPORT ">.\n"
	      PACKAGE_NAME " home page: <" PACKAGE_URL ">.\n",
	      stderr);
}

static void parse_parameters(int argc, char *argv[])
{
	static const char short_options[] = "hV";
	static const struct option long_options[] = {
		{ "help", 0, NULL, 'h' },
		{ "version", 0, NULL, 'V' },
		{}
	};
	int c;
	char *endptr;

	while ((c = getopt_long(argc, argv, short_options, long_options, NULL)) != -1) {
		switch (c) {
		case 'h':
			help();
			exit(EXIT_SUCCESS);
		case 'V':
			puts("lsfirewirephy version " PACKAGE_VERSION);
			exit(EXIT_SUCCESS);
		default:
		syntax_error:
			help();
			exit(EXIT_FAILURE);
		}
	}

	if (optind < argc) {
		device_file_name = strdup(argv[optind++]);
		if (!device_file_name) {
			fputs("out of memory\n", stderr);
			exit(EXIT_FAILURE);
		}

		if (optind < argc) {
			list_phy_id = strtol(argv[optind], &endptr, 0);
			if (argv[optind][0] != '\0' && *endptr != '\0')
				goto syntax_error;
			if (list_phy_id < 0 || list_phy_id >= 63) {
				fputs("phy-id must be between 0 and 62\n", stderr);
				exit(EXIT_FAILURE);
			}
			++optind;

			if (optind < argc)
				goto syntax_error;
		}
	}
}

static struct dirent **fw_dirents;
static int fw_dirent_count;
static int fw_dirent_index;

static int fw_filter(const struct dirent *dirent)
{
	return dirent->d_name[0] == 'f' &&
	       dirent->d_name[1] == 'w' &&
	       isdigit(dirent->d_name[2]);
}

static void init_enumerated_fw_devs(void)
{
	fw_dirent_count = scandir("/dev", &fw_dirents, fw_filter, versionsort);
	if (fw_dirent_count < 0) {
		perror("cannot read /dev");
		exit(EXIT_FAILURE);
	}
	fw_dirent_index = 0;
}

static void cleanup_enumerated_fw_devs(void)
{
	int i;

	for (i = 0; i < fw_dirent_count; ++i)
		free(fw_dirents[i]);
	free(fw_dirents);
	fw_dirents = NULL;
	fw_dirent_count = 0;
}

static bool has_enumerated_fw_dev(void)
{
	free(device_file_name);
	device_file_name = NULL;

	if (fw_dirent_index < fw_dirent_count) {
		if (asprintf(&device_file_name, "/dev/%s",
			     fw_dirents[fw_dirent_index]->d_name) < 0) {
			perror("asprintf failed");
			exit(EXIT_FAILURE);
		}
		return true;
	} else {
		cleanup_enumerated_fw_devs();
		return false;
	}
}

static void next_enumerated_fw_dev(void)
{
	++fw_dirent_index;
}

static bool open_device(bool force)
{
	fd = open(device_file_name, O_RDWR);
	if (fd == -1) {
		if (!force && errno == ENODEV)
			return false;
		perror(device_file_name);
		exit(EXIT_FAILURE);
	}

	get_info.version = 4;
	get_info.rom_length = 0;
	get_info.rom = 0;
	get_info.bus_reset = (u64)&bus_reset;
	get_info.bus_reset_closure = 0;
	if (ioctl(fd, FW_CDEV_IOC_GET_INFO, &get_info) < 0) {
		perror("GET_INFO ioctl failed");
		exit(EXIT_FAILURE);
	}
	if (get_info.version < 4) {
		fputs("this kernel is too old\n", stderr);
		exit(EXIT_FAILURE);
	}

	return true;
}

static void enable_phy_packets(void)
{
	struct fw_cdev_receive_phy_packets receive_phy_packets;

	receive_phy_packets.closure = 0;
	if (ioctl(fd, FW_CDEV_IOC_RECEIVE_PHY_PACKETS, &receive_phy_packets) < 0) {
		perror("RECEIVE_PHY_PACKETS ioctl failed");
		exit(EXIT_FAILURE);
	}
}

static bool device_is_local_node(void)
{
	return bus_reset.node_id == bus_reset.local_node_id;
}

static void check_local_node(void)
{
	if (!device_is_local_node()) {
		fprintf(stderr, "%s: not a local node\n", device_file_name);
		exit(EXIT_FAILURE);
	}
}

static const struct vendor *search_vendor(u24 oui)
{
	const struct vendor *vendor;

	for (vendor = vendors; vendor->name; ++vendor)
		if (vendor->oui== oui)
			return vendor;
	return NULL;
}

static const struct phy *search_phy(const struct vendor *vendor, u24 id)
{
	const struct phy *phy;

	for (phy = vendor->phys; phy->name; ++phy)
		if (phy->id == id)
			return phy;
	return NULL;
}

static void list_phy(void)
{
	struct fw_cdev_send_phy_packet send_phy_packet;
	unsigned int reg, regs_read;
	int ready, r;
	struct pollfd pfd;
	u8 buf[256];
	struct fw_cdev_event_common *event;
	u8 reg_values[6];
	u24 oui, id;
	const struct vendor *vendor;
	const struct phy *phy;

	send_phy_packet.closure = 0;
	send_phy_packet.generation = bus_reset.generation;
	for (reg = 2; reg <= 7; ++reg) {
		send_phy_packet.data[0] = PHY_REMOTE_ACCESS_PAGED(list_phy_id, 1, 0, reg);
		send_phy_packet.data[1] = ~send_phy_packet.data[0];
		if (ioctl(fd, FW_CDEV_IOC_SEND_PHY_PACKET, &send_phy_packet) < 0) {
			perror("SEND_PHY_PACKET ioctl failed");
			exit(EXIT_FAILURE);
		}
	}

	pfd.fd = fd;
	pfd.events = POLLIN;
	regs_read = 0;
	while (regs_read != 0xfc) {
		ready = poll(&pfd, 1, 123);
		if (ready < 0) {
			perror("poll failed");
			exit(EXIT_FAILURE);
		}
		if (!ready) {
			fputs("timeout\n", stderr);
			return; /* try next PHY */
		}
		r = read(fd, buf, sizeof buf);
		if (r < sizeof(struct fw_cdev_event_common)) {
			fputs("short read\n", stderr);
			exit(EXIT_FAILURE);
		}
		event = (void *)buf;
		if (event->type == FW_CDEV_EVENT_BUS_RESET) {
			/* TODO: retry */
			fputs("bus reset\n", stderr);
			exit(EXIT_FAILURE);
		}
		if (event->type == FW_CDEV_EVENT_PHY_PACKET_SENT) {
			struct fw_cdev_event_phy_packet *phy_packet = (void *)buf;
			if (phy_packet->rcode != RCODE_COMPLETE) {
				fprintf(stderr, "PHY packet failed: rcode %u\n",
					(unsigned int)phy_packet->rcode);
				exit(EXIT_FAILURE);
			}
		} else if (event->type == FW_CDEV_EVENT_PHY_PACKET_RECEIVED) {
			struct fw_cdev_event_phy_packet *phy_packet = (void *)buf;
			if (phy_packet->length == 8 &&
			    (phy_packet->data[0] & 0xffff8000)
			    == PHY_REMOTE_REPLY_PAGED(list_phy_id, 1, 0, 0, 0)) {
				reg = (phy_packet->data[0] >> 8) & 7;
				if (reg >= 2) {
					reg_values[reg - 2] = phy_packet->data[0] & 0xff;
					regs_read |= 1 << reg;
				}
			}
		}
	}

	oui = (reg_values[0] << 16) | (reg_values[1] << 8) | reg_values[2];
	id  = (reg_values[3] << 16) | (reg_values[4] << 8) | reg_values[5];
	vendor = search_vendor(oui);
	phy = vendor ? search_phy(vendor, id) : NULL;

	printf("bus %u, node %d: %06x:%06x  ", get_info.card, list_phy_id, oui, id);
	if (vendor)
		printf("%s %s\n", vendor->name, phy ? phy->name : "(unknown)");
	else
		printf("%s\n", "(unknown)");

	if (!phy)
		any_unknown_phys = true;
}

static void list_one_phy(void)
{
	open_device(true);
	check_local_node();
	enable_phy_packets();
	list_phy();
	close(fd);
}

static void list_device(void)
{
	unsigned int list_card;

	open_device(true);
	list_phy_id = bus_reset.node_id & 0x3f;
	list_card = get_info.card;
	if (!device_is_local_node()) {
		close(fd);
		for (init_enumerated_fw_devs();
		     has_enumerated_fw_dev();
		     next_enumerated_fw_dev()) {
			if (!open_device(false))
				continue;
			if (get_info.card == list_card &&
			    device_is_local_node())
				goto found;
			close(fd);
		}
		fprintf(stderr, "local node for card %u not found\n", list_card);
		exit(EXIT_FAILURE);
	}
found:
	enable_phy_packets();
	list_phy();
	close(fd);
}

static void list_all_buses(void)
{
	for (init_enumerated_fw_devs();
	     has_enumerated_fw_dev();
	     next_enumerated_fw_dev()) {
		if (!open_device(false))
			continue;
		if (device_is_local_node()) {
			int root_phy_id = bus_reset.root_node_id & 0x3f;
			enable_phy_packets();
			for (list_phy_id = 0; list_phy_id <= root_phy_id; ++list_phy_id)
				list_phy();
		}
		close(fd);
	}
}

int main(int argc, char *argv[])
{
	parse_parameters(argc, argv);
	if (device_file_name)
		if (list_phy_id >= 0)
			list_one_phy();
		else
			list_device();
	else
		list_all_buses();
	if (any_unknown_phys)
		fputs("  Please check this web page for updated PHY IDs:\n"
		      "  http://code.google.com/p/jujuutils/wiki/PhyIds\n", stderr);
	return 0;
}
