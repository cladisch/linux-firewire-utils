/*
 * lsfirewirephy.c - list the PHYs on a bus
 *
 * Copyright 2010 Clemens Ladisch <clemens@ladisch.de>
 *
 * licensed under the terms of version 2 of the GNU General Public License
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include <dirent.h>
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

static bool list_all;
static const char *device_file_name = "/dev/fw0";
static int list_phy_id = -1;
static int fd;
static int root_phy_id;
static u32 generation;
static u32 card_index;

static void help(void)
{
	fputs("Usage: lsfirewirephy [options]\n"
	      "Options:\n"
	      " -a, --all          list all PHYs on all buses\n"
	      " -d, --device=file  device file of the local node on the bus\n"
	      " -p, --phy-id=nr    list only this PHY\n"
	      " -h, --help         show this message and exit\n"
	      " -V, --version      show version number and exit\n"
	      "\n"
	      "Report bugs to <" PACKAGE_BUGREPORT ">.\n"
	      PACKAGE_NAME " home page: <" PACKAGE_URL ">.\n",
	      stderr);
}

static void parse_parameters(int argc, char *argv[])
{
	static const char short_options[] = "ad:p:hV";
	static const struct option long_options[] = {
		{ "all", 0, NULL, 'a' },
		{ "device", 1, NULL, 'd' },
		{ "phy-id", 1, NULL, 'p' },
		{ "help", 0, NULL, 'h' },
		{ "version", 0, NULL, 'V' },
		{}
	};
	int c;
	char *endptr;

	while ((c = getopt_long(argc, argv, short_options, long_options, NULL)) != -1) {
		switch (c) {
		case 'a':
			list_all = true;
			break;
		case 'd':
			device_file_name = optarg;
			break;
		case 'p':
			list_phy_id = strtol(optarg, &endptr, 0);
			if (optarg[0] != '\0' && *endptr != '\0')
				goto syntax_error;
			if (list_phy_id < 0 || list_phy_id >= 63) {
				fputs("phy-id must be between 0 and 62\n", stderr);
				exit(EXIT_FAILURE);
			}
			break;
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
	if (optind < argc)
		goto syntax_error;
}

static bool open_device(const char *local_node_file)
{
	struct fw_cdev_get_info get_info;
	struct fw_cdev_event_bus_reset bus_reset;
	struct fw_cdev_receive_phy_packets receive_phy_packets;

	fd = open(local_node_file, O_RDWR);
	if (fd == -1) {
		if (!list_all) {
			perror(local_node_file);
			exit(EXIT_FAILURE);
		} else {
			return false;
		}
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
	if (bus_reset.node_id != bus_reset.local_node_id) {
		if (!list_all) {
			fprintf(stderr, "%s: not a local node\n", local_node_file);
			exit(EXIT_FAILURE);
		} else {
			close(fd);
			return false;
		}
	}
	root_phy_id = bus_reset.root_node_id & 0x3f;
	generation = bus_reset.generation;
	card_index = get_info.card;

	receive_phy_packets.closure = 0;
	if (ioctl(fd, FW_CDEV_IOC_RECEIVE_PHY_PACKETS, &receive_phy_packets) < 0) {
		perror("RECEIVE_PHY_PACKETS ioctl failed");
		exit(EXIT_FAILURE);
	}

	return true;
}

static void list_phy(int phy_id)
{
	static u8 buf[256];
	struct fw_cdev_send_phy_packet send_phy_packet;
	int reg, regs_read;
	struct pollfd pfd;
	int ready, r;
	struct fw_cdev_event_common *event;
	u8 reg_values[6];

	send_phy_packet.closure = 0;
	send_phy_packet.generation = generation;
	for (reg = 2; reg <= 7; ++reg) {
		send_phy_packet.data[0] = PHY_REMOTE_ACCESS_PAGED(phy_id, 1, 0, reg);
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
			    == PHY_REMOTE_REPLY_PAGED(phy_id, 1, 0, 0, 0)) {
				reg = (phy_packet->data[0] >> 8) & 7;
				if (reg >= 2) {
					reg_values[reg - 2] = phy_packet->data[0] & 0xff;
					regs_read |= 1 << reg;
				}
			}
		}
	}

	printf("%u.%d: %02x%02x%02x:%02x%02x%02x\n",
	       (unsigned int)card_index, phy_id,
	       reg_values[0], reg_values[1], reg_values[2],
	       reg_values[3], reg_values[4], reg_values[5]);
}

static void list_phys(void)
{
	int p;

	for (p = 0; p <= root_phy_id; ++p)
		list_phy(p);
}

static void list_bus(const char *local_node_file)
{
	open_device(device_file_name);
	if (list_phy_id >= 0)
		list_phy(list_phy_id);
	else
		list_phys();
	close(fd);
}

static int fw_filter(const struct dirent *dirent)
{
	return dirent->d_name[0] == 'f' &&
	       dirent->d_name[1] == 'w' &&
	       isdigit(dirent->d_name[2]);
}

static void list_all_buses(void)
{
	int i, ents;
	struct dirent **devices;
	char *device_file;

	ents = scandir("/dev", &devices, fw_filter, versionsort);
	if (ents < 0) {
		perror("cannot read /dev");
		exit(EXIT_FAILURE);
	}
	for (i = 0; i < ents; ++i) {
		if (asprintf(&device_file, "/dev/%s", devices[i]->d_name) < 0) {
			perror("asprintf failed");
			exit(EXIT_FAILURE);
		}
		if (open_device(device_file)) {
			list_phys();
			close(fd);
		}
		free(devices[i]);
	}
	free(devices);
}

int main(int argc, char *argv[])
{
	parse_parameters(argc, argv);
	if (!list_all)
		list_bus(device_file_name);
	else
		list_all_buses();
	return 0;
}
