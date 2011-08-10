/*
 * firewire-phy-command.c - send PHY packets
 *
 * Copyright 2011 Clemens Ladisch <clemens@ladisch.de>
 *
 * licensed under the terms of the GNU General Public License, version 2
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <getopt.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/firewire-cdev.h>
#include <linux/firewire-constants.h>

#ifndef FW_CDEV_IOC_SEND_PHY_PACKET
#error kernel headers too old
#endif

#define ARRAY_SIZE(a) (sizeof(a) / sizeof *(a))

#define ptr_to_u64(p) ((uintptr_t)(p))

typedef __u8 u8;
typedef __u32 u32;
typedef __u64 u64;

struct node {
	struct node *next;
	char *name;
	int fd;
	u32 card;
	u32 id;
	u32 generation;
	bool is_local;
};

static struct node *nodes;
static struct node *local_node;
static u32 param_node_id;
static u32 ping_time;
static u32 self_ids[3];

static void help(void)
{
	fputs("Usage: firewire-phy-command [options] command [parameters]\n"
	      "Commands:\n"
	      "  config [root <node>] [gapcount <value>]\n"
	      "  ping <node>\n"
	      "  read <node> [<page> <port>] <register>\n"
	      "  nop|disable|suspend|clear|enable|resume <node> <port>\n"
	      "  resume\n"
	      "  linkon <node>\n"
	      "  reset\n"
	      "Options:\n"
	      " -b, --bus=node  bus to send packet on\n"
	      " -h, --help      show this message and exit\n"
	      " -V, --version   show version number and exit\n"
	      "\n"
	      "Report bugs to <" PACKAGE_BUGREPORT ">.\n"
	      PACKAGE_NAME " home page: <" PACKAGE_URL ">.\n",
	      stderr);
}

static int fw_filter(const struct dirent *dirent)
{
	unsigned int i;

	if (dirent->d_name[0] != 'f' ||
	    dirent->d_name[1] != 'w')
		return false;
	i = 2;
	do {
		if (!isdigit(dirent->d_name[i]))
			return false;
	} while (dirent->d_name[++i]);
	return true;
}

static void open_all_nodes(void)
{
	struct dirent **ents;
	int count, i;
	struct node *node;
	struct fw_cdev_get_info get_info;
	struct fw_cdev_event_bus_reset bus_reset;
	bool eacces = false;

	count = scandir("/dev", &ents, fw_filter, versionsort);
	if (count < 0) {
		perror("cannot read /dev");
		exit(EXIT_FAILURE);
	}

	for (i = count - 1; i >= 0; --i) {
		node = malloc(sizeof(*node));
		if (!node) {
			fputs("out of memory\n", stderr);
			exit(EXIT_FAILURE);
		}

		if (asprintf(&node->name, "/dev/%s", ents[i]->d_name) < 0) {
			perror("asprintf failed");
			exit(EXIT_FAILURE);
		}
		node->fd = open(node->name, O_RDWR);
		if (node->fd == -1) {
			if (errno == EACCES)
				eacces = true;
			free(node->name);
			free(node);
			continue;
		}

		get_info.version = 4;
		get_info.rom_length = 0;
		get_info.rom = 0;
		get_info.bus_reset = ptr_to_u64(&bus_reset);
		get_info.bus_reset_closure = 0;
		if (ioctl(node->fd, FW_CDEV_IOC_GET_INFO, &get_info) < 0) {
			close(node->fd);
			free(node->name);
			free(node);
			continue;
		}
		node->card = get_info.card;
		node->id = bus_reset.node_id;
		node->generation = bus_reset.generation;
		node->is_local = bus_reset.node_id == bus_reset.local_node_id;

		node->next = nodes;
		nodes = node;
		free(ents[i]);
	}
	free(ents);

	if (!nodes) {
		if (eacces) {
			errno = EACCES;
			perror("/dev/fw*");
		} else {
			fputs("no fw devices found\n", stderr);
		}
		exit(EXIT_FAILURE);
	}
}

static void close_all_nodes(void)
{
	struct node *node, *next;

	for (node = nodes; node; node = next) {
		close(node->fd);
		free(node->name);
		next = node->next;
		free(node);
	}
}

static void find_local_node(const char *bus_name)
{
	int bus_card;
	char *endptr;
	int fd;
	struct fw_cdev_get_info get_info;
	struct node *node;

	if (bus_name) {
		bus_card = strtol(bus_name, &endptr, 0);
		if (!*endptr) {
			if (bus_card < 0) {
				fputs("invalid bus number\n", stderr);
				exit(EXIT_FAILURE);
			}
		} else {
			bus_card = -1;
			for (node = nodes; node; node = node->next)
				if (!strcmp(node->name, bus_name)) {
					bus_card = node->card;
					break;
				}
			if (bus_card < 0) {
				fd = open(bus_name, O_RDWR);
				if (fd == -1) {
					perror(bus_name);
					exit(EXIT_FAILURE);
				}
				get_info.version = 4;
				get_info.rom_length = 0;
				get_info.rom = 0;
				get_info.bus_reset = 0;
				get_info.bus_reset_closure = 0;
				if (ioctl(fd, FW_CDEV_IOC_GET_INFO, &get_info) < 0) {
					fprintf(stderr, "%s: not a fw device\n", bus_name);
					exit(EXIT_FAILURE);
				}
				close(fd);
				bus_card = get_info.card;
			}
		}
	} else {
		bus_card = -1;
	}

	for (node = nodes; node; node = node->next) {
		if (node->is_local &&
		    (bus_card == -1 || node->card == bus_card)) {
			local_node = node;
			return;
		}
	}

	if (bus_card == -1)
		fputs("local node not found\n", stderr);
	else
		fprintf(stderr, "local node for card %d not found\n", bus_card);
	exit(EXIT_FAILURE);
}

static void find_param_node(const char *name)
{
	int id;
	char *endptr;
	struct node *node;
	int fd;
	struct fw_cdev_get_info get_info;
	struct fw_cdev_event_bus_reset bus_reset;
	char card_str[16];

	id = strtol(name, &endptr, 0);
	if (!*endptr) {
		if (id < 0 || id > 63) {
			fputs("invalid node id\n", stderr);
			exit(EXIT_FAILURE);
		}
		param_node_id = id;
		return;
	}

	for (node = nodes; node; node = node->next)
		if (!strcmp(node->name, name)) {
			param_node_id = node->id & 0x3f;
			sprintf(card_str, "%u", node->card);
			find_local_node(card_str);
			return;
		}

	fd = open(name, O_RDWR);
	if (fd == -1) {
		perror(name);
		exit(EXIT_FAILURE);
	}
	get_info.version = 4;
	get_info.rom_length = 0;
	get_info.rom = 0;
	get_info.bus_reset = ptr_to_u64(&bus_reset);
	get_info.bus_reset_closure = 0;
	if (ioctl(fd, FW_CDEV_IOC_GET_INFO, &get_info) < 0) {
		fprintf(stderr, "%s: not a fw device\n", name);
		exit(EXIT_FAILURE);
	}
	close(fd);
	param_node_id = bus_reset.node_id & 0x3f;
	sprintf(card_str, "%u", get_info.card);
	find_local_node(card_str);
}

static u32 _send_packet(u32 quadlet0, u32 quadlet1, u32 response_mask, u32 response_bits)
{
	struct fw_cdev_receive_phy_packets receive_phy_packets;
	struct fw_cdev_send_phy_packet send_phy_packet;
	struct pollfd pollfd;
	int poll_result;
	ssize_t bytes;
	union fw_cdev_event event;
	bool wait_for_sent = true;
	bool wait_for_response = response_mask != 0;
	unsigned int self_id_index = 0;
	u32 response = 0;

	if (wait_for_response) {
		receive_phy_packets.closure = 0;
		if (ioctl(local_node->fd, FW_CDEV_IOC_RECEIVE_PHY_PACKETS, &receive_phy_packets) < 0) {
			perror("RECEIVE_PHY_PACKETS ioctl failed");
			exit(EXIT_FAILURE);
		}
	}

	send_phy_packet.closure = 0;
	send_phy_packet.data[0] = quadlet0;
	send_phy_packet.data[1] = quadlet1;
	send_phy_packet.generation = local_node->generation;
	if (ioctl(local_node->fd, FW_CDEV_IOC_SEND_PHY_PACKET, &send_phy_packet) < 0) {
		perror("SEND_PHY_PACKET ioctl failed");
		exit(EXIT_FAILURE);
	}

	pollfd.fd = local_node->fd;
	pollfd.events = POLLIN;
	while (wait_for_sent || wait_for_response) {
		poll_result = poll(&pollfd, 1, 100);
		if (poll_result < 0) {
			perror("poll failed");
			exit(EXIT_FAILURE);
		}
		if (poll_result == 0) {
			fputs("timeout\n", stderr);
			exit(EXIT_FAILURE);
		}
		bytes = read(local_node->fd, &event, sizeof(event));
		if (bytes < sizeof(struct fw_cdev_event_common)) {
			fputs("short read\n", stderr);
			exit(EXIT_FAILURE);
		}
		switch (event.common.type) {
		case FW_CDEV_EVENT_BUS_RESET:
			fputs("bus reset\n", stderr);
			exit(EXIT_FAILURE);
		case FW_CDEV_EVENT_PHY_PACKET_SENT:
			wait_for_sent = false;
			if (event.phy_packet.length >= 4)
				ping_time = event.phy_packet.data[0];
			break;
		case FW_CDEV_EVENT_PHY_PACKET_RECEIVED:
			if ((event.phy_packet.data[0] & response_mask) == response_bits) {
				response = event.phy_packet.data[0];
				if ((response & 0xc0000000) == 0x80000000) {
					self_ids[self_id_index++] = response;
					if (self_id_index >= ARRAY_SIZE(self_ids) ||
					    !(response & 1))
						wait_for_response = false;
				} else {
					wait_for_response = false;
				}
			}
			break;
		}
	}

	return response;
}

static u32 send_packet(u32 quadlet, u32 response_mask, u32 response_bits)
{
	return _send_packet(quadlet, ~quadlet, response_mask, response_bits);
}

static void command_config(char *args[])
{
	bool new_root = false;
	int gap_count = -1;
	char *endptr;
	u32 packet;

	if (!args[0]) {
		fputs("missing configuration parameter\n", stderr);
syntax_error:
		help();
		exit(EXIT_FAILURE);
	}

	do {
		if (!strcmp(args[0], "root")) {
			if (!args[1]) {
				fputs("no root node specified\n", stderr);
				goto syntax_error;
			}
			find_param_node(args[1]);
			new_root = true;
		} else if (!strcmp(args[0], "gapcount")) {
			if (!args[1]) {
				fputs("no gap count specified\n", stderr);
				goto syntax_error;
			}
			gap_count = strtol(args[1], &endptr, 0);
			if (*endptr) {
				fputs("gap count is not a number\n", stderr);
				exit(EXIT_FAILURE);
			} else if (gap_count < 0 || gap_count > 63) {
				fputs("gap count out of range\n", stderr);
				exit(EXIT_FAILURE);
			}
		} else {
			fprintf(stderr, "unknown configuration parameter `%s'\n", args[0]);
			goto syntax_error;
		}
		args += 2;
	} while (args[0]);

	packet = 0;
	if (new_root)
		packet |= (1 << 23) | (param_node_id << 24);
	if (gap_count >= 0)
		packet |= (1 << 22) | (gap_count << 16);
	send_packet(packet, 0, 0);
}

static void print_port(u32 self_id, unsigned int shift)
{
	static const char *const port[] = {
		[0] = "",
		[1] = "-",
		[2] = "p",
		[3] = "c",
	};

	fputs(port[(self_id >> shift) & 3], stdout);
}

static void command_ping(char *args[])
{
	static const char *const speed[] = {
		[0] = "S100",
		[1] = "S200",
		[2] = "S400",
		[3] = "beta",
	};
	static const char *const power[] = {
		[0] = "+0W",
		[1] = "+15W",
		[2] = "+30W",
		[3] = "+45W",
		[4] = "-3W",
		[5] = " ?W",
		[6] = "-3..-6W",
		[7] = "-3..-10W",
	};
	unsigned int i;

	if (!args[0]) {
		fputs("missing destination node\n", stderr);
syntax_error:
		help();
		exit(EXIT_FAILURE);
	}
	find_param_node(args[0]);

	if (args[1]) {
		fprintf(stderr, "unexpected parameter `%s'\n", args[1]);
		goto syntax_error;
	}

	send_packet((0 << 18) | (param_node_id << 24),
		    0xff000000,
		    (2 << 30) | (param_node_id << 24));
	printf("time: %u ticks (%llu ns)",
	       ping_time, (ping_time * 1000000uLL + 12288u) / 24576u);
	printf(", selfID: phy %u %s gc=%u %s %s%s%s [",
	       (self_ids[0] >> 24) & 0x3f,
	       speed[(self_ids[0] >> 14) & 3],
	       (self_ids[0] >> 16) & 0x3f,
	       power[(self_ids[0] >> 8) & 7],
	       self_ids[0] & (1 << 22) ? "L" : "",
	       self_ids[0] & (1 << 11) ? "c" : "",
	       self_ids[0] & (1 << 1) ? "i" : "");
	print_port(self_ids[0], 6);
	print_port(self_ids[0], 4);
	print_port(self_ids[0], 2);
	if (self_ids[0] & 1) {
		for (i = 1; i < ARRAY_SIZE(self_ids); ++i) {
			print_port(self_ids[i], 16);
			print_port(self_ids[i], 14);
			print_port(self_ids[i], 12);
			print_port(self_ids[i], 10);
			print_port(self_ids[i], 8);
			print_port(self_ids[i], 6);
			print_port(self_ids[i], 4);
			print_port(self_ids[i], 2);
			if (!(self_ids[i] & 1))
				break;
		}
	}
	puts("]");
}

static void command_read(char *args[])
{
	unsigned int page, port, reg;
	char *endptr;
	u32 packet;
	u32 response;

	if (!args[0]) {
		fputs("missing destination node\n", stderr);
syntax_error:
		help();
		exit(EXIT_FAILURE);
	}
	find_param_node(args[0]);

	if (!args[1]) {
		fputs("missing register number\n", stderr);
		goto syntax_error;
	}
	if (args[2]) {
		if (!args[3]) {
			fputs("missing register number\n", stderr);
			goto syntax_error;
		}

		page = strtol(args[1], &endptr, 0);
		if (*endptr) {
			fputs("invalid page number\n", stderr);
			exit(EXIT_FAILURE);
		}
		if (page > 7) {
			fputs("page number out of range\n", stderr);
			exit(EXIT_FAILURE);
		}

		port = strtol(args[2], &endptr, 0);
		if (*endptr) {
			fputs("invalid port number\n", stderr);
			exit(EXIT_FAILURE);
		}
		if (port > 15) {
			fputs("port number out of range\n", stderr);
			exit(EXIT_FAILURE);
		}

		reg = strtol(args[3], &endptr, 0);
		if (*endptr) {
			fputs("invalid register number\n", stderr);
			exit(EXIT_FAILURE);
		}
		if (reg < 8 || reg > 15) {
			fputs("register number out of range\n", stderr);
			exit(EXIT_FAILURE);
		}

		if (args[4]) {
			fprintf(stderr, "unexpected parameter `%s'\n", args[4]);
			goto syntax_error;
		}
	} else {
		page = 0;
		port = 0;

		reg = strtol(args[1], &endptr, 0);
		if (*endptr) {
			fputs("invalid register number\n", stderr);
			exit(EXIT_FAILURE);
		}
		if (reg > 7) {
			fputs("register number out of range\n", stderr);
			exit(EXIT_FAILURE);
		}

		if (args[2]) {
			fprintf(stderr, "unexpected parameter `%s'\n", args[2]);
			goto syntax_error;
		}
	}

	packet = reg < 8 ? 1 << 18 : 5 << 18;
	packet |= page << 15;
	packet |= port << 11;
	packet |= (reg & 7) << 8;
	packet |= param_node_id << 24;
	response = send_packet(packet, 0xffffff00, packet | (2 << 18));
	printf("value: 0x%02x\n", response & 0xff);
}

static void command_remote_cmd(char *args[], u32 cmd)
{
	unsigned int port;
	char *endptr;
	u32 response;

	if (!args[0]) {
		fputs("missing destination node\n", stderr);
syntax_error:
		help();
		exit(EXIT_FAILURE);
	}
	find_param_node(args[0]);

	if (!args[1]) {
		fputs("missing port number\n", stderr);
		goto syntax_error;
	}
	port = strtol(args[1], &endptr, 0);
	if (*endptr) {
		fputs("invalid port number\n", stderr);
		exit(EXIT_FAILURE);
	}
	if (port > 15) {
		fputs("port number out of range\n", stderr);
		exit(EXIT_FAILURE);
	}

	if (args[2]) {
		fprintf(stderr, "unexpected parameter `%s'\n", args[2]);
		goto syntax_error;
	}

	response = send_packet((0x8 << 18) | cmd | (port << 11) | (param_node_id << 24),
			       0xff3ff807,
			       (0xa << 18) | cmd | (port << 11) | (param_node_id << 24));
	if (!(response & (1 << 3)))
		puts("command rejected");
	else if (!(response & 0x1f0))
		fputs("port status: ok\n", stdout);
	else
		printf("port status:%s%s%s%s%s\n",
		       response & (1 << 4) ? " disabled" : "",
		       response & (1 << 5) ? " bias" : "",
		       response & (1 << 6) ? " connected" : "",
		       response & (1 << 7) ? " fault" : "",
		       response & (1 << 8) ? " standby_fault" : "");
}

static void command_nop(char *args[])
{
	command_remote_cmd(args, 0);
}

static void command_disable(char *args[])
{
	command_remote_cmd(args, 1);
}

static void command_suspend(char *args[])
{
	command_remote_cmd(args, 2);
}

static void command_clear(char *args[])
{
	command_remote_cmd(args, 4);
}

static void command_enable(char *args[])
{
	command_remote_cmd(args, 5);
}

static void command_resume(char *args[])
{
	if (args[0])
		command_remote_cmd(args, 6);
	else
		send_packet((0xf << 18) | (local_node->id << 24), 0, 0);
}

static void command_standby(char *args[])
{
	command_remote_cmd(args, 7 | (1 << 15));
}

static void command_restore(char *args[])
{
	command_remote_cmd(args, 7 | (2 << 15));
}

static void command_linkon(char *args[])
{
	if (!args[0]) {
		fputs("missing destination node\n", stderr);
syntax_error:
		help();
		exit(EXIT_FAILURE);
	}
	find_param_node(args[0]);

	if (args[1]) {
		fprintf(stderr, "unexpected parameter `%s'\n", args[1]);
		goto syntax_error;
	}

	send_packet((1 << 30) | (param_node_id << 24), 0, 0);
}

static void command_versaphy(char *args[])
{
	u32 q0, q1;
	char *endptr;

	if (!args[0] || !args[1]) {
		fputs("missing data\n", stderr);
syntax_error:
		help();
		exit(EXIT_FAILURE);
	}

	q0 = strtol(args[0], &endptr, 16);
	if (*endptr) {
		fputs("invalid data quadlet\n", stderr);
		exit(EXIT_FAILURE);
	}
	q1 = strtol(args[1], &endptr, 16);
	if (*endptr) {
		fputs("invalid data quadlet\n", stderr);
		exit(EXIT_FAILURE);
	}
	if ((q0 & 0xc0000000) != 0xc0000000) {
		fputs("not a VersaPHY packet\n", stderr);
		exit(EXIT_FAILURE);
	}

	if (args[2]) {
		fprintf(stderr, "unexpected parameter `%s'\n", args[2]);
		goto syntax_error;
	}

	_send_packet(q0, q1, 0, 0);
}

static void command_reset(char *args[])
{
	struct fw_cdev_initiate_bus_reset initiate_bus_reset;

	if (args[0]) {
		fprintf(stderr, "unexpected parameter `%s'\n", args[0]);
		help();
		exit(EXIT_FAILURE);
	}

	initiate_bus_reset.type = FW_CDEV_SHORT_RESET;
	if (ioctl(local_node->fd, FW_CDEV_IOC_INITIATE_BUS_RESET, &initiate_bus_reset) < 0) {
		perror("INITIATE_BUS_RESET ioctl failed");
		exit(EXIT_FAILURE);
	}
}

int main(int argc, char *argv[])
{
	static const char short_options[] = "b:hV";
	static const struct option long_options[] = {
		{ "bus", 1, NULL, 'b' },
		{ "help", 0, NULL, 'h' },
		{ "version", 0, NULL, 'V' },
		{}
	};
	static const struct {
		const char *name;
		void (*fn)(char *args[]);
	} commands[] = {
		{ "config",   command_config },
		{ "ping",     command_ping },
		{ "read",     command_read },
		{ "nop",      command_nop },
		{ "disable",  command_disable },
		{ "suspend",  command_suspend },
		{ "clear",    command_clear },
		{ "enable",   command_enable },
		{ "resume",   command_resume },
		{ "standby",  command_standby },
		{ "restore",  command_restore },
		{ "linkon",   command_linkon },
		{ "link-on",  command_linkon },
		{ "link_on",  command_linkon },
		{ "versaphy", command_versaphy },
		{ "reset",    command_reset },
	};
	const char *bus_name = NULL;
	unsigned int i;
	int c;

	while ((c = getopt_long(argc, argv, short_options, long_options, NULL)) != -1) {
		switch (c) {
		case 'b':
			bus_name = optarg;
			break;
		case 'h':
			help();
			return 0;
		case 'V':
			puts("firewire-phy-command version " PACKAGE_VERSION);
			return 0;
		default:
		syntax_error:
			help();
			return 1;
		}
	}

	if (optind >= argc) {
		fputs("missing command\n", stderr);
		goto syntax_error;
	}
	for (i = 0; i < ARRAY_SIZE(commands); ++i)
		if (!strcmp(commands[i].name, argv[optind])) {
			open_all_nodes();
			find_local_node(bus_name);
			commands[i].fn(argv + optind + 1);
			close_all_nodes();
			return 0;
		}

	fprintf(stderr, "unknown command `%s'\n", argv[optind]);
	return 1;
}
