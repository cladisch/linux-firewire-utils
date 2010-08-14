/*
 * firewire-request.c - send requests to FireWire devices
 *
 * Copyright 2010 Clemens Ladisch <clemens@ladisch.de>
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
#include <getopt.h>
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/firewire-cdev.h>
#include <linux/firewire-constants.h>
#include <asm/byteorder.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof *(a))

typedef __u8 u8;
typedef __u32 u32;
typedef __u64 u64;

typedef void (*command_func)(void);

struct data {
	unsigned int length;
	u8 *data;
};

static bool verbose;
static const char *device_name;
static u64 address;
static unsigned int register_length;
static unsigned int read_length;
static struct data data;
static struct data data2;
static int fd;
static u32 generation;

static void open_device(void)
{
	struct fw_cdev_get_info get_info;
	struct fw_cdev_event_bus_reset bus_reset;

	fd = open(device_name, O_RDWR);
	if (fd == -1) {
		perror(device_name);
		exit(EXIT_FAILURE);
	}

	get_info.version = 2;
	get_info.rom_length = 0;
	get_info.rom = 0;
	get_info.bus_reset = (u64)&bus_reset;
	get_info.bus_reset_closure = 0;
	if (ioctl(fd, FW_CDEV_IOC_GET_INFO, &get_info) < 0) {
		perror("GET_INFO ioctl failed");
		exit(EXIT_FAILURE);
	}
	generation = bus_reset.generation;
}

static struct fw_cdev_event_response *wait_for_response(void)
{
	static u8 buf[sizeof(struct fw_cdev_event_response) + 16384];
	struct pollfd pfd;
	int ready, r;
	struct fw_cdev_event_common *event;
	struct fw_cdev_event_bus_reset *event_bus_reset;

	pfd.fd = fd;
	pfd.events = POLLIN;
	for (;;) {
		ready = poll(&pfd, 1, 123);
		if (ready < 0) {
			perror("poll failed");
			exit(EXIT_FAILURE);
		}
		if (!ready) {
			fputs("timeout (no ack)\n", stderr);
			exit(EXIT_FAILURE);
		}
		r = read(fd, buf, sizeof buf);
		if (r < sizeof(struct fw_cdev_event_common)) {
			fputs("short read\n", stderr);
			exit(EXIT_FAILURE);
		}
		event = (void *)buf;
		if (event->type == FW_CDEV_EVENT_RESPONSE)
			return (struct fw_cdev_event_response *)buf;
		if (event->type == FW_CDEV_EVENT_BUS_RESET) {
			event_bus_reset = (void *)buf;
			generation = event_bus_reset->generation;
		}
	}
}

static void print_rcode(u32 rcode)
{
	const char *s;

	switch (rcode) {
	case RCODE_CONFLICT_ERROR:	s = "conflict error";	break;
	case RCODE_DATA_ERROR:		s = "data error";	break;
	case RCODE_TYPE_ERROR:		s = "type error";	break;
	case RCODE_ADDRESS_ERROR:	s = "address error";	break;
	case RCODE_SEND_ERROR:		s = "send error";	break;
	case RCODE_CANCELLED:		s = "error: cancelled";	break;
	case RCODE_BUSY:		s = "error: busy";	break;
	case RCODE_GENERATION:		s = "error: bus reset";	break;
	case RCODE_NO_ACK:		s = "error: no ack";	break;
	default:			s = "unknown error";	break;
	}
	fprintf(stderr, "%s\n", s);
}

static void print_data(const char *prefix, const void *p, unsigned int length, bool allow_value)
{
	const u8 *data = p;
	unsigned int line, col;

	if (allow_value) {
		const u32 *data = p;
		if (length == 4) {
			printf("%s%08x\n", prefix, __be32_to_cpu(*data));
			return;
		}
		if (length == 8) {
			printf("%s%08x%08x\n", prefix, __be32_to_cpu(data[0]), __be32_to_cpu(data[1]));
			return;
		}
	}

	for (line = 0; line < ((length + 15) & ~15); line += 16) {
		printf("%s%03x:", prefix, line);
		for (col = line; col < line + 16 && col < length; ++col)
			printf(" %02x", data[col]);
		printf("%*s", 1 + (line + 16 - col) * 3, "");
		for (col = line; col < line + 16 && col < length; ++col)
			if (data[col] >= 32 && data[col] < 127)
				putchar(data[col]);
			else
				putchar('.');
		putchar('\n');
	}
}

static void do_read(void)
{
	struct fw_cdev_send_request send_request;
	struct fw_cdev_event_response *response;

	do {
		if (read_length == 4 && !(address & 3))
			send_request.tcode = TCODE_READ_QUADLET_REQUEST;
		else
			send_request.tcode = TCODE_READ_BLOCK_REQUEST;
		send_request.length = read_length;
		send_request.offset = address;
		send_request.closure = 0;
		send_request.data = 0;
		send_request.generation = generation;
		if (ioctl(fd, FW_CDEV_IOC_SEND_REQUEST, &send_request) < 0) {
			perror("SEND_REQUEST ioctl failed");
			exit(EXIT_FAILURE);
		}
		response = wait_for_response();
	} while (response->rcode == RCODE_GENERATION);
	if (response->rcode != RCODE_COMPLETE)
		print_rcode(response->rcode);
	else
		print_data("result: ", response->data, response->length, response->length == read_length);
}

static void do_write_request(int request)
{
	struct fw_cdev_send_request send_request;
	struct fw_cdev_event_response *response;

	do {
		if (data.length == 4 && !(address & 3))
			send_request.tcode = TCODE_WRITE_QUADLET_REQUEST;
		else
			send_request.tcode = TCODE_WRITE_BLOCK_REQUEST;
		send_request.length = data.length;
		send_request.offset = address;
		send_request.closure = 0;
		send_request.data = (u64)data.data;
		send_request.generation = generation;
		if (ioctl(fd, request, &send_request) < 0) {
			perror("SEND_REQUEST ioctl failed");
			exit(EXIT_FAILURE);
		}
		response = wait_for_response();
	} while (response->rcode == RCODE_GENERATION);
	if (response->rcode != RCODE_COMPLETE)
		print_rcode(response->rcode);
}

static void do_write(void)
{
	do_write_request(FW_CDEV_IOC_SEND_REQUEST);
}

static void do_broadcast(void)
{
	do_write_request(FW_CDEV_IOC_SEND_BROADCAST_REQUEST);
}

static void do_lock_request(u32 tcode)
{
	bool has_data2;
	u8 *buf;
	struct fw_cdev_send_request send_request;
	struct fw_cdev_event_response *response;

	has_data2 = tcode != TCODE_LOCK_FETCH_ADD && tcode != TCODE_LOCK_LITTLE_ADD;
	if ((data.length != 4 && data.length != 8) ||
	    (has_data2 && (data2.length != 4 && data2.length != 8))) {
		fputs("data size must be 32 or 64 bits\n", stderr);
		exit(EXIT_FAILURE);
	}
	if (has_data2 && data.length != data2.length) {
		fputs("both data blocks must have the same size\n", stderr);
		exit(EXIT_FAILURE);
	}
	if (has_data2) {
		buf = malloc(data.length * 2);
		if (!buf) {
			fputs("out of memory\n", stderr);
			exit(EXIT_FAILURE);
		}
		memcpy(buf, data.data, data.length);
		memcpy(buf + data.length, data2.data, data2.length);
	} else
		buf = data.data;
	do {
		send_request.tcode = tcode;;
		send_request.length = has_data2 ? data.length * 2 : data.length;
		send_request.offset = address;
		send_request.closure = 0;
		send_request.data = (u64)buf;
		send_request.generation = generation;
		if (ioctl(fd, FW_CDEV_IOC_SEND_REQUEST, &send_request) < 0) {
			perror("SEND_REQUEST ioctl failed");
			exit(EXIT_FAILURE);
		}
		response = wait_for_response();
	} while (response->rcode == RCODE_GENERATION);
	if (response->rcode != RCODE_COMPLETE)
		print_rcode(response->rcode);
	else
		print_data("old: ", response->data, response->length, true);
}

static void do_mask_swap(void)
{
	do_lock_request(TCODE_LOCK_MASK_SWAP);
}

static void do_compare_swap(void)
{
	do_lock_request(TCODE_LOCK_COMPARE_SWAP);
}

static void do_add_big(void)
{
	do_lock_request(TCODE_LOCK_FETCH_ADD);
}

static void do_add_little(void)
{
	do_lock_request(TCODE_LOCK_LITTLE_ADD);
}

static void do_bounded_add(void)
{
	do_lock_request(TCODE_LOCK_BOUNDED_ADD);
}

static void do_wrap_add(void)
{
	do_lock_request(TCODE_LOCK_WRAP_ADD);
}

static void send_response(u32 handle, u32 rcode)
{
	struct fw_cdev_send_response send_response;

	send_response.rcode = rcode;
	send_response.length = 0;
	send_response.data = 0;
	send_response.handle = handle;
	if (ioctl(fd, FW_CDEV_IOC_SEND_RESPONSE, &send_response) < 0) {
		perror("SEND_RESPONSE ioctl failed");
		exit(EXIT_FAILURE);
	}
}

static void do_fcp(void)
{
	static u8 buf[sizeof(struct fw_cdev_event_response) + 0x200];
	struct fw_cdev_allocate allocate;
	struct fw_cdev_send_request send_request;
	bool ack_received, response_received;
	struct pollfd pfd;
	int ready, r;
	struct fw_cdev_event_common *event;

	allocate.offset = 0xfffff0000d00uLL; /* FCP response */
	allocate.closure = 0;
	allocate.length = 0x200;
	if (ioctl(fd, FW_CDEV_IOC_ALLOCATE, &allocate) < 0) {
		perror("ALLOCATE ioctl failed");
		exit(EXIT_FAILURE);
	}

	send_request.tcode = data.length == 4 ? TCODE_WRITE_QUADLET_REQUEST : TCODE_WRITE_BLOCK_REQUEST;
	send_request.length = data.length;
	send_request.offset = 0xfffff0000b00uLL; /* FCP command */
	send_request.closure = 0;
	send_request.data = (u64)data.data;
	send_request.generation = generation;
	if (ioctl(fd, FW_CDEV_IOC_SEND_REQUEST, &send_request) < 0) {
		perror("SEND_REQUEST ioctl failed");
		exit(EXIT_FAILURE);
	}

	ack_received = false;
	response_received = false;
	pfd.fd = fd;
	pfd.events = POLLIN;
	while (!ack_received && !response_received) {
		ready = poll(&pfd, 1, 123);
		if (ready < 0) {
			perror("poll failed");
			exit(EXIT_FAILURE);
		}
		if (!ready) {
			fprintf(stderr, "timeout (%s)\n", !ack_received ? "no ack" : "no response");
			exit(EXIT_FAILURE);
		}
		r = read(fd, buf, sizeof buf);
		if (r < sizeof(struct fw_cdev_event_common)) {
			fputs("short read\n", stderr);
			exit(EXIT_FAILURE);
		}
		event = (void *)buf;
		if (event->type == FW_CDEV_EVENT_BUS_RESET) {
			fputs("bus reset\n", stderr);
			exit(EXIT_FAILURE);
		}
		if (event->type == FW_CDEV_EVENT_RESPONSE) {
			struct fw_cdev_event_response *response = (void *)buf;
			if (response->rcode != RCODE_COMPLETE) {
				print_rcode(response->rcode);
				return;
			}
			ack_received = true;
		} else if (event->type == FW_CDEV_EVENT_REQUEST) {
			struct fw_cdev_event_request *request = (void *)buf;
			send_response(request->handle, RCODE_COMPLETE);
			print_data("response: ", request->data, request->length, false);
		}
	}
}

static void do_bus_reset(u32 type)
{
	struct fw_cdev_initiate_bus_reset reset;

	reset.type = type;
	if (ioctl(fd, FW_CDEV_IOC_INITIATE_BUS_RESET, &reset) < 0) {
		perror("INITIATE_BUS_RESET ioctl failed");
		exit(EXIT_FAILURE);
	}
}

static void do_reset(void)
{
	do_bus_reset(FW_CDEV_SHORT_RESET);
}

static void do_long_reset(void)
{
	do_bus_reset(FW_CDEV_LONG_RESET);
}

static const struct command {
	const char *name;
	command_func function;
	bool has_addr;
	bool has_length;
	bool has_data;
	bool has_data2;
} commands[] = {
	{ "read",            do_read,         .has_addr = true, .has_length = true },
	{ "write",           do_write,        .has_addr = true, .has_data = true },
	{ "broadcast",       do_broadcast,    .has_addr = true, .has_data = true },
	{ "mask_swap",       do_mask_swap,    .has_addr = true, .has_data = true, .has_data2 = true },
	{ "compare_swap",    do_compare_swap, .has_addr = true, .has_data = true, .has_data2 = true },
	{ "add",             do_add_big,      .has_addr = true, .has_data = true },
	{ "add_big",         do_add_big,      .has_addr = true, .has_data = true },
	{ "add_little",      do_add_little,   .has_addr = true, .has_data = true },
	{ "bounded_add",     do_bounded_add,  .has_addr = true, .has_data = true, .has_data2 = true },
	{ "bounded_add_big", do_bounded_add,  .has_addr = true, .has_data = true, .has_data2 = true },
	{ "wrap_add",        do_wrap_add,     .has_addr = true, .has_data = true, .has_data2 = true },
	{ "wrap_add_big",    do_wrap_add,     .has_addr = true, .has_data = true, .has_data2 = true },
	{ "fcp",             do_fcp,                            .has_data = true },
	{ "reset",           do_reset },
	{ "long_reset",      do_long_reset },
};

static const struct register_name {
	u64 address;
	unsigned int size;
	const char *name;
	bool hide;
#define HIDDEN true
} register_names[] = {
	{ 0xfffff0000000uLL, 4, "state_clear" },
	{ 0xfffff0000004uLL, 4, "state_set" },
	{ 0xfffff0000008uLL, 4, "node_ids" },
	{ 0xfffff000000cuLL, 4, "reset_start" },
	{ 0xfffff0000018uLL, 8, "split_timeout" },
	{ 0xfffff0000018uLL, 4, "split_timeout_hi" },
	{ 0xfffff000001cuLL, 4, "split_timeout_lo" },
	{ 0xfffff0000020uLL, 8, "argument", HIDDEN },
	{ 0xfffff0000020uLL, 4, "argument_hi", HIDDEN },
	{ 0xfffff0000024uLL, 4, "argument_lo", HIDDEN },
	{ 0xfffff0000028uLL, 4, "test_start", HIDDEN },
	{ 0xfffff000002cuLL, 4, "test_status", HIDDEN },
	{ 0xfffff0000050uLL, 4, "interrupt_target", HIDDEN },
	{ 0xfffff0000054uLL, 4, "interrupt_mask", HIDDEN },
	{ 0xfffff0000080uLL, 64, "message_request" },
	{ 0xfffff00000c0uLL, 64, "message_response" },
	{ 0xfffff0000180uLL, 128, "error_log_buffer", HIDDEN },
	{ 0xfffff0000200uLL, 4, "cycle_time" },
	{ 0xfffff0000204uLL, 4, "bus_time" },
	{ 0xfffff0000208uLL, 4, "power_fail_imminent", HIDDEN },
	{ 0xfffff000020cuLL, 4, "power_source", HIDDEN },
	{ 0xfffff0000210uLL, 4, "busy_timeout" },
	{ 0xfffff0000214uLL, 4, "quarantine", HIDDEN },
	{ 0xfffff0000218uLL, 4, "priority_budget" },
	{ 0xfffff000021cuLL, 4, "bus_manager_id" },
	{ 0xfffff0000220uLL, 4, "bandwidth_available" },
	{ 0xfffff0000224uLL, 8, "channels_available" },
	{ 0xfffff0000224uLL, 4, "channels_available_hi" },
	{ 0xfffff0000228uLL, 4, "channels_available_lo" },
	{ 0xfffff000022cuLL, 4, "maint_control", HIDDEN },
	{ 0xfffff0000230uLL, 4, "maint_utility" },
	{ 0xfffff0000234uLL, 4, "broadcast_channel" },
	{ 0xfffff0000400uLL, 0x400, "config_rom" },
	{ 0xfffff0000900uLL, 4, "output_master_plug" },
	{ 0xfffff0000904uLL, 4, "output_plug0" },
	{ 0xfffff0000908uLL, 4, "output_plug1", HIDDEN },
	{ 0xfffff000090cuLL, 4, "output_plug2", HIDDEN },
	{ 0xfffff0000910uLL, 4, "output_plug3", HIDDEN },
	{ 0xfffff0000914uLL, 4, "output_plug4", HIDDEN },
	{ 0xfffff0000918uLL, 4, "output_plug5", HIDDEN },
	{ 0xfffff000091cuLL, 4, "output_plug6", HIDDEN },
	{ 0xfffff0000920uLL, 4, "output_plug7", HIDDEN },
	{ 0xfffff0000924uLL, 4, "output_plug8", HIDDEN },
	{ 0xfffff0000928uLL, 4, "output_plug9", HIDDEN },
	{ 0xfffff000092cuLL, 4, "output_plug10", HIDDEN },
	{ 0xfffff0000930uLL, 4, "output_plug11", HIDDEN },
	{ 0xfffff0000934uLL, 4, "output_plug12", HIDDEN },
	{ 0xfffff0000938uLL, 4, "output_plug13", HIDDEN },
	{ 0xfffff000093cuLL, 4, "output_plug14", HIDDEN },
	{ 0xfffff0000940uLL, 4, "output_plug15", HIDDEN },
	{ 0xfffff0000944uLL, 4, "output_plug16", HIDDEN },
	{ 0xfffff0000948uLL, 4, "output_plug17", HIDDEN },
	{ 0xfffff000094cuLL, 4, "output_plug18", HIDDEN },
	{ 0xfffff0000950uLL, 4, "output_plug19", HIDDEN },
	{ 0xfffff0000954uLL, 4, "output_plug20", HIDDEN },
	{ 0xfffff0000958uLL, 4, "output_plug21", HIDDEN },
	{ 0xfffff000095cuLL, 4, "output_plug22", HIDDEN },
	{ 0xfffff0000960uLL, 4, "output_plug23", HIDDEN },
	{ 0xfffff0000964uLL, 4, "output_plug24", HIDDEN },
	{ 0xfffff0000968uLL, 4, "output_plug25", HIDDEN },
	{ 0xfffff000096cuLL, 4, "output_plug26", HIDDEN },
	{ 0xfffff0000970uLL, 4, "output_plug27", HIDDEN },
	{ 0xfffff0000974uLL, 4, "output_plug28", HIDDEN },
	{ 0xfffff0000978uLL, 4, "output_plug29", HIDDEN },
	{ 0xfffff000097cuLL, 4, "output_plug30" },
	{ 0xfffff0000980uLL, 4, "input_master_plug" },
	{ 0xfffff0000984uLL, 4, "input_plug0" },
	{ 0xfffff0000988uLL, 4, "input_plug1", HIDDEN },
	{ 0xfffff000098cuLL, 4, "input_plug2", HIDDEN },
	{ 0xfffff0000990uLL, 4, "input_plug3", HIDDEN },
	{ 0xfffff0000994uLL, 4, "input_plug4", HIDDEN },
	{ 0xfffff0000998uLL, 4, "input_plug5", HIDDEN },
	{ 0xfffff000099cuLL, 4, "input_plug6", HIDDEN },
	{ 0xfffff00009a0uLL, 4, "input_plug7", HIDDEN },
	{ 0xfffff00009a4uLL, 4, "input_plug8", HIDDEN },
	{ 0xfffff00009a8uLL, 4, "input_plug9", HIDDEN },
	{ 0xfffff00009acuLL, 4, "input_plug10", HIDDEN },
	{ 0xfffff00009b0uLL, 4, "input_plug11", HIDDEN },
	{ 0xfffff00009b4uLL, 4, "input_plug12", HIDDEN },
	{ 0xfffff00009b8uLL, 4, "input_plug13", HIDDEN },
	{ 0xfffff00009bcuLL, 4, "input_plug14", HIDDEN },
	{ 0xfffff00009c0uLL, 4, "input_plug15", HIDDEN },
	{ 0xfffff00009c4uLL, 4, "input_plug16", HIDDEN },
	{ 0xfffff00009c8uLL, 4, "input_plug17", HIDDEN },
	{ 0xfffff00009ccuLL, 4, "input_plug18", HIDDEN },
	{ 0xfffff00009d0uLL, 4, "input_plug19", HIDDEN },
	{ 0xfffff00009d4uLL, 4, "input_plug20", HIDDEN },
	{ 0xfffff00009d8uLL, 4, "input_plug21", HIDDEN },
	{ 0xfffff00009dcuLL, 4, "input_plug22", HIDDEN },
	{ 0xfffff00009e0uLL, 4, "input_plug23", HIDDEN },
	{ 0xfffff00009e4uLL, 4, "input_plug24", HIDDEN },
	{ 0xfffff00009e8uLL, 4, "input_plug25", HIDDEN },
	{ 0xfffff00009ecuLL, 4, "input_plug26", HIDDEN },
	{ 0xfffff00009f0uLL, 4, "input_plug27", HIDDEN },
	{ 0xfffff00009f4uLL, 4, "input_plug28", HIDDEN },
	{ 0xfffff00009f8uLL, 4, "input_plug29", HIDDEN },
	{ 0xfffff00009fcuLL, 4, "input_plug30" },
	{ 0xfffff0000b00uLL, 0x200, "fcp_command" },
	{ 0xfffff0000d00uLL, 0x200, "fcp_response" },
	{ 0xfffff0001000uLL, 0x400, "topology_map" },
	{ 0xfffff0001c00uLL, 0x200, "virtual_id_map", HIDDEN },
	{ 0xfffff0001e00uLL, 0x100, "route_map", HIDDEN },
	{ 0xfffff0001f00uLL, 8, "clan_eui_64", HIDDEN },
	{ 0xfffff0001f08uLL, 4, "clan_info", HIDDEN },
	{ 0xfffff0002000uLL, 0x1000, "speed_map", HIDDEN },
};

static void parse_address(const char *s)
{
	char *endptr;
	unsigned int i;

	address = strtoull(s, &endptr, 16);
	if (*endptr != '\0') {
		for (i = 0; i < ARRAY_SIZE(register_names); ++i)
			if (!strcasecmp(s, register_names[i].name)) {
				address = register_names[i].address;
				register_length = register_names[i].size;
				return;
			}
		fprintf(stderr, "invalid address: `%s'\n", s);
		exit(EXIT_FAILURE);
	}
}

static void parse_read_length(const char *s)
{
	char *endptr;
	unsigned long int l;

	l = strtoul(s, &endptr, 16);
	if (*endptr != '\0') {
		fprintf(stderr, "invalid length: `%s'\n", s);
		exit(EXIT_FAILURE);
	}
	read_length = l;
}

static void parse_data(const char *s, struct data *data)
{
	unsigned int len;
	bool high_nibble = true;
	u8 val;

	while (isspace(*s))
		++s;
	if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
		s += 2;
       
	len = strlen(s);
	data->data = malloc(len / 2 + 1);
	if (!data->data) {
		fputs("out of memory\n", stderr);
		exit(EXIT_FAILURE);
	}

	for (len = 0; *s != '\0'; ++s) {
		if (isspace(*s) || *s == '_')
			continue;
		if ('0' <= *s && *s <= '9')
			val = *s - '0';
		else if ('A' <= *s && *s <= 'F')
			val = *s - 'A' + 10;
		else if ('a' <= *s && *s <= 'f')
			val = *s - 'a' + 10;
		else {
			fprintf(stderr, "invalid character in data: `%c'\n", *s);
			exit(EXIT_FAILURE);
		}
		if (high_nibble) {
			data->data[len] = val << 4;
			high_nibble = false;
		} else {
			data->data[len++] |= val;
			high_nibble = true;
		}
	}
	if (!high_nibble) {
		fputs("data ends in a half byte\n", stderr);
		exit(EXIT_FAILURE);
	}
	if (register_length && register_length <= 8 && register_length != len) {
		fprintf(stderr, "data for this register must have %u bits\n", register_length * 8);
		exit(EXIT_FAILURE);
	}
	data->length = len;
}

static void help(void)
{
	fputs("firewire-request <dev> read <addr> [<length>]\n"
	      "firewire-request <dev> write <addr> <data>\n"
	      "firewire-request <dev> <locktype> <addr> <data> [<data>]\n"
	      "firewire-request <dev> broadcast <addr> <data>\n"
	      "firewire-request <dev> fcp <data>\n"
	      "firewire-request <dev> reset|long_reset\n"
	      "\n"
	      "<dev> is device node (/dev/fwX)\n"
	      "<addr> is address in hex or register name\n"
	      "<length> is byte length in hex, default from register or 4\n"
	      "<data> is data bytes in hex (spaces must be quoted)\n"
	      "<locktype> is mask_swap|compare_swap|add_big|add_little|bounded_add|wrap_add\n"
	      "\n"
	      "Options:\n"
	      " -D,--dump-register-names  show known register names and exit\n"
	      " -v,--verbose              more information\n"
	      " -h,--help                 show this message and exit\n"
	      " -V,--version              show version number and exit\n"
	      "\n"
	      "Report bugs to <" PACKAGE_BUGREPORT ">.\n"
	      PACKAGE_NAME " home page: <" PACKAGE_URL ">.\n",
	      stderr);
}

static void help_registers(void)
{
	unsigned int i;

	puts("address    length name");
	for (i = 0; i < ARRAY_SIZE(register_names); ++i) {
		bool hi_lo;
		if (register_names[i].hide && !verbose)
			continue;
		hi_lo = !verbose && i < ARRAY_SIZE(register_names) - 2 &&
			register_names[i].address == register_names[i + 1].address;
		printf("%012Lx %4x %s%s\n", register_names[i].address,
		       register_names[i].size, register_names[i].name,
		       hi_lo ? "[_hi|_lo]" : "");
		if (hi_lo)
			i += 2;
	}
}

static command_func parse_parameters(int argc, char *argv[])
{
	static const char short_options[] = "vDh";
	static const struct option long_options[] = {
		{ "dump-register-names", 0, NULL, 'D' },
		{ "verbose", 0, NULL, 'v' },
		{ "help", 0, NULL, 'h' },
		{ "version", 0, NULL, 'V' },
		{}
	};
	int c;
	bool show_regs = false, show_help = false, show_version = false;
	const struct command *command;

	while ((c = getopt_long(argc, argv, short_options, long_options, NULL)) != -1) {
		switch (c) {
		case 'D':
			show_regs = true;
			break;
		case 'v':
			verbose = true;
			break;
		case 'h':
			show_help = true;
			break;
		case 'V':
			show_version = true;
			break;
		default:
		syntax_error:
			help();
			exit(EXIT_FAILURE);
		}
	}

	if (show_regs)
		help_registers();
	if (show_help)
		help();
	if (show_version)
		puts("firewire-request version " PACKAGE_VERSION);
	if (show_regs || show_help || show_version)
		exit(EXIT_SUCCESS);

	if (optind >= argc)
		goto syntax_error;
	device_name = argv[optind++];

	if (optind >= argc)
		goto syntax_error;
	for (c = 0; c < ARRAY_SIZE(commands); ++c)
		if (!strcasecmp(argv[optind], commands[c].name)) {
			command = &commands[c];
			goto command_found;
		}
	fprintf(stderr, "unknown command: `%s'\n", argv[optind]);
	goto syntax_error;
command_found:
	++optind;

	if (command->has_addr) {
		if (optind >= argc)
			goto syntax_error;
		parse_address(argv[optind++]);
	}

	if (command->has_length) {
		if (optind < argc)
			parse_read_length(argv[optind++]);
		else if (register_length)
			read_length = register_length;
		else
			read_length = 4;
	}

	if (command->has_data) {
		if (optind >= argc)
			goto syntax_error;
		parse_data(argv[optind++], &data);
	}

	if (command->has_data2) {
		if (optind >= argc)
			goto syntax_error;
		parse_data(argv[optind++], &data2);
	}

	if (optind < argc) {
		fprintf(stderr, "superfluous parameter: `%s'\n", argv[optind]);
		goto syntax_error;
	}

	return command->function;
}

int main(int argc, char *argv[])
{
	command_func fn;

	fn = parse_parameters(argc, argv);
	open_device();
	fn();
	close(fd);
	return 0;
}
