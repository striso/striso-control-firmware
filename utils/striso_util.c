/*
 * striso_util: Striso USB toolbox
 * Copyright © 2016-2019 Piers Titus van der Torren <pierstitus@striso.org>
 *
 * Based on xusb by Pete Batard <pete@akeo.ie>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>

#include "libusb.h"

#if defined(_WIN32)
#define msleep(msecs) Sleep(msecs)
#else
#include <time.h>
#define msleep(msecs) nanosleep(&(struct timespec){msecs / 1000, (msecs * 1000000) % 1000000000UL}, NULL);
#endif

#if defined(_MSC_VER)
#define snprintf _snprintf
#define putenv _putenv
#endif

#if !defined(bool)
#define bool int
#endif
#if !defined(true)
#define true (1 == 1)
#endif
#if !defined(false)
#define false (!true)
#endif

// override printf to print to stderr
#define printf perr

// Future versions of libusb will use usb_interface instead of interface
// in libusb_config_descriptor => catter for that
#define usb_interface interface

// Global variables
static bool binary_dump = false;
static bool extra_info = false;
static const char* binary_name = NULL;

static int perr(char const *format, ...)
{
	va_list args;
	int r;

	va_start (args, format);
	r = vfprintf(stderr, format, args);
	va_end(args);

	return r;
}

#define ERR_EXIT(errcode) do { perr("   %s\n", libusb_strerror((enum libusb_error)errcode)); return -1; } while (0)
#define CALL_CHECK(fcall) do { r=fcall; if (r < 0) ERR_EXIT(r); } while (0);
#define B(x) (((x)!=0)?1:0)
#define be_to_int32(buf) (((buf)[0]<<24)|((buf)[1]<<16)|((buf)[2]<<8)|(buf)[3])

#define RETRY_MAX                     5
#define REQUEST_SENSE_LENGTH          0x12
#define INQUIRY_LENGTH                0x24
#define READ_CAPACITY_LENGTH          0x08

// Section 5.1: Command Block Wrapper (CBW)
struct command_block_wrapper {
	uint8_t dCBWSignature[4];
	uint32_t dCBWTag;
	uint32_t dCBWDataTransferLength;
	uint8_t bmCBWFlags;
	uint8_t bCBWLUN;
	uint8_t bCBWCBLength;
	uint8_t CBWCB[16];
};

// Section 5.2: Command Status Wrapper (CSW)
struct command_status_wrapper {
	uint8_t dCSWSignature[4];
	uint32_t dCSWTag;
	uint32_t dCSWDataResidue;
	uint8_t bCSWStatus;
};

static uint8_t cdb_length[256] = {
//	 0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  0
	06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,06,  //  1
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  2
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  3
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  4
	10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,  //  5
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  6
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  7
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  8
	16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,  //  9
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  A
	12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,12,  //  B
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  C
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  D
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  E
	00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,00,  //  F
};

static enum test_type {
	USE_GENERIC,
	USE_PIPEBULK,
	USE_PIPETEXT,
	USE_PIPESTOP,
	USE_VERSION,
	USE_SYSINFO,
	USE_CALIBRATION,
	USE_DFU,
	USE_BOOTLOADER,
} test_mode;
static uint16_t VID, PID;

static void display_buffer_hex(unsigned char *buffer, unsigned size)
{
	unsigned i, j, k;

	for (i=0; i<size; i+=16) {
		printf("\n  %08x  ", i);
		for(j=0,k=0; k<16; j++,k++) {
			if (i+j < size) {
				printf("%02x", buffer[i+j]);
			} else {
				printf("  ");
			}
			printf(" ");
		}
		printf(" ");
		for(j=0,k=0; k<16; j++,k++) {
			if (i+j < size) {
				if ((buffer[i+j] < 32) || (buffer[i+j] > 126)) {
					printf(".");
				} else {
					printf("%c", buffer[i+j]);
				}
			}
		}
	}
	printf("\n" );
}

static char* uuid_to_string(const uint8_t* uuid)
{
	static char uuid_string[40];
	if (uuid == NULL) return NULL;
	sprintf(uuid_string, "{%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x}",
		uuid[0], uuid[1], uuid[2], uuid[3], uuid[4], uuid[5], uuid[6], uuid[7],
		uuid[8], uuid[9], uuid[10], uuid[11], uuid[12], uuid[13], uuid[14], uuid[15]);
	return uuid_string;
}

// Pipe data from Striso bulk device to stdout
static int pipe_bulk(libusb_device_handle *handle, uint8_t endpoint_in, uint8_t endpoint_out)
{
	int r, size;
	uint8_t buffer[64];
	memset(buffer, 0, sizeof(buffer));

	// enable Striso binary protocol
	printf("Enable Striso binary protocol:\n");
	buffer[0] = 'S';
	buffer[1] = 't';
	buffer[2] = 'c';
	buffer[3] = 'S';
	CALL_CHECK(libusb_bulk_transfer(handle, endpoint_out, (unsigned char*)&buffer, 4, &size, 100));
	//printf("   send %d bytes\n", size);

	while (r == LIBUSB_SUCCESS || r == LIBUSB_ERROR_TIMEOUT) {
		r = libusb_bulk_transfer(handle, endpoint_in, (unsigned char*)&buffer, sizeof(buffer), &size, 100);
		//printf("   received %d bytes\n", size);
		write(1, buffer, size);
	}
	ERR_EXIT(r);

	return 0;
}

void unpack(uint8_t *in, int *out, int n) {
    int c;
    for (c=0; c<n; c++) {
        out[c] = ( ((int)in[c*2])<<7 | ((int)in[c*2+1]) ) - ( (int)(in[c*2] & 0x40) << 8);
    }
}

// Pipe data from Striso bulk device to stdout
static int pipe_text(libusb_device_handle *handle, uint8_t endpoint_in, uint8_t endpoint_out)
{
	int r, size;
	uint8_t buffer[512];
	memset(buffer, 0, sizeof(buffer));

	// enable Striso binary protocol
	printf("Enable Striso binary protocol:\n");
	buffer[0] = 'S';
	buffer[1] = 't';
	buffer[2] = 'c';
	buffer[3] = 'S';
	CALL_CHECK(libusb_bulk_transfer(handle, endpoint_out, (unsigned char*)&buffer, 4, &size, 100));
	//printf("   send %d bytes\n", size);

	#define MAX_MSGSIZE 16
	uint8_t cmsg[MAX_MSGSIZE];
	int msg[MAX_MSGSIZE/2];
	int idx = 0;
	int done = 0;
	int lostbytes = 0;
	int rxsize, msgsize = 0;

	while (r == LIBUSB_SUCCESS || r == LIBUSB_ERROR_TIMEOUT) {
		r = libusb_bulk_transfer(handle, endpoint_in, (unsigned char*)&buffer, sizeof(buffer), &size, 100);
		//printf("   received %d bytes\n", size);
		idx = -1;

		while (++idx < size) {
			if (done == 0) {
				// Read next message
				if (!(buffer[idx] & 0x80)) {
					lostbytes++;
				} else {
					cmsg[0] = buffer[idx];

					rxsize = ((cmsg[0] & 0x07)+1) * 2;
					msgsize = cmsg[0] & 0x07;

					if (rxsize > MAX_MSGSIZE) {
						// error!
						continue;
					}

					done = 1;
				}
			} else {
				if (buffer[idx] & 0x80) {
					// message error, skip to next message
					done = 0;
					continue;
				}
				cmsg[done] = buffer[idx];
				done++;
				if (done == rxsize) {
					// message OK, print as text 
					int src = (cmsg[0] & 0x7f)>>3;
					int id = cmsg[1];
					unpack(&cmsg[2], msg, msgsize);
					fprintf(stdout, "%d,%d", src, id);
					for (int i=0; i<msgsize; i++) {
						fprintf(stdout, ",%d", msg[i]);
					}
					fprintf(stdout, "\n");
					done = 0;
				}
			}
		}
	}
	ERR_EXIT(r);

	return 0;
}

// Send command to Striso bulk device
static int striso_command(libusb_device_handle *handle, uint8_t endpoint_in, uint8_t endpoint_out, uint8_t command)
{
	int r, size = 0, size2 = 0, count = 0;
	uint8_t buffer[512];
	memset(buffer, 0, sizeof(buffer));

	// send Striso command
	buffer[0] = 'S';
	buffer[1] = 't';
	buffer[2] = 'c';
	buffer[3] = command;
	CALL_CHECK(libusb_bulk_transfer(handle, endpoint_out, (unsigned char*)&buffer, 4, &size, 100));
	// printf("   send %d bytes\n", size);

	while (size || size2 || !count) {
		// store last size as there seems to happen 0 size calls every now and then.
		size2 = size;
		libusb_bulk_transfer(handle, endpoint_in, (unsigned char*)&buffer, sizeof(buffer), &size, 10);
		// printf("   received %d bytes\n", size);
		count += size;
		r = write(1, buffer, size);
		// CALL_CHECK(libusb_bulk_transfer(handle, endpoint_in, (unsigned char*)&buffer, sizeof(buffer), &size, 1000));
		// printf("   received %d bytes\n", size);
		// count += size;
		// r = write(1, buffer, size);
	}

	return 0;
}

static void print_device_cap(struct libusb_bos_dev_capability_descriptor *dev_cap)
{
	switch(dev_cap->bDevCapabilityType) {
	case LIBUSB_BT_USB_2_0_EXTENSION: {
		struct libusb_usb_2_0_extension_descriptor *usb_2_0_ext = NULL;
		libusb_get_usb_2_0_extension_descriptor(NULL, dev_cap, &usb_2_0_ext);
		if (usb_2_0_ext) {
			printf("    USB 2.0 extension:\n");
			printf("      attributes             : %02X\n", usb_2_0_ext->bmAttributes);
			libusb_free_usb_2_0_extension_descriptor(usb_2_0_ext);
		}
		break;
	}
	case LIBUSB_BT_SS_USB_DEVICE_CAPABILITY: {
		struct libusb_ss_usb_device_capability_descriptor *ss_usb_device_cap = NULL;
		libusb_get_ss_usb_device_capability_descriptor(NULL, dev_cap, &ss_usb_device_cap);
		if (ss_usb_device_cap) {
			printf("    USB 3.0 capabilities:\n");
			printf("      attributes             : %02X\n", ss_usb_device_cap->bmAttributes);
			printf("      supported speeds       : %04X\n", ss_usb_device_cap->wSpeedSupported);
			printf("      supported functionality: %02X\n", ss_usb_device_cap->bFunctionalitySupport);
			libusb_free_ss_usb_device_capability_descriptor(ss_usb_device_cap);
		}
		break;
	}
	case LIBUSB_BT_CONTAINER_ID: {
		struct libusb_container_id_descriptor *container_id = NULL;
		libusb_get_container_id_descriptor(NULL, dev_cap, &container_id);
		if (container_id) {
			printf("    Container ID:\n      %s\n", uuid_to_string(container_id->ContainerID));
			libusb_free_container_id_descriptor(container_id);
		}
		break;
	}
	default:
		printf("    Unknown BOS device capability %02x:\n", dev_cap->bDevCapabilityType);
	}
}

static int test_device(uint16_t vid, uint16_t pid)
{
	libusb_device_handle *handle;
	libusb_device *dev;
	uint8_t bus, port_path[8];
	struct libusb_bos_descriptor *bos_desc;
	struct libusb_config_descriptor *conf_desc;
	const struct libusb_endpoint_descriptor *endpoint;
	int i, j, k, r;
	int iface, nb_ifaces, first_iface = -1;
	struct libusb_device_descriptor dev_desc;
	const char* speed_name[5] = { "Unknown", "1.5 Mbit/s (USB LowSpeed)", "12 Mbit/s (USB FullSpeed)",
		"480 Mbit/s (USB HighSpeed)", "5000 Mbit/s (USB SuperSpeed)"};
	char string[128];
	uint8_t string_index[3];	// indexes of the string descriptors
	uint8_t endpoint_in = 0, endpoint_out = 0;	// default IN and OUT endpoints

	if (extra_info)
		printf("Opening device %04X:%04X...\n", vid, pid);
	handle = libusb_open_device_with_vid_pid(NULL, vid, pid);

	if (handle == NULL) {
		perr("  Opening device %04X:%04X failed.\n", vid, pid);
		return -1;
	}

	dev = libusb_get_device(handle);
	bus = libusb_get_bus_number(dev);
	if (extra_info) {
		r = libusb_get_port_numbers(dev, port_path, sizeof(port_path));
		if (r > 0) {
			printf("\nDevice properties:\n");
			printf("        bus number: %d\n", bus);
			printf("         port path: %d", port_path[0]);
			for (i=1; i<r; i++) {
				printf("->%d", port_path[i]);
			}
			printf(" (from root hub)\n");
		}
		r = libusb_get_device_speed(dev);
		if ((r<0) || (r>4)) r=0;
		printf("             speed: %s\n", speed_name[r]);

		printf("\nReading device descriptor:\n");
		CALL_CHECK(libusb_get_device_descriptor(dev, &dev_desc));
		printf("            length: %d\n", dev_desc.bLength);
		printf("      device class: %d\n", dev_desc.bDeviceClass);
		printf("               S/N: %d\n", dev_desc.iSerialNumber);
		printf("           VID:PID: %04X:%04X\n", dev_desc.idVendor, dev_desc.idProduct);
		printf("         bcdDevice: %04X\n", dev_desc.bcdDevice);
		printf("   iMan:iProd:iSer: %d:%d:%d\n", dev_desc.iManufacturer, dev_desc.iProduct, dev_desc.iSerialNumber);
		printf("          nb confs: %d\n", dev_desc.bNumConfigurations);
		// Copy the string descriptors for easier parsing
		string_index[0] = dev_desc.iManufacturer;
		string_index[1] = dev_desc.iProduct;
		string_index[2] = dev_desc.iSerialNumber;

		printf("\nReading BOS descriptor: ");
		if (libusb_get_bos_descriptor(handle, &bos_desc) == LIBUSB_SUCCESS) {
			printf("%d caps\n", bos_desc->bNumDeviceCaps);
			for (i = 0; i < bos_desc->bNumDeviceCaps; i++)
				print_device_cap(bos_desc->dev_capability[i]);
			libusb_free_bos_descriptor(bos_desc);
		} else {
			printf("no descriptor\n");
		}

		printf("\nReading first configuration descriptor:\n");
	}
	CALL_CHECK(libusb_get_config_descriptor(dev, 0, &conf_desc));
	nb_ifaces = conf_desc->bNumInterfaces;
	if (extra_info) {
		printf("             nb interfaces: %d\n", nb_ifaces);
		if (nb_ifaces > 0)
			first_iface = conf_desc->usb_interface[0].altsetting[0].bInterfaceNumber;
		for (i=0; i<nb_ifaces; i++) {
			printf("              interface[%d]: id = %d\n", i,
				conf_desc->usb_interface[i].altsetting[0].bInterfaceNumber);
			for (j=0; j<conf_desc->usb_interface[i].num_altsetting; j++) {
				printf("interface[%d].altsetting[%d]: num endpoints = %d\n",
					i, j, conf_desc->usb_interface[i].altsetting[j].bNumEndpoints);
				printf("   Class.SubClass.Protocol: %02X.%02X.%02X\n",
					conf_desc->usb_interface[i].altsetting[j].bInterfaceClass,
					conf_desc->usb_interface[i].altsetting[j].bInterfaceSubClass,
					conf_desc->usb_interface[i].altsetting[j].bInterfaceProtocol);
				for (k=0; k<conf_desc->usb_interface[i].altsetting[j].bNumEndpoints; k++) {
					struct libusb_ss_endpoint_companion_descriptor *ep_comp = NULL;
					endpoint = &conf_desc->usb_interface[i].altsetting[j].endpoint[k];
					printf("       endpoint[%d].address: %02X\n", k, endpoint->bEndpointAddress);
					// Use the first interrupt or bulk IN/OUT endpoints as default for testing
					if ((endpoint->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) & (LIBUSB_TRANSFER_TYPE_BULK | LIBUSB_TRANSFER_TYPE_INTERRUPT)) {
						if (endpoint->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
							//if (!endpoint_in)
								endpoint_in = endpoint->bEndpointAddress;
						} else {
							//if (!endpoint_out)
								endpoint_out = endpoint->bEndpointAddress;
						}
					}
					printf("           max packet size: %04X\n", endpoint->wMaxPacketSize);
					printf("          polling interval: %02X\n", endpoint->bInterval);
					libusb_get_ss_endpoint_companion_descriptor(NULL, endpoint, &ep_comp);
					if (ep_comp) {
						printf("                 max burst: %02X   (USB 3.0)\n", ep_comp->bMaxBurst);
						printf("        bytes per interval: %04X (USB 3.0)\n", ep_comp->wBytesPerInterval);
						libusb_free_ss_endpoint_companion_descriptor(ep_comp);
					}
				}
			}
		}
	}

	libusb_set_auto_detach_kernel_driver(handle, 1);
	iface = 2;
	for (k=0; k<conf_desc->usb_interface[iface].altsetting[0].bNumEndpoints; k++) {
		endpoint = &conf_desc->usb_interface[iface].altsetting[0].endpoint[k];
		// Use the first interrupt or bulk IN/OUT endpoints as default
		if ((endpoint->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) & LIBUSB_TRANSFER_TYPE_BULK) {
			if (endpoint->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
				if (!endpoint_in)
					endpoint_in = endpoint->bEndpointAddress;
			} else {
				if (!endpoint_out)
					endpoint_out = endpoint->bEndpointAddress;
			}
		}
	}
	libusb_free_config_descriptor(conf_desc);

	if (extra_info)
		printf("\nClaiming interface %d...\n", iface);
	r = libusb_claim_interface(handle, iface);
	if (r != LIBUSB_SUCCESS) {
		perr("  Claiming interface %d failed.\n", iface);
		return -1;
	}

	if (extra_info) {
		printf("\nReading string descriptors:\n");
		for (i=0; i<3; i++) {
			if (string_index[i] == 0) {
				continue;
			}
			if (libusb_get_string_descriptor_ascii(handle, string_index[i], (unsigned char*)string, 128) >= 0) {
				printf("   String (0x%02X): \"%s\"\n", string_index[i], string);
			}
		}
		// Read the OS String Descriptor
		if (libusb_get_string_descriptor_ascii(handle, 0xEE, (unsigned char*)string, 128) >= 0) {
			printf("   String (0x%02X): \"%s\"\n", 0xEE, string);
		}
	}

	switch(test_mode) {
	case USE_VERSION:
		striso_command(handle, endpoint_in, endpoint_out, 'V');
		break;
	case USE_SYSINFO:
		striso_command(handle, endpoint_in, endpoint_out, 'I');
		break;
	case USE_CALIBRATION:
		striso_command(handle, endpoint_in, endpoint_out, 'C');
		break;
	case USE_DFU:
		striso_command(handle, endpoint_in, endpoint_out, 'D');
		break;
	case USE_BOOTLOADER:
		striso_command(handle, endpoint_in, endpoint_out, 'B');
		break;
	case USE_PIPEBULK:
		pipe_bulk(handle, endpoint_in, endpoint_out);
		break;
	case USE_PIPETEXT:
		pipe_text(handle, endpoint_in, endpoint_out);
		break;
	case USE_PIPESTOP:
		striso_command(handle, endpoint_in, endpoint_out, 's');
		break;
	case USE_GENERIC:
		break;
	}

	printf("\n");
	for (iface = 2; iface<nb_ifaces; iface++) {
		if (extra_info)
			printf("Releasing interface %d...\n", iface);
		libusb_release_interface(handle, iface);
	}

	if (extra_info)
		printf("Closing device...\n");
	libusb_close(handle);

	return 0;
}

int main(int argc, char** argv)
{
	bool show_help = false;
	bool debug_mode = false;
	bool disconnect_reopen = false;
	const struct libusb_version* version;
	int j, r;
	size_t arglen;
	uint16_t endian_test = 0xBE00;
	char *error_lang = NULL, *old_dbg_str = NULL, str[256];

	// Default to generic, expecting VID:PID
	VID = 0;
	PID = 0;
	test_mode = USE_GENERIC;

	// Striso - 3 interfaces
	VID = 0xF055;
	PID = 0x57C0;

	if (((uint8_t*)&endian_test)[0] == 0xBE) {
		printf("Despite their natural superiority for end users, big endian\n"
			"CPUs are not supported with this program, sorry.\n");
		return 0;
	}

	if (argc >= 2) {
		for (j = 1; j<argc; j++) {
			arglen = strlen(argv[j]);
			if ( ((argv[j][0] == '-') || (argv[j][0] == '/'))
			  && (arglen >= 2) ) {
				switch(argv[j][1]) {
				case 'D':
					debug_mode = true;
					break;
				case 'i':
					extra_info = true;
					break;
				case 'p':
					test_mode = USE_PIPEBULK;
					break;
				case 'P':
					test_mode = USE_PIPEBULK;
					disconnect_reopen = true;
					break;
				case 't':
					test_mode = USE_PIPETEXT;
					break;
				case 'T':
					test_mode = USE_PIPETEXT;
					disconnect_reopen = true;
					break;
				case 's':
					test_mode = USE_PIPESTOP;
					break;
				case 'v':
					test_mode = USE_VERSION;
					break;
				case 'I':
					test_mode = USE_SYSINFO;
					break;
				case 'C':
					test_mode = USE_CALIBRATION;
					break;
				case 'd':
					test_mode = USE_DFU;
					break;
				case 'B':
					test_mode = USE_BOOTLOADER;
					break;
				default:
					show_help = true;
					break;
				}
			}
		}
	}

	if ((show_help) || (argc == 1) || (argc > 7)) {
		printf("usage: %s [-h] [-d] [-i]\n", argv[0]);
		printf("   -h      : display usage\n");
		printf("   -D      : enable debug output\n");
		printf("   -p      : pipe binary striso data\n");
		printf("   -P      : pipe binary striso data, retry on disconnect\n");
		printf("   -t      : pipe text striso data\n");
		printf("   -T      : pipe text striso data, retry on disconnect\n");
		printf("   -s      : stop striso data stream\n");
		printf("   -v      : request firmware version\n");
		printf("   -d      : reboot to DFU firmware update mode\n");
		printf("   -B      : reboot to bootloader mode\n");
		printf("   -I      : show ChibiOS thread info\n");
		printf("   -C      : calibration mode\n");
		printf("   -i      : show usb info\n");
		return 0;
	}

	// xusb is commonly used as a debug tool, so it's convenient to have debug output during libusb_init(),
	// but since we can't call on libusb_set_debug() before libusb_init(), we use the env variable method
	old_dbg_str = getenv("LIBUSB_DEBUG");
	if (debug_mode) {
		if (putenv("LIBUSB_DEBUG=4") != 0)	// LIBUSB_LOG_LEVEL_DEBUG
			printf("Unable to set debug level");
	}

	version = libusb_get_version();
	if (extra_info)
		printf("Using libusb v%d.%d.%d.%d\n\n", version->major, version->minor, version->micro, version->nano);
	r = libusb_init(NULL);
	if (r < 0)
		return r;

	// If not set externally, and no debug option was given, use info log level
	if ((old_dbg_str == NULL) && (!debug_mode))
		libusb_set_debug(NULL, LIBUSB_LOG_LEVEL_INFO);
	if (error_lang != NULL) {
		r = libusb_setlocale(error_lang);
		if (r < 0)
			printf("Invalid or unsupported locale '%s': %s\n", error_lang, libusb_strerror((enum libusb_error)r));
	}

	if (disconnect_reopen) {
		while(true) {
			r = test_device(VID, PID);
			msleep(500);
		}
	} else {
		r = test_device(VID, PID);
	}

	libusb_exit(NULL);

	if (debug_mode) {
		snprintf(str, sizeof(str), "LIBUSB_DEBUG=%s", (old_dbg_str == NULL)?"":old_dbg_str);
		str[sizeof(str) - 1] = 0;	// Windows may not NUL terminate the string
	}

	return r;
}
