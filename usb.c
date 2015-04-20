/*
 * libiio - Library for interfacing industrial I/O (IIO) devices
 *
 * Copyright (C) 2015 Analog Devices, Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
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
 * */

#include "debug.h"
#include "iio-private.h"

#include <errno.h>
#include <libusb-1.0/libusb.h>
#include <stdbool.h>
#include <string.h>

#define DEFAULT_TIMEOUT_MS 5000

struct iio_context_pdata {
	libusb_context *ctx;
	libusb_device_handle *hdl;
};

static const unsigned int libusb_to_errno_codes[] = {
	[- LIBUSB_ERROR_INVALID_PARAM]	= EINVAL,
	[- LIBUSB_ERROR_ACCESS]		= EACCES,
	[- LIBUSB_ERROR_NO_DEVICE]	= ENODEV,
	[- LIBUSB_ERROR_NOT_FOUND]	= ENXIO,
	[- LIBUSB_ERROR_BUSY]		= EBUSY,
	[- LIBUSB_ERROR_TIMEOUT]	= ETIMEDOUT,
	[- LIBUSB_ERROR_OVERFLOW]	= EIO,
	[- LIBUSB_ERROR_PIPE]		= EPIPE,
	[- LIBUSB_ERROR_INTERRUPTED]	= EINTR,
	[- LIBUSB_ERROR_NO_MEM]		= ENOMEM,
	[- LIBUSB_ERROR_NOT_SUPPORTED]	= ENOSYS,
};

static unsigned int libusb_to_errno(int error)
{
	switch ((enum libusb_error) error) {
	case LIBUSB_ERROR_INVALID_PARAM:
	case LIBUSB_ERROR_ACCESS:
	case LIBUSB_ERROR_NO_DEVICE:
	case LIBUSB_ERROR_NOT_FOUND:
	case LIBUSB_ERROR_BUSY:
	case LIBUSB_ERROR_TIMEOUT:
	case LIBUSB_ERROR_PIPE:
	case LIBUSB_ERROR_INTERRUPTED:
	case LIBUSB_ERROR_NO_MEM:
	case LIBUSB_ERROR_NOT_SUPPORTED:
		return libusb_to_errno_codes[- (int) error];
	case LIBUSB_ERROR_IO:
	case LIBUSB_ERROR_OTHER:
	case LIBUSB_ERROR_OVERFLOW:
	default:
		return EIO;
	}
}

static int write_data_sync(struct iio_context_pdata *pdata,
		int ep, char *data, size_t len)
{
	int transferred, ret;

	ret = libusb_bulk_transfer(pdata->hdl, ep | LIBUSB_ENDPOINT_OUT,
			data, len, &transferred, DEFAULT_TIMEOUT_MS);
	if (ret)
		return -libusb_to_errno(ret);
	else
		return transferred != len ? -EIO : len;
}

static int read_data_sync(struct iio_context_pdata *pdata,
		int ep, char *buf, size_t len)
{
	int transferred, ret;

	ret = libusb_bulk_transfer(pdata->hdl, ep | LIBUSB_ENDPOINT_IN,
			buf, len, &transferred, DEFAULT_TIMEOUT_MS);
	if (ret)
		return -libusb_to_errno(ret);
	else
		return transferred;
}

static void usb_shutdown(struct iio_context *ctx)
{
	libusb_close(ctx->pdata->hdl);
	libusb_exit(ctx->pdata->ctx);
}

static const struct iio_backend_ops usb_ops = {
	.shutdown = usb_shutdown,
};

struct iio_context * usb_create_context(unsigned short vid, unsigned short pid)
{
	libusb_context *usb_ctx;
	libusb_device_handle *hdl;
	struct iio_context *ctx;
	struct iio_context_pdata *pdata;
	int ret, transferred;
	char buf[256];
	long xml_len;
	char *xml, *end;

	pdata = calloc(1, sizeof(*pdata));
	if (!pdata) {
		ERROR("Unable to allocate pdata\n");
		ret = -ENOMEM;
		goto err_set_errno;
	}

	ret = libusb_init(&usb_ctx);
	if (ret) {
		ERROR("Unable to init libusb: %i\n", ret);
		goto err_free_pdata;
	}

	hdl = libusb_open_device_with_vid_pid(usb_ctx, vid, pid);
	if (!hdl) {
		ERROR("Unable to find device 0x%04hx:0x%04hx\n", vid, pid);
		ret = -ENODEV;
		goto err_libusb_exit;
	}

	libusb_set_auto_detach_kernel_driver(hdl, true);

	ret = libusb_claim_interface(hdl, 0);
	if (ret < 0) {
		ERROR("Unable to claim interface 0\n");
		goto err_libusb_close;
	}

	pdata->ctx = usb_ctx;
	pdata->hdl = hdl;

	DEBUG("Sending PRINT command\n");
	ret = write_data_sync(pdata, 1, "PRINT\r\n", sizeof("PRINT\r\n") - 1);
	if (ret < 0) {
		ERROR("Unable to send print command: %i\n", ret);
		goto err_libusb_close;
	}

	ret = read_data_sync(pdata, 1, buf, sizeof(buf));
	if (ret < 0) {
		ERROR("Unable to read result of print command: %i\n", ret);
		goto err_libusb_close;
	}

	xml_len = strtol(buf, &end, 10);
	if (end == buf) {
		ERROR("Print command returned unexpected result\n");
		ret = -EIO;
		goto err_libusb_close;
	}

	xml = malloc(xml_len);
	if (!xml) {
		ERROR("Unable to allocate XML string\n");
		ret = -ENOMEM;
		goto err_libusb_close;
	}

	DEBUG("Reading XML string...\n");
	ret = read_data_sync(pdata, 1, xml, xml_len);
	if (ret < 0) {
		ERROR("Unable to read XML string: %i\n", ret);
		goto err_free_xml;
	}

	/* Discard \n character */
	read_data_sync(pdata, 1, buf, 1);

	DEBUG("Creating context from XML...\n");
	ctx = iio_create_xml_context_mem(xml, xml_len);
	free(xml);
	if (!ctx)
		goto err_libusb_close;

	ctx->name = "usb";
	ctx->ops = &usb_ops;
	ctx->pdata = pdata;

	return ctx;

err_free_xml:
	free(xml);
err_libusb_close:
	libusb_close(hdl);
err_libusb_exit:
	libusb_exit(usb_ctx);
err_free_pdata:
	free(pdata);
err_set_errno:
	errno = -ret;
	return NULL;
}
