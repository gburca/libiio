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
#include "iio-lock.h"
#include "iio-private.h"

#include <errno.h>
#include <libusb-1.0/libusb.h>
#include <stdbool.h>
#include <string.h>

#define DEFAULT_TIMEOUT_MS 5000

/* Endpoint for non-streaming operations */
#define EP_OPS		1

struct iio_context_pdata {
	libusb_context *ctx;
	libusb_device_handle *hdl;

	/* Lock for non-streaming operations */
	struct iio_mutex *lock;
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

static ssize_t usb_read_value(struct iio_context_pdata *pdata)
{
	int ret;
	char buf[256], *end;
	long value;

	ret = read_data_sync(pdata, EP_OPS, buf, sizeof(buf));
	if (ret < 0)
		return (ssize_t) ret;

	value = strtol(buf, &end, 10);
	if (buf == end)
		return -EIO;

	return (ssize_t) value;
}

static ssize_t usb_exec_command(struct iio_context_pdata *pdata, char *cmd)
{
	int ret;
	char buf[256], *end;
	long value;

	ret = write_data_sync(pdata, EP_OPS, cmd, strlen(cmd));
	if (ret < 0)
		return (ssize_t) ret;

	return usb_read_value(pdata);
}

static int usb_get_version(const struct iio_context *ctx,
		unsigned int *major, unsigned int *minor, char git_tag[8])
{
	struct iio_context_pdata *pdata = ctx->pdata;
	char buf[256], *ptr = buf, *end;
	long maj, min;
	int ret;

	iio_mutex_lock(pdata->lock);

	ret = write_data_sync(pdata, EP_OPS,
			"VERSION\r\n", sizeof("VERSION\r\n") - 1);
	if (ret < 0) {
		iio_mutex_unlock(pdata->lock);
		return ret;
	}

	ret = read_data_sync(pdata, EP_OPS, buf, sizeof(buf));

	iio_mutex_unlock(pdata->lock);
	if (ret < 0)
		return ret;

	maj = strtol(ptr, &end, 10);
	if (ptr == end)
		return -EIO;

	ptr = end + 1;
	min = strtol(ptr, &end, 10);
	if (ptr == end)
		return -EIO;

	ptr = end + 1;
	if (buf + ret < ptr + 8)
		return -EIO;

	/* Strip the \n */
	ptr[buf + ret - ptr - 1] = '\0';

	if (major)
		*major = (unsigned int) maj;
	if (minor)
		*minor = (unsigned int) min;
	if (git_tag)
		strncpy(git_tag, ptr, 8);
	return 0;
}

static ssize_t usb_read_attr_helper(const struct iio_device *dev,
		const struct iio_channel *chn, const char *attr, char *dst,
		size_t len, bool is_debug)
{
	struct iio_context_pdata *pdata = dev->ctx->pdata;
	ssize_t read_len;
	ssize_t ret;
	char buf[1024];
	const char *id = dev->id;

	if (chn)
		snprintf(buf, sizeof(buf), "READ %s %s %s %s\r\n", id,
				chn->is_output ? "OUTPUT" : "INPUT",
				chn->id, attr ? attr : "");
	else if (is_debug)
		snprintf(buf, sizeof(buf), "READ %s DEBUG %s\r\n",
				id, attr ? attr : "");
	else
		snprintf(buf, sizeof(buf), "READ %s %s\r\n",
				id, attr ? attr : "");

	iio_mutex_lock(pdata->lock);

	read_len = usb_exec_command(pdata, buf);
	if (read_len < 0) {
		iio_mutex_unlock(pdata->lock);
		return read_len;
	}

	if ((size_t) read_len > len) {
		iio_mutex_unlock(pdata->lock);

		ERROR("Value returned by server is too large\n");
		return -EIO;
	}

	ret = (ssize_t) read_data_sync(pdata, EP_OPS, dst, read_len);
	iio_mutex_unlock(pdata->lock);

	if (ret < 0) {
		ERROR("Unable to read response to READ: %i\n", ret);
		return ret;
	}

	dst[ret - 1] = '\0';
	return ret;
}

static ssize_t usb_write_attr_helper(const struct iio_device *dev,
		const struct iio_channel *chn, const char *attr,
		const char *src, size_t len, bool is_debug)
{
	struct iio_context_pdata *pdata = dev->ctx->pdata;
	ssize_t read_len;
	ssize_t ret;
	char buf[1024];
	const char *id = dev->id;
	long resp;

	if (chn)
		snprintf(buf, sizeof(buf), "WRITE %s %s %s %s %lu\r\n",
				id, chn->is_output ? "OUTPUT" : "INPUT",
				chn->id, attr ? attr : "", (unsigned long) len);
	else if (is_debug)
		snprintf(buf, sizeof(buf), "WRITE %s DEBUG %s %lu\r\n",
				id, attr ? attr : "", (unsigned long) len);
	else
		snprintf(buf, sizeof(buf), "WRITE %s %s %lu\r\n",
				id, attr ? attr : "", (unsigned long) len);

	iio_mutex_lock(pdata->lock);

	ret = write_data_sync(pdata, EP_OPS, buf, strlen(buf));
	if (ret < 0)
		goto out_unlock_mutex;

	ret = write_data_sync(pdata, EP_OPS, (char *) src, len);
	if (ret < 0)
		goto out_unlock_mutex;

	ret = usb_read_value(pdata);

out_unlock_mutex:
	iio_mutex_unlock(pdata->lock);
	return ret;
}

static ssize_t usb_read_dev_attr(const struct iio_device *dev,
		const char *attr, char *dst, size_t len, bool is_debug)
{
	if (attr && ((is_debug && !iio_device_find_debug_attr(dev, attr)) ||
			(!is_debug && !iio_device_find_attr(dev, attr))))
		return -ENOENT;

	return usb_read_attr_helper(dev, NULL, attr, dst, len, is_debug);
}

static ssize_t usb_write_dev_attr(const struct iio_device *dev,
		const char *attr, const char *src, size_t len, bool is_debug)
{
	if (attr && ((is_debug && !iio_device_find_debug_attr(dev, attr)) ||
			(!is_debug && !iio_device_find_attr(dev, attr))))
		return -ENOENT;

	return usb_write_attr_helper(dev, NULL, attr, src, len, is_debug);
}

static ssize_t usb_read_chn_attr(const struct iio_channel *chn,
		const char *attr, char *dst, size_t len)
{
	if (attr && !iio_channel_find_attr(chn, attr))
		return -ENOENT;

	return usb_read_attr_helper(chn->dev, chn, attr, dst, len, false);
}

static ssize_t usb_write_chn_attr(const struct iio_channel *chn,
		const char *attr, const char *src, size_t len)
{
	if (attr && !iio_channel_find_attr(chn, attr))
		return -ENOENT;

	return usb_write_attr_helper(chn->dev, chn, attr, src, len, false);
}

static int usb_set_kernel_buffers_count(const struct iio_device *dev,
		unsigned int nb_blocks)
{
	struct iio_context_pdata *pdata = dev->ctx->pdata;
	char buf[1024];
	int ret;

	snprintf(buf, sizeof(buf), "SET %s BUFFERS_COUNT %u\r\n",
			dev->id, nb_blocks);

	iio_mutex_lock(pdata->lock);
	ret = (int) usb_exec_command(pdata, buf);
	iio_mutex_unlock(pdata->lock);

	return ret;
}

static void usb_shutdown(struct iio_context *ctx)
{
	iio_mutex_destroy(ctx->pdata->lock);

	libusb_close(ctx->pdata->hdl);
	libusb_exit(ctx->pdata->ctx);
}

static const struct iio_backend_ops usb_ops = {
	.get_version = usb_get_version,
	.read_device_attr = usb_read_dev_attr,
	.read_channel_attr = usb_read_chn_attr,
	.write_device_attr = usb_write_dev_attr,
	.write_channel_attr = usb_write_chn_attr,
	.set_kernel_buffers_count = usb_set_kernel_buffers_count,
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
	ssize_t xml_len;
	char *xml, *end;

	pdata = calloc(1, sizeof(*pdata));
	if (!pdata) {
		ERROR("Unable to allocate pdata\n");
		ret = -ENOMEM;
		goto err_set_errno;
	}

	pdata->lock = iio_mutex_create();
	if (!pdata->lock) {
		ERROR("Unable to create mutex\n");
		ret = -ENOMEM;
		goto err_free_pdata;
	}

	ret = libusb_init(&usb_ctx);
	if (ret) {
		ret = -libusb_to_errno(ret);
		ERROR("Unable to init libusb: %i\n", ret);
		goto err_destroy_mutex;
	}

	hdl = libusb_open_device_with_vid_pid(usb_ctx, vid, pid);
	if (!hdl) {
		ERROR("Unable to find device 0x%04hx:0x%04hx\n", vid, pid);
		ret = -ENODEV;
		goto err_libusb_exit;
	}

	libusb_set_auto_detach_kernel_driver(hdl, true);

	ret = libusb_claim_interface(hdl, 0);
	if (ret) {
		ret = -libusb_to_errno(ret);
		ERROR("Unable to claim interface 0: %i\n", ret);
		goto err_libusb_close;
	}

	pdata->ctx = usb_ctx;
	pdata->hdl = hdl;

	DEBUG("Sending PRINT command\n");
	xml_len = usb_exec_command(pdata, "PRINT\r\n");
	if (xml_len < 0) {
		ERROR("Unable to send print command: %i\n", ret);
		goto err_libusb_close;
	}

	xml = malloc((size_t) xml_len);
	if (!xml) {
		ERROR("Unable to allocate XML string\n");
		ret = -ENOMEM;
		goto err_libusb_close;
	}

	DEBUG("Reading XML string...\n");
	ret = read_data_sync(pdata, EP_OPS, xml, xml_len);
	if (ret < 0) {
		ERROR("Unable to read XML string: %i\n", ret);
		goto err_free_xml;
	}

	/* Discard \n character */
	read_data_sync(pdata, EP_OPS, buf, 1);

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
err_destroy_mutex:
	iio_mutex_destroy(pdata->lock);
err_free_pdata:
	free(pdata);
err_set_errno:
	errno = -ret;
	return NULL;
}
