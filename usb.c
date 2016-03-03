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

#include "iio-lock.h"
#include "iio-private.h"
#include "iiod-client.h"

#include <errno.h>
#include <libusb-1.0/libusb.h>
#include <stdbool.h>
#include <string.h>

#ifdef ERROR
#undef ERROR
#endif

#include "debug.h"

#define DEFAULT_TIMEOUT_MS 5000

/* Endpoint for non-streaming operations */
#define EP_OPS		1

struct iio_usb_io_endpoint {
	unsigned char address;
	bool in_use;

	struct iio_mutex *lock;
};

struct iio_usb_io_context {
	int ep;

	bool cancellable;
	bool cancelled;
	struct libusb_transfer *transfer;
};

struct iio_context_pdata {
	libusb_context *ctx;
	libusb_device_handle *hdl;

	struct iiod_client *iiod_client;

	/* Lock for non-streaming operations */
	struct iio_mutex *lock;

	/* Lock for endpoint reservation */
	struct iio_mutex *ep_lock;

	struct iio_usb_io_endpoint *io_endpoints;
	unsigned int nb_io_endpoints;

	unsigned int timeout_ms;

	struct iio_usb_io_context io_ctx;
};

struct iio_device_pdata {
	bool is_tx;
	struct iio_mutex *lock;

	bool opened;
	struct iio_usb_io_context io_ctx;
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

static int usb_get_version(const struct iio_context *ctx,
		unsigned int *major, unsigned int *minor, char git_tag[8])
{
	return iiod_client_get_version(ctx->pdata->iiod_client,
			(uintptr_t) &ctx->pdata->io_ctx, major, minor, git_tag);
}

static unsigned int usb_calculate_remote_timeout(unsigned int timeout)
{
	/* XXX(pcercuei): We currently hardcode timeout / 2 for the backend used
	 * by the remote. Is there something better to do here? */
	return timeout / 2;
}

static int usb_reset_pipes(libusb_device_handle *hdl)
{
	return libusb_control_transfer(hdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_RECIPIENT_INTERFACE, 0, 0, 0, NULL, 0, 0);
}

static int usb_open_pipe(libusb_device_handle *hdl, unsigned int ep)
{
	return libusb_control_transfer(hdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_RECIPIENT_INTERFACE, 1, ep - 1, 0, NULL, 0, 0);
}

static int usb_close_pipe(libusb_device_handle *hdl, unsigned int ep)
{
	return libusb_control_transfer(hdl, LIBUSB_REQUEST_TYPE_VENDOR |
		LIBUSB_RECIPIENT_INTERFACE, 2, ep - 1, 0, NULL, 0, 0);
}

static int usb_reserve_ep_unlocked(const struct iio_device *dev)
{
	struct iio_context_pdata *pdata = dev->ctx->pdata;
	unsigned int i;

	for (i = 0; i < pdata->nb_io_endpoints; i++) {
		struct iio_usb_io_endpoint *ep = &pdata->io_endpoints[i];

		if (!ep->in_use) {
			ep->in_use = true;

			dev->pdata->io_ctx.ep = ep->address;
			dev->pdata->lock = ep->lock;
			return 0;
		}
	}

	return -EBUSY;
}

static void usb_free_ep_unlocked(const struct iio_device *dev)
{
	struct iio_context_pdata *pdata = dev->ctx->pdata;
	unsigned int i;

	for (i = 0; i < pdata->nb_io_endpoints; i++) {
		struct iio_usb_io_endpoint *ep = &pdata->io_endpoints[i];

		if (ep->lock == dev->pdata->lock) {
			ep->in_use = false;
			return;
		}
	}
}

static int usb_open(const struct iio_device *dev,
		size_t samples_count, bool cyclic)
{
	struct iio_context_pdata *ctx_pdata = dev->ctx->pdata;
	struct iio_device_pdata *pdata = dev->pdata;
	int ret = -EBUSY;

	iio_mutex_lock(ctx_pdata->ep_lock);

	if (pdata->opened)
		goto out_unlock;

	ret = usb_reserve_ep_unlocked(dev);
	if (ret)
		goto out_unlock;

	usb_open_pipe(ctx_pdata->hdl, pdata->io_ctx.ep);

	iio_mutex_lock(pdata->lock);

	ret = iiod_client_open_unlocked(ctx_pdata->iiod_client,
			(uintptr_t) &pdata->io_ctx, dev, samples_count, cyclic);

	if (!ret) {
		unsigned int remote_timeout =
			usb_calculate_remote_timeout(ctx_pdata->timeout_ms);

		ret = iiod_client_set_timeout(ctx_pdata->iiod_client,
				(uintptr_t) &pdata->io_ctx, remote_timeout);
	}

	pdata->opened = !ret;

	iio_mutex_unlock(pdata->lock);

	if (ret)
		usb_free_ep_unlocked(dev);

out_unlock:
	iio_mutex_unlock(ctx_pdata->ep_lock);
	return ret;
}

static int usb_close(const struct iio_device *dev)
{
	struct iio_context_pdata *ctx_pdata = dev->ctx->pdata;
	struct iio_device_pdata *pdata = dev->pdata;
	int ret = -EBADF;

	iio_mutex_lock(ctx_pdata->ep_lock);
	if (!pdata->opened)
		goto out_unlock;

	iio_mutex_lock(pdata->lock);
	ret = iiod_client_close_unlocked(ctx_pdata->iiod_client,
			(uintptr_t) &pdata->io_ctx, dev);
	pdata->opened = false;

	iio_mutex_unlock(pdata->lock);

	usb_close_pipe(ctx_pdata->hdl, pdata->io_ctx.ep);

	usb_free_ep_unlocked(dev);

out_unlock:
	iio_mutex_unlock(ctx_pdata->ep_lock);
	return ret;
}

static ssize_t usb_read(const struct iio_device *dev, void *dst, size_t len,
		uint32_t *mask, size_t words)
{
	struct iio_device_pdata *pdata = dev->pdata;
	ssize_t ret;

	iio_mutex_lock(pdata->lock);
	ret = iiod_client_read_unlocked(dev->ctx->pdata->iiod_client,
			(uintptr_t) &pdata->io_ctx, dev, dst, len, mask, words);
	iio_mutex_unlock(pdata->lock);

	return ret;
}

static ssize_t usb_write(const struct iio_device *dev,
		const void *src, size_t len)
{
	struct iio_device_pdata *pdata = dev->pdata;
	ssize_t ret;

	iio_mutex_lock(pdata->lock);
	ret = iiod_client_write_unlocked(dev->ctx->pdata->iiod_client,
			(uintptr_t) &pdata->io_ctx, dev, src, len);
	iio_mutex_unlock(pdata->lock);

	return ret;
}

static ssize_t usb_read_dev_attr(const struct iio_device *dev,
		const char *attr, char *dst, size_t len, bool is_debug)
{
	struct iio_context_pdata *pdata = dev->ctx->pdata;

	return iiod_client_read_attr(pdata->iiod_client,
			(uintptr_t) &pdata->io_ctx, dev, NULL, attr,
			dst, len, is_debug);
}

static ssize_t usb_write_dev_attr(const struct iio_device *dev,
		const char *attr, const char *src, size_t len, bool is_debug)
{
	struct iio_context_pdata *pdata = dev->ctx->pdata;

	return iiod_client_write_attr(pdata->iiod_client,
			(uintptr_t) &pdata->io_ctx, dev, NULL, attr,
			src, len, is_debug);
}

static ssize_t usb_read_chn_attr(const struct iio_channel *chn,
		const char *attr, char *dst, size_t len)
{
	struct iio_context_pdata *pdata = chn->dev->ctx->pdata;

	return iiod_client_read_attr(pdata->iiod_client,
			(uintptr_t) &pdata->io_ctx, chn->dev, chn, attr,
			dst, len, false);
}

static ssize_t usb_write_chn_attr(const struct iio_channel *chn,
		const char *attr, const char *src, size_t len)
{
	struct iio_context_pdata *pdata = chn->dev->ctx->pdata;

	return iiod_client_write_attr(pdata->iiod_client,
			(uintptr_t) &pdata->io_ctx, chn->dev, chn, attr,
			src, len, false);
}

static int usb_set_kernel_buffers_count(const struct iio_device *dev,
		unsigned int nb_blocks)
{
	struct iio_context_pdata *pdata = dev->ctx->pdata;

	return iiod_client_set_kernel_buffers_count(pdata->iiod_client,
			 (uintptr_t) &pdata->io_ctx, dev, nb_blocks);
}

static int usb_set_timeout(struct iio_context *ctx, unsigned int timeout)
{
	struct iio_context_pdata *pdata = ctx->pdata;
	unsigned int remote_timeout = usb_calculate_remote_timeout(timeout);
	int ret;

	ret = iiod_client_set_timeout(pdata->iiod_client,
			 (uintptr_t) &pdata->io_ctx, remote_timeout);
	if (!ret)
		pdata->timeout_ms = timeout;

	return ret;
}

static void usb_shutdown(struct iio_context *ctx)
{
	unsigned int i;

	iio_mutex_destroy(ctx->pdata->lock);
	iio_mutex_destroy(ctx->pdata->ep_lock);

	for (i = 0; i < ctx->pdata->nb_io_endpoints; i++)
		if (ctx->pdata->io_endpoints[i].lock)
			iio_mutex_destroy(ctx->pdata->io_endpoints[i].lock);
	if (ctx->pdata->io_endpoints)
		free(ctx->pdata->io_endpoints);

	for (i = 0; i < ctx->nb_devices; i++) {
		struct iio_device *dev = ctx->devices[i];

		free(dev->pdata);
	}

	iiod_client_destroy(ctx->pdata->iiod_client);

	usb_reset_pipes(ctx->pdata->hdl); /* Close everything */

	libusb_close(ctx->pdata->hdl);
	libusb_exit(ctx->pdata->ctx);
	free(ctx->pdata);
}

static void usb_context_info_free(struct iio_context_info *info)
{
	free(info->description);
	free(info->uri);
}

#define USB_URI_MAX_LEN (sizeof("usb:127.255") + 1)

static int usb_fill_context_info(struct iio_context_info *info,
	libusb_device *dev)
{
	struct libusb_device_descriptor desc;
	char manufacturer[64], product[64];
	struct libusb_device_handle *hdl;
	char *description;
	size_t description_len;
	char *uri;
	int ret;

	libusb_get_device_descriptor(dev, &desc);

	uri = malloc(USB_URI_MAX_LEN);
	if (!uri)
		return -ENOMEM;

	ret = snprintf(uri, USB_URI_MAX_LEN, "usb:%d.%d",
		libusb_get_bus_number(dev), libusb_get_device_address(dev));
	if (ret < 0) {
		free(uri);
		return -ENOMEM;
	}

	description_len = sizeof("USB  ID 0000:0000") + strlen(uri) + 1;

	ret = libusb_open(dev, &hdl);
	if (ret == 0) {
		if (desc.iManufacturer == 0) {
			manufacturer[0] = '\0';
		} else {
			libusb_get_string_descriptor_ascii(hdl,
				desc.iManufacturer,
				(unsigned char *) manufacturer,
				sizeof(manufacturer));
		}

		if (desc.iProduct == 0) {
			product[0] = '\0';
		} else {
			libusb_get_string_descriptor_ascii(hdl,
				desc.iProduct, (unsigned char *) product,
				sizeof(product));
		}

		libusb_close(hdl);

		description_len += strlen(product) + strlen(manufacturer) + 2;
		description = malloc(description_len);
		if (!description) {
			free(uri);
		    return -ENOMEM;
		}

		ret = snprintf(description, description_len,
			"USB %s ID %.4x:%.4x %s %s",
			uri+4, desc.idVendor, desc.idProduct, manufacturer,
			product);
	} else {
		description = malloc(description_len);
		if (!description) {
			free(uri);
		    return -ENOMEM;
		}

		ret = snprintf(description, description_len, "USB %s ID %.4x:%.4x",
			uri+4, desc.idVendor, desc.idProduct);
	}

	if (ret < 0) {
		free(description);
		free(uri);
		return -ENOMEM;
	}

	info->uri = uri;
	info->description = description;
	info->free = usb_context_info_free;

	return 0;
}

struct iio_usb_vid_pid {
	unsigned short vid;
	unsigned short pid;
};

/* List of known VID/PID combinations that support the IIO interface */
static const struct iio_usb_vid_pid iio_usb_vid_pids[] = {
	{ 0x0456, 0xb672 },
	{ 0, 0 }
};

static bool iio_usb_match_device(libusb_device *dev)
{
	const struct iio_usb_vid_pid *vid_pid;
	struct libusb_device_descriptor desc;

	libusb_get_device_descriptor(dev, &desc);

	for (vid_pid = iio_usb_vid_pids; vid_pid->vid != 0; vid_pid++) {
		if (desc.idVendor == vid_pid->vid &&
		    desc.idProduct == vid_pid->pid)
			return true;
	}

	return false;
}

int usb_context_scan(struct iio_scan_result *scan_result)
{
	struct iio_context_info **info;
	libusb_device **device_list;
	unsigned int num_devices;
	libusb_context *usb_ctx;
	unsigned int i;
	int ret;

	ret = libusb_init(&usb_ctx);
	if (ret) {
		DEBUG("Unable to init libusb: %d\n", ret);
		return 0; /* No USB support found on the system. */
	}

	ret = libusb_get_device_list(usb_ctx, &device_list);
	if (ret < 0) {
		ret = -(int) libusb_to_errno(ret);
		goto cleanup_exit;
	}

	num_devices = 0;
	for (i = 0; device_list[i]; i++) {
		if (iio_usb_match_device(device_list[i]))
			num_devices++;
	}

	info = iio_scan_result_add(scan_result, num_devices);
	if (!info) {
		ret = -ENOMEM;
		goto cleanup_free_device_list;
	}

	for (i = 0; device_list[i]; i++) {
		if (iio_usb_match_device(device_list[i])) {
			ret = usb_fill_context_info(*info, device_list[i]);
			if (ret)
				goto cleanup_free_device_list;
			info++;
		}
	}
	ret = 0;

cleanup_free_device_list:
	libusb_free_device_list(device_list, true);
cleanup_exit:
	libusb_exit(usb_ctx);

	return ret;
}

static int usb_set_cancellable(const struct iio_device *dev,
	bool cancellable)
{
	struct iio_device_pdata *ppdata = dev->pdata;

	if (ppdata->io_ctx.cancellable == cancellable)
		return true;

	ppdata->io_ctx.cancellable = cancellable;

	return 0;
}

static void usb_cancel(const struct iio_device *dev)
{
	struct iio_device_pdata *ppdata = dev->pdata;

	if (ppdata->io_ctx.cancellable && ppdata->io_ctx.transfer)
		libusb_cancel_transfer(ppdata->io_ctx.transfer);

	ppdata->io_ctx.cancelled = true;
}

static const struct iio_backend_ops usb_ops = {
	.get_version = usb_get_version,
	.open = usb_open,
	.close = usb_close,
	.read = usb_read,
	.write = usb_write,
	.read_device_attr = usb_read_dev_attr,
	.read_channel_attr = usb_read_chn_attr,
	.write_device_attr = usb_write_dev_attr,
	.write_channel_attr = usb_write_chn_attr,
	.set_kernel_buffers_count = usb_set_kernel_buffers_count,
	.set_timeout = usb_set_timeout,
	.shutdown = usb_shutdown,

	.set_cancellable = usb_set_cancellable,
	.cancel = usb_cancel,
};

static void LIBUSB_CALL sync_transfer_cb(struct libusb_transfer *transfer)
{
	int *completed = transfer->user_data;
	*completed = 1;
}

static int transfer_sync(struct iio_context_pdata *pdata,
	struct iio_usb_io_context *io_ctx, unsigned int ep_type,
	char *data, size_t len, int *transferred)
{
	struct libusb_transfer *transfer;
	int completed = 0;
	int ret;

	if (io_ctx->cancelled)
		return -EBADF;

	transfer = libusb_alloc_transfer(0);
	if (!transfer)
		return -ENOMEM;

	transfer->user_data = &completed;

	libusb_fill_bulk_transfer(transfer, pdata->hdl, io_ctx->ep | ep_type,
		(unsigned char *) data, (int) len,
		sync_transfer_cb, &completed, pdata->timeout_ms);
	transfer->type = LIBUSB_TRANSFER_TYPE_BULK;

	ret = libusb_submit_transfer(transfer);
	if (ret < 0) {
		libusb_free_transfer(transfer);
		return ret;
	}

	io_ctx->transfer = transfer;

	while (!completed) {
		ret = libusb_handle_events_completed(pdata->ctx, &completed);
		if (ret < 0) {
			if (ret == LIBUSB_ERROR_INTERRUPTED)
				continue;
			libusb_cancel_transfer(transfer);
			continue;
		}
	}

	*transferred = transfer->actual_length;

	io_ctx->transfer = NULL;

	libusb_free_transfer(transfer);

	return 0;
}

static ssize_t write_data_sync(struct iio_context_pdata *pdata,
		uintptr_t ep, const char *data, size_t len)
{
	int transferred, ret;

	ret = transfer_sync(pdata, (void *)ep,
		LIBUSB_ENDPOINT_OUT, (char *)data, len, &transferred);
	if (ret)
		return -(int) libusb_to_errno(ret);
	else
		return (size_t) transferred != len ? -EIO : (ssize_t) len;
}

static ssize_t read_data_sync(struct iio_context_pdata *pdata,
		uintptr_t ep, char *data, size_t len)
{
	int transferred, ret;

	ret = transfer_sync(pdata, (void *)ep,
		LIBUSB_ENDPOINT_IN, data, len, &transferred);;
	if (ret)
		return -(int) libusb_to_errno(ret);
	else
		return transferred;
}

static const struct iiod_client_ops usb_iiod_client_ops = {
	.write = write_data_sync,
	.read = read_data_sync,
	.read_line = read_data_sync,
};

static int usb_count_io_eps(const struct libusb_interface_descriptor *iface)
{
	unsigned int eps = iface->bNumEndpoints;
	unsigned int i, curr;

	/* Check that for a number of endpoints X provided by the interface, we
	 * have the input and output endpoints in the address range [1, ... X/2]
	 * and that each input endpoint has a corresponding output endpoint at
	 * the same address. */

	if (eps < 2 || eps % 2)
		return -EINVAL;

	for (curr = 1; curr < (eps / 2) + 1; curr++) {
		bool found_in = false, found_out = false;

		for (i = 0; !found_in && i < eps; i++)
			found_in = iface->endpoint[i].bEndpointAddress ==
				(LIBUSB_ENDPOINT_IN | curr);
		if (!found_in)
			return -EINVAL;

		for (i = 0; !found_out && i < eps; i++)
			found_out = iface->endpoint[i].bEndpointAddress ==
				(LIBUSB_ENDPOINT_OUT | curr);
		if (!found_out)
			return -EINVAL;
	}

	/* -1: we reserve the first I/O endpoint couple for global operations */
	return (int) curr - 1;
}

struct iio_context * usb_create_context(unsigned int bus,
	unsigned int address)
{
	libusb_context *usb_ctx;
	libusb_device_handle *hdl;
	const struct libusb_interface_descriptor *iface;
	libusb_device *dev, *usb_dev;
	struct libusb_config_descriptor *conf_desc;
	libusb_device **device_list;
	struct iio_context *ctx;
	struct iio_context_pdata *pdata;
	unsigned int i;
	int ret;

	pdata = zalloc(sizeof(*pdata));
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

	pdata->ep_lock = iio_mutex_create();
	if (!pdata->ep_lock) {
		ERROR("Unable to create mutex\n");
		ret = -ENOMEM;
		goto err_destroy_mutex;
	}

	pdata->iiod_client = iiod_client_new(pdata, pdata->lock,
			&usb_iiod_client_ops);
	if (!pdata->iiod_client) {
		ERROR("Unable to create IIOD client\n");
		ret = -errno;
		goto err_destroy_ep_mutex;
	}

	ret = libusb_init(&usb_ctx);
	if (ret) {
		ret = -(int) libusb_to_errno(ret);
		ERROR("Unable to init libusb: %i\n", ret);
		goto err_destroy_iiod_client;
	}

	libusb_get_device_list(usb_ctx, &device_list);

	usb_dev = NULL;

	for (i = 0; device_list[i]; i++) {
		dev = device_list[i];

		if (bus == libusb_get_bus_number(dev) &&
			address == libusb_get_device_address(dev)) {
			usb_dev = dev;
			libusb_ref_device(usb_dev);
			break;
		}
	}

	libusb_free_device_list(device_list, true);

	if (!usb_dev)
		goto err_libusb_exit;

	ret = libusb_open(usb_dev, &hdl);
	libusb_unref_device(usb_dev); /* Open gets us a extra ref */
	if (ret) {
		ret = -(int) libusb_to_errno(ret);
		ERROR("Unable to open device\n");
		goto err_libusb_exit;
	}

	libusb_set_auto_detach_kernel_driver(hdl, true);

	ret = libusb_claim_interface(hdl, 0);
	if (ret) {
		ret = -(int) libusb_to_errno(ret);
		ERROR("Unable to claim interface 0: %i\n", ret);
		goto err_libusb_close;
	}

	ret = libusb_get_active_config_descriptor(usb_dev, &conf_desc);
	if (ret) {
		ret = -(int) libusb_to_errno(ret);
		ERROR("Unable to get config descriptor: %i\n", ret);
		goto err_libusb_close;
	}

	iface = &conf_desc->interface[0].altsetting[0];

	ret = usb_count_io_eps(iface);
	if (ret < 0) {
		ERROR("Invalid configuration of endpoints\n");
		goto err_free_config_descriptor;
	}

	pdata->nb_io_endpoints = ret;

	DEBUG("Found %hhu usable i/o endpoints\n", pdata->nb_io_endpoints);

	if (pdata->nb_io_endpoints) {
		pdata->io_endpoints = calloc(pdata->nb_io_endpoints,
				sizeof(*pdata->io_endpoints));
		if (!pdata->io_endpoints) {
			ERROR("Unable to allocate endpoints\n");
			ret = -ENOMEM;
			goto err_free_config_descriptor;
		}

		for (i = 0; i < pdata->nb_io_endpoints; i++) {
			struct iio_usb_io_endpoint *ep =
				&pdata->io_endpoints[i];

			/* +2: endpoints start at number 1, and we skip the
			 * endpoint #1 that we reserve for global operations */
			ep->address = i + 2;

			ep->lock = iio_mutex_create();
			if (!ep->lock) {
				ERROR("Unable to create mutex\n");
				ret = -ENOMEM;
				goto err_free_endpoints;
			}
		}
	}

	pdata->ctx = usb_ctx;
	pdata->hdl = hdl;
	pdata->timeout_ms = DEFAULT_TIMEOUT_MS;
	pdata->io_ctx.ep = EP_OPS;

	usb_reset_pipes(hdl);
	usb_open_pipe(hdl, EP_OPS);

	ctx = iiod_client_create_context(pdata->iiod_client,
			(uintptr_t)&pdata->io_ctx);
	if (!ctx)
		goto err_free_endpoints;

	libusb_free_config_descriptor(conf_desc);

	ctx->name = "usb";
	ctx->ops = &usb_ops;
	ctx->pdata = pdata;

	DEBUG("Initializing context...\n");
	ret = iio_context_init(ctx);
	if (ret < 0)
		goto err_context_destroy;

	for (i = 0; i < ctx->nb_devices; i++) {
		struct iio_device *dev = ctx->devices[i];

		dev->pdata = zalloc(sizeof(*dev->pdata));
		if (!dev->pdata) {
			ERROR("Unable to allocate memory\n");
			ret = -ENOMEM;
			goto err_context_destroy;
		}

		dev->pdata->is_tx = iio_device_is_tx(dev);
	}

	return ctx;

err_context_destroy:
	iio_context_destroy(ctx);
	errno = -ret;
	return NULL;

err_free_endpoints:
	for (i = 0; i < pdata->nb_io_endpoints; i++)
		if (pdata->io_endpoints[i].lock)
			iio_mutex_destroy(pdata->io_endpoints[i].lock);
	if (pdata->io_endpoints)
		free(pdata->io_endpoints);
err_free_config_descriptor:
	libusb_free_config_descriptor(conf_desc);
err_libusb_close:
	usb_reset_pipes(hdl); /* Close everything */
	libusb_close(hdl);
err_libusb_exit:
	libusb_exit(usb_ctx);
err_destroy_iiod_client:
	iiod_client_destroy(pdata->iiod_client);
err_destroy_ep_mutex:
	iio_mutex_destroy(pdata->ep_lock);
err_destroy_mutex:
	iio_mutex_destroy(pdata->lock);
err_free_pdata:
	free(pdata);
err_set_errno:
	errno = -ret;
	return NULL;
}

struct iio_context * usb_create_context_from_uri(const char *uri)
{
	unsigned int bus, address;

	if (strncmp(uri, "usb:", sizeof("usb:") - 1) != 0)
		return NULL;

	sscanf(uri+4, "%u.%u", &bus, &address);

	return usb_create_context(bus, address);
}
