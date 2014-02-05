/*
 * spidev_module.c - Python bindings for Linux SPI access through spidev
 * Copyright (C) 2009 Volker Thoms <unconnected@gmx.de>
 * Copyright (C) 2012 Stephen Caudle <scaudle@doceme.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <Python.h>
#include "structmember.h"
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <sys/ioctl.h>

#define SPIDEV_MAXPATH 4096

PyDoc_STRVAR(SpiDev_module_doc,
	"This module defines an object type that allows SPI transactions\n"
	"on hosts running the Linux kernel. The host kernel must have SPI\n"
	"support and SPI device interface support.\n"
	"All of these can be either built-in to the kernel, or loaded from\n"
	"modules.\n"
	"\n"
	"Because the SPI device interface is opened R/W, users of this\n"
	"module usually must have root permissions.\n");


typedef struct {
	PyObject_HEAD

	int fd;	/* open file descriptor: /dev/spi-X.Y */
	uint8_t mode;	/* current SPI mode */
	uint8_t bits_per_word;	/* current SPI bits per word setting */
	uint32_t max_speed_hz;	/* current SPI max speed setting in Hz */
} SpiDevObject;

static PyObject *
SpiDev_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
	SpiDevObject *self;
	if ((self = (SpiDevObject *)type->tp_alloc(type, 0)) == NULL)
		return NULL;

	self->fd = -1;
	self->mode = 0;
	self->bits_per_word = 0;
	self->max_speed_hz = 0;

	Py_INCREF(self);
	return (PyObject *)self;
}

static const unsigned char BitReverseTable256[] = 
{
	  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 
	  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
	  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
	  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
	  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 
	  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
	  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 
	  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
	  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
	  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 
	  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
	  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
	  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 
	  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
	  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 
	  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};


static void 
SpiDev_reverseBufferBits(uint8_t *buf, int len)
{
	int i;
	for(i = 0; i < len; i++)
	{
		*buf = BitReverseTable256[*buf];
		*buf++;
	}
}

PyDoc_STRVAR(SpiDev_close_doc,
	"close()\n\n"
	"Disconnects the object from the interface.\n");

static PyObject *
SpiDev_close(SpiDevObject *self)
{
	if ((self->fd != -1) && (close(self->fd) == -1)) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}

	self->fd = -1;
	self->mode = 0;
	self->bits_per_word = 0;
	self->max_speed_hz = 0;

	Py_INCREF(Py_None);
	return Py_None;
}

static void
SpiDev_dealloc(SpiDevObject *self)
{
	PyObject *ref = SpiDev_close(self);
	Py_XDECREF(ref);

	self->ob_type->tp_free((PyObject *)self);
}

static char *wrmsg = "Argument must be a list of at least one, "
				"but not more than 4096 integers";

PyDoc_STRVAR(SpiDev_write_doc,
	"write([values]) -> None\n\n"
	"Write bytes to SPI device.\n");

static PyObject *
SpiDev_writebytes(SpiDevObject *self, PyObject *args)
{
	int		status;
	uint16_t	ii, len;
	uint8_t	buf[SPIDEV_MAXPATH];
	PyObject	*list;

	if (!PyArg_ParseTuple(args, "O:write", &list))
		return NULL;

	if (!PyList_Check(list)) {
		PyErr_SetString(PyExc_TypeError, wrmsg);
		return NULL;
	}

	if ((len = PyList_GET_SIZE(list)) > SPIDEV_MAXPATH) {
		PyErr_SetString(PyExc_OverflowError, wrmsg);
		return NULL;
	}

	for (ii = 0; ii < len; ii++) {
		PyObject *val = PyList_GET_ITEM(list, ii);
		if (!PyInt_Check(val)) {
			PyErr_SetString(PyExc_TypeError, wrmsg);
			return NULL;
		}
		buf[ii] = (__u8)PyInt_AS_LONG(val);
	}

	status = write(self->fd, &buf[0], len);

	if (status < 0) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}

	if (status != len) {
		perror("short write");
		return NULL;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

PyDoc_STRVAR(SpiDev_writebuff_doc,
	"write(object) -> None\n\n"
	"Write bytes to SPI device.\n");

static PyObject *
SpiDev_writeobject(SpiDevObject *self, PyObject *args)
{
	int		status;
	PyObject	*obj;
	Py_buffer	view;
	uint8_t         buf2[1024];
	static char *msg_no_buf = "Object does not support buffer protocol";
	static char *msg_size = "Buffer is greater than 4096 bytes";
	static char *msg_no_simple = "Object cannot return a PyBUF_SIMPLE "
					"buffer";

	if (!PyArg_ParseTuple(args, "O:object", &obj))
		return NULL;

	if (!PyObject_CheckBuffer(obj)) {
		return NULL;
	}

	if (PyObject_GetBuffer(obj, &view, PyBUF_SIMPLE)) {
		return NULL;
	}

	if (view.len > SPIDEV_MAXPATH) {
		PyErr_SetString(PyExc_OverflowError, msg_size);
		return NULL;
	}

	memcpy ((void *)buf2, view.buf, view.len);
	SpiDev_reverseBufferBits(buf2, view.len);

	status = write(self->fd, buf2, view.len);

	if (status < 0) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}

	if (status != view.len) {
		perror("short write");
		return NULL;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

PyDoc_STRVAR(SpiDev_read_doc,
	"read(len) -> [values]\n\n"
	"Read len bytes from SPI device.\n");

static PyObject *
SpiDev_readbytes(SpiDevObject *self, PyObject *args)
{
	uint8_t	rxbuf[SPIDEV_MAXPATH];
	int		status, len, ii;
	PyObject	*list;

	if (!PyArg_ParseTuple(args, "i:read", &len))
		return NULL;

	/* read at least 1 byte, no more than SPIDEV_MAXPATH */
	if (len < 1)
		len = 1;
	else if (len > sizeof(rxbuf))
		len = sizeof(rxbuf);

	memset(rxbuf, 0, sizeof rxbuf);
	status = read(self->fd, &rxbuf[0], len);

	if (status < 0) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}

	if (status != len) {
		perror("short read");
		return NULL;
	}

	list = PyList_New(len);

	for (ii = 0; ii < len; ii++) {
		PyObject *val = Py_BuildValue("l", (long)rxbuf[ii]);
		PyList_SET_ITEM(list, ii, val);
	}

	Py_INCREF(list);
	return list;
}

PyDoc_STRVAR(SpiDev_xfer_doc,
	"xfer([values]) -> [values]\n\n"
	"Perform SPI transaction.\n"
	"CS will be released and reactivated between blocks.\n"
	"delay specifies delay in usec between blocks.\n");

static PyObject *
SpiDev_xfer(SpiDevObject *self, PyObject *args)
{
	uint16_t ii, len;
	int status;
	uint16_t delay_usecs = 0;
	uint32_t speed_hz = 0;
	uint8_t bits_per_word = 0;
	PyObject *list;
#ifdef SPIDEV_SINGLE
	struct spi_ioc_transfer *xferptr;
#else
	struct spi_ioc_transfer xfer;
#endif
	uint8_t *txbuf, *rxbuf;

	if (!PyArg_ParseTuple(args, "O|IHB:xfer", &list, &speed_hz, &delay_usecs, &bits_per_word))
		return NULL;

	if (!PyList_Check(list)) {
		PyErr_SetString(PyExc_TypeError, wrmsg);
		return NULL;
	}

	if ((len = PyList_GET_SIZE(list)) > SPIDEV_MAXPATH) {
		PyErr_SetString(PyExc_OverflowError, wrmsg);
		return NULL;
	}

	txbuf = malloc(sizeof(__u8) * len);
	rxbuf = malloc(sizeof(__u8) * len);

#ifdef SPIDEV_SINGLE
	xferptr = (struct spi_ioc_transfer*) malloc(sizeof(struct spi_ioc_transfer) * len);

	for (ii = 0; ii < len; ii++) {
		PyObject *val = PyList_GET_ITEM(list, ii);
		if (!PyInt_Check(val)) {
			PyErr_SetString(PyExc_TypeError, wrmsg);
			return NULL;
		}
		txbuf[ii] = (__u8)PyInt_AS_LONG(val);
		xferptr[ii].tx_buf = (unsigned long)&txbuf[ii];
		xferptr[ii].rx_buf = (unsigned long)&rxbuf[ii];
		xferptr[ii].len = 1;
		xferptr[ii].delay_usecs = delay;
		xferptr[ii].speed_hz = speed_hz ? speed_hz : self->max_speed_hz;
		xferptr[ii].bits_per_word = bits_per_word ? bits_per_word : self->bits_per_word;
	}

	status = ioctl(self->fd, SPI_IOC_MESSAGE(len), xferptr);
	if (status < 0) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
#else
	for (ii = 0; ii < len; ii++) {
		PyObject *val = PyList_GET_ITEM(list, ii);
		if (!PyInt_Check(val)) {
			PyErr_SetString(PyExc_TypeError, wrmsg);
			return NULL;
		}
		txbuf[ii] = (__u8)PyInt_AS_LONG(val);
	}

	xfer.tx_buf = (unsigned long)txbuf;
	xfer.rx_buf = (unsigned long)rxbuf;
	xfer.len = len;
	xfer.delay_usecs = delay_usecs;
	xfer.speed_hz = speed_hz ? speed_hz : self->max_speed_hz;
	xfer.bits_per_word = bits_per_word ? bits_per_word : self->bits_per_word;

	status = ioctl(self->fd, SPI_IOC_MESSAGE(1), &xfer);
	if (status < 0) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
#endif

	for (ii = 0; ii < len; ii++) {
		PyObject *val = Py_BuildValue("l", (long)rxbuf[ii]);
		PyList_SET_ITEM(list, ii, val);
	}

	// WA:
	// in CS_HIGH mode CS isn't pulled to low after transfer, but after read
	// reading 0 bytes doesnt matter but brings cs down
	status = read(self->fd, &rxbuf[0], 0);

	free(txbuf);
	free(rxbuf);

	Py_INCREF(list);
	return list;
}


PyDoc_STRVAR(SpiDev_xfer2_doc,
	"xfer2([values]) -> [values]\n\n"
	"Perform SPI transaction.\n"
	"CS will be held active between blocks.\n");

static PyObject *
SpiDev_xfer2(SpiDevObject *self, PyObject *args)
{
	static char *msg = "Argument must be a list of at least one, "
				"but not more than 4096 integers";
	int status;
	uint16_t delay_usecs = 0;
	uint32_t speed_hz = 0;
	uint8_t bits_per_word = 0;
	uint16_t ii, len;
	PyObject *list;
	struct spi_ioc_transfer xfer;
	uint8_t *txbuf, *rxbuf;

	if (!PyArg_ParseTuple(args, "O|IHB:xfer2", &list, &speed_hz, &delay_usecs, &bits_per_word))
		return NULL;

	if (!PyList_Check(list)) {
		PyErr_SetString(PyExc_TypeError, wrmsg);
		return NULL;
	}

	if ((len = PyList_GET_SIZE(list)) > SPIDEV_MAXPATH) {
		PyErr_SetString(PyExc_OverflowError, wrmsg);
		return NULL;
	}

	txbuf = malloc(sizeof(__u8) * len);
	rxbuf = malloc(sizeof(__u8) * len);

	for (ii = 0; ii < len; ii++) {
		PyObject *val = PyList_GET_ITEM(list, ii);
		if (!PyInt_Check(val)) {
			PyErr_SetString(PyExc_TypeError, msg);
			return NULL;
		}
		txbuf[ii] = (__u8)PyInt_AS_LONG(val);
	}

	xfer.tx_buf = (unsigned long)txbuf;
	xfer.rx_buf = (unsigned long)rxbuf;
	xfer.len = len;
	xfer.delay_usecs = delay_usecs;
	xfer.speed_hz = speed_hz ? speed_hz : self->max_speed_hz;
	xfer.bits_per_word = bits_per_word ? bits_per_word : self->bits_per_word;

	status = ioctl(self->fd, SPI_IOC_MESSAGE(1), &xfer);
	if (status < 0) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}

	for (ii = 0; ii < len; ii++) {
		PyObject *val = Py_BuildValue("l", (long)rxbuf[ii]);
		PyList_SET_ITEM(list, ii, val);
	}
	// WA:
	// in CS_HIGH mode CS isnt pulled to low after transfer
	// reading 0 bytes doesn't really matter but brings CS down
	status = read(self->fd, &rxbuf[0], 0);

	free(txbuf);
	free(rxbuf);

	Py_INCREF(list);
	return list;
}

static int __spidev_set_mode( int fd, __u8 mode) {
	__u8 test;
	if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return -1;
	}
	if (ioctl(fd, SPI_IOC_RD_MODE, &test) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return -1;
	}
	if (test != mode) {
		return -1;
	}
	return 0;
}

static PyObject *
SpiDev_get_mode(SpiDevObject *self, void *closure)
{
	PyObject *result = Py_BuildValue("i", (self->mode & (SPI_CPHA | SPI_CPOL) ) );
	Py_INCREF(result);
	return result;
}

static PyObject *
SpiDev_get_cshigh(SpiDevObject *self, void *closure)
{
	PyObject *result;

	if (self->mode & SPI_CS_HIGH)
		result = Py_True;
	else
		result = Py_False;

	Py_INCREF(result);
	return result;
}

static PyObject *
SpiDev_get_lsbfirst(SpiDevObject *self, void *closure)
{
	PyObject *result;

	if (self->mode & SPI_LSB_FIRST)
		result = Py_True;
	else
		result = Py_False;

	Py_INCREF(result);
	return result;
}

static PyObject *
SpiDev_get_3wire(SpiDevObject *self, void *closure)
{
	PyObject *result;

	if (self->mode & SPI_3WIRE)
		result = Py_True;
	else
		result = Py_False;

	Py_INCREF(result);
	return result;
}

static PyObject *
SpiDev_get_loop(SpiDevObject *self, void *closure)
{
	PyObject *result;

	if (self->mode & SPI_LOOP)
		result = Py_True;
	else
		result = Py_False;

	Py_INCREF(result);
	return result;
}


static int
SpiDev_set_mode(SpiDevObject *self, PyObject *val, void *closure)
{
	uint8_t mode, tmp;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
	else if (!PyInt_Check(val)) {
		PyErr_SetString(PyExc_TypeError,
			"The mode attribute must be an integer");
		return -1;
	}

	mode = PyInt_AsLong(val);

	if ( mode > 3 ) {
		PyErr_SetString(PyExc_TypeError,
			"The mode attribute must be an integer"
				 "between 0 and 3.");
		return -1;
	}

	// clean and set CPHA and CPOL bits
	tmp = ( self->mode & ~(SPI_CPHA | SPI_CPOL) ) | mode ;

	__spidev_set_mode(self->fd, tmp);

	self->mode = tmp;
	return 0;
}

static int
SpiDev_set_cshigh(SpiDevObject *self, PyObject *val, void *closure)
{
	uint8_t tmp;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
	else if (!PyBool_Check(val)) {
		PyErr_SetString(PyExc_TypeError,
			"The cshigh attribute must be boolean");
		return -1;
	}

	if (val == Py_True)
		tmp = self->mode | SPI_CS_HIGH;
	else
		tmp = self->mode & ~SPI_CS_HIGH;

	__spidev_set_mode(self->fd, tmp);

	self->mode = tmp;
	return 0;
}

static int
SpiDev_set_lsbfirst(SpiDevObject *self, PyObject *val, void *closure)
{
	uint8_t tmp;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
	else if (!PyBool_Check(val)) {
		PyErr_SetString(PyExc_TypeError,
			"The lsbfirst attribute must be boolean");
		return -1;
	}

	if (val == Py_True)
		tmp = self->mode | SPI_LSB_FIRST;
	else
		tmp = self->mode & ~SPI_LSB_FIRST;

	__spidev_set_mode(self->fd, tmp);

	self->mode = tmp;
	return 0;
}

static int
SpiDev_set_3wire(SpiDevObject *self, PyObject *val, void *closure)
{
	uint8_t tmp;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
	else if (!PyBool_Check(val)) {
		PyErr_SetString(PyExc_TypeError,
			"The 3wire attribute must be boolean");
		return -1;
	}

	if (val == Py_True)
		tmp = self->mode | SPI_3WIRE;
	else
		tmp = self->mode & ~SPI_3WIRE;

	__spidev_set_mode(self->fd, tmp);

	self->mode = tmp;
	return 0;
}

static int
SpiDev_set_loop(SpiDevObject *self, PyObject *val, void *closure)
{
	uint8_t tmp;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
	else if (!PyBool_Check(val)) {
		PyErr_SetString(PyExc_TypeError,
			"The loop attribute must be boolean");
		return -1;
	}

	if (val == Py_True)
		tmp = self->mode | SPI_LOOP;
	else
		tmp = self->mode & ~SPI_LOOP;

	__spidev_set_mode(self->fd, tmp);

	self->mode = tmp;
	return 0;
}

static PyObject *
SpiDev_get_bits_per_word(SpiDevObject *self, void *closure)
{
	PyObject *result = Py_BuildValue("i", self->bits_per_word);
	Py_INCREF(result);
	return result;
}

static int
SpiDev_set_bits_per_word(SpiDevObject *self, PyObject *val, void *closure)
{
	uint8_t bits;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
	else if (!PyInt_Check(val)) {
		PyErr_SetString(PyExc_TypeError,
			"The bits_per_word attribute must be an integer");
		return -1;
	}

	bits = PyInt_AsLong(val);

        if (bits < 8 || bits > 16) {
		PyErr_SetString(PyExc_TypeError,
                                "invalid bits_per_word (8 to 16)");
		return -1;
	}

	if (self->bits_per_word != bits) {
		if (ioctl(self->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1) {
			PyErr_SetFromErrno(PyExc_IOError);
			return -1;
		}
		self->bits_per_word = bits;
	}
	return 0;
}

static PyObject *
SpiDev_get_max_speed_hz(SpiDevObject *self, void *closure)
{
	PyObject *result = Py_BuildValue("i", self->max_speed_hz);
	Py_INCREF(result);
	return result;
}

static int
SpiDev_set_max_speed_hz(SpiDevObject *self, PyObject *val, void *closure)
{
	uint32_t max_speed_hz;

	if (val == NULL) {
		PyErr_SetString(PyExc_TypeError,
			"Cannot delete attribute");
		return -1;
	}
	else if (!PyInt_Check(val)) {
		PyErr_SetString(PyExc_TypeError,
			"The max_speed_hz attribute must be an integer");
		return -1;
	}

	max_speed_hz = PyInt_AsLong(val);

	if (self->max_speed_hz != max_speed_hz) {
		if (ioctl(self->fd, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed_hz) == -1) {
			PyErr_SetFromErrno(PyExc_IOError);
			return -1;
		}
		self->max_speed_hz = max_speed_hz;
	}
	return 0;
}

static PyGetSetDef SpiDev_getset[] = {
	{"mode", (getter)SpiDev_get_mode, (setter)SpiDev_set_mode,
			"SPI mode as two bit pattern of \n"
			"Clock Polarity  and Phase [CPOL|CPHA]\n"
			"min: 0b00 = 0 max: 0b11 = 3\n"},
	{"cshigh", (getter)SpiDev_get_cshigh, (setter)SpiDev_set_cshigh,
			"CS active high\n"},
	{"threewire", (getter)SpiDev_get_3wire, (setter)SpiDev_set_3wire,
			"SI/SO signals shared\n"},
	{"lsbfirst", (getter)SpiDev_get_lsbfirst, (setter)SpiDev_set_lsbfirst,
			"LSB first\n"},
	{"loop", (getter)SpiDev_get_loop, (setter)SpiDev_set_loop,
			"loopback configuration\n"},
	{"bits_per_word", (getter)SpiDev_get_bits_per_word, (setter)SpiDev_set_bits_per_word,
			"bits per word\n"},
	{"max_speed_hz", (getter)SpiDev_get_max_speed_hz, (setter)SpiDev_set_max_speed_hz,
			"maximum speed in Hz\n"},
	{NULL},
};

PyDoc_STRVAR(SpiDev_open_doc,
	"open(bus, device)\n\n"
	"Connects the object to the specified SPI device.\n"
	"open(X,Y) will open /dev/spidev-X.Y\n");

static PyObject *
SpiDev_open(SpiDevObject *self, PyObject *args, PyObject *kwds)
{
	int bus, device;
	char path[SPIDEV_MAXPATH];
	uint8_t tmp8;
	uint32_t tmp32;
	static char *kwlist[] = {"bus", "device", NULL};
	if (!PyArg_ParseTupleAndKeywords(args, kwds, "ii:open", kwlist, &bus, &device))
		return NULL;
	if (snprintf(path, SPIDEV_MAXPATH, "/dev/spidev%d.%d", bus, device) >= SPIDEV_MAXPATH) {
		PyErr_SetString(PyExc_OverflowError,
			"Bus and/or device number is invalid.");
		return NULL;
	}
	if ((self->fd = open(path, O_RDWR, 0)) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	if (ioctl(self->fd, SPI_IOC_RD_MODE, &tmp8) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	self->mode = tmp8;
	if (ioctl(self->fd, SPI_IOC_RD_BITS_PER_WORD, &tmp8) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	self->bits_per_word = tmp8;
	if (ioctl(self->fd, SPI_IOC_RD_MAX_SPEED_HZ, &tmp32) == -1) {
		PyErr_SetFromErrno(PyExc_IOError);
		return NULL;
	}
	self->max_speed_hz = tmp32;

	Py_INCREF(Py_None);
	return Py_None;
}

static int
SpiDev_init(SpiDevObject *self, PyObject *args, PyObject *kwds)
{
	int bus = -1;
	int client = -1;
	static char *kwlist[] = {"bus", "client", NULL};

	if (!PyArg_ParseTupleAndKeywords(args, kwds, "|ii:__init__",
			kwlist, &bus, &client))
		return -1;

	if (bus >= 0) {
		SpiDev_open(self, args, kwds);
		if (PyErr_Occurred())
			return -1;
	}

	return 0;
}


PyDoc_STRVAR(SpiDevObjectType_doc,
	"SpiDev([bus],[client]) -> SPI\n\n"
	"Return a new SPI object that is (optionally) connected to the\n"
	"specified SPI device interface.\n");

static PyMethodDef SpiDev_methods[] = {
	{"open", (PyCFunction)SpiDev_open, METH_VARARGS | METH_KEYWORDS,
		SpiDev_open_doc},
	{"close", (PyCFunction)SpiDev_close, METH_NOARGS,
		SpiDev_close_doc},
	{"readbytes", (PyCFunction)SpiDev_readbytes, METH_VARARGS,
		SpiDev_read_doc},
	{"writebytes", (PyCFunction)SpiDev_writebytes, METH_VARARGS,
		SpiDev_write_doc},
	{"writeobject", (PyCFunction)SpiDev_writeobject, METH_VARARGS,
		SpiDev_write_doc},
	{"xfer", (PyCFunction)SpiDev_xfer, METH_VARARGS,
		SpiDev_xfer_doc},
	{"xfer2", (PyCFunction)SpiDev_xfer2, METH_VARARGS,
		SpiDev_xfer2_doc},
	{NULL},
};

static PyTypeObject SpiDevObjectType = {
	PyObject_HEAD_INIT(NULL)
	0,				/* ob_size */
	"SpiDev",			/* tp_name */
	sizeof(SpiDevObject),		/* tp_basicsize */
	0,				/* tp_itemsize */
	(destructor)SpiDev_dealloc,	/* tp_dealloc */
	0,				/* tp_print */
	0,				/* tp_getattr */
	0,				/* tp_setattr */
	0,				/* tp_compare */
	0,				/* tp_repr */
	0,				/* tp_as_number */
	0,				/* tp_as_sequence */
	0,				/* tp_as_mapping */
	0,				/* tp_hash */
	0,				/* tp_call */
	0,				/* tp_str */
	0,				/* tp_getattro */
	0,				/* tp_setattro */
	0,				/* tp_as_buffer */
	Py_TPFLAGS_DEFAULT,		/* tp_flags */
	SpiDevObjectType_doc,		/* tp_doc */
	0,				/* tp_traverse */
	0,				/* tp_clear */
	0,				/* tp_richcompare */
	0,				/* tp_weaklistoffset */
	0,				/* tp_iter */
	0,				/* tp_iternext */
	SpiDev_methods,			/* tp_methods */
	0,				/* tp_members */
	SpiDev_getset,			/* tp_getset */
	0,				/* tp_base */
	0,				/* tp_dict */
	0,				/* tp_descr_get */
	0,				/* tp_descr_set */
	0,				/* tp_dictoffset */
	(initproc)SpiDev_init,		/* tp_init */
	0,				/* tp_alloc */
	SpiDev_new,			/* tp_new */
};

static PyMethodDef SpiDev_module_methods[] = {
	{NULL}
};

#ifndef PyMODINIT_FUNC	/* declarations for DLL import/export */
#define PyMODINIT_FUNC void
#endif
PyMODINIT_FUNC
initspidev(void)
{
	PyObject* m;

	if (PyType_Ready(&SpiDevObjectType) < 0)
		return;

	m = Py_InitModule3("spidev", SpiDev_module_methods, SpiDev_module_doc);
	Py_INCREF(&SpiDevObjectType);
	PyModule_AddObject(m, "SpiDev", (PyObject *)&SpiDevObjectType);
}

