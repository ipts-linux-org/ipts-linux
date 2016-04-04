#
# Makefile - Intel Management Engine Interface (Intel MEI) Linux driver
# Copyright (c) 2010-2014, Intel Corporation.
#

obj-$(CONFIG_INTEL_MEI_ITOUCH)+= mei-itouch-hid.o
mei-itouch-hid-objs += mei-hid.o
mei-itouch-hid-objs += mei-hid-itouch.o
mei-itouch-hid-objs += mei-hid-gfx-itouch.o
mei-itouch-hid-objs += mei-itouch-client.o
mei-itouch-hid-objs += mei-itouch-messages.o
mei-itouch-hid-objs += mei-itouch-gfx-messages.o
mei-itouch-hid-objs += itouch-gfx-interface.o
mei-itouch-hid-objs += itouch-gfx-hid-interface.o
mei-itouch-hid-objs += itouch-state-handler.o
mei-itouch-hid-$(CONFIG_DEBUG_FS) += debugfs.o
