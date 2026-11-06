# ************************************************************************************
# Copyright (c) [2025] ams-OSRAM AG
#
# SPDX-License-Identifier: GPL-2.0 OR MIT
#
# For full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.
# ************************************************************************************

CC = gcc
TARGET = tmf8829

ENABLE_JSON_LOGGING = 1

ENABLE_HISTOGRAM = 1

ENABLE_KEYSTONE = 1

SRCS = main.c tmf8829.c tmf8829_driver.c tmf8829_frameparser.c tmf8829_shim.c

CFLAGS = -Wall -Wextra -O2

ifdef ENABLE_JSON_LOGGING
CFLAGS += -DENABLE_JSON_LOGGING
SRCS += tmf8829_json.c
endif

ifdef ENABLE_HISTOGRAM
CFLAGS += -DENABLE_HISTOGRAM
endif

ifdef ENABLE_KEYSTONE
CFLAGS += -DENABLE_KEYSTONE
SRCS += tmf8829_keystone.c
endif

OBJS = $(SRCS:.c=.o)

LDFLAGS = -lm -lpthread

ifdef ENABLE_JSON_LOGGING
LDFLAGS += -lz
endif

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $(TARGET) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@ -MD -MP

-include $(OBJS:.o=.d)

clean:
	rm -f $(OBJS) $(TARGET) $(OBJS:.o=.d)

rebuild: clean all

install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/

uninstall:
	rm -f /usr/local/bin/$(TARGET)

.PHONY: all clean rebuild install uninstall
