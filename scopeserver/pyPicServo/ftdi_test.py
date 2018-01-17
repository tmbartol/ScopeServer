#!/usr/bin/env python3

import pylibftdi


dev = pylibftdi.Device()
dev.ftdi_fn.ftdi_set_line_property(8, 1, 0)
dev.ftdi_fn.ftdi_set_latency_timer(1)

