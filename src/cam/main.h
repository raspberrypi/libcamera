/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main.h - Cam application
 */
#ifndef __CAM_MAIN_H__
#define __CAM_MAIN_H__

enum {
	OptCamera = 'c',
	OptCapture = 'C',
	OptFile = 'F',
	OptHelp = 'h',
	OptInfo = 'I',
	OptList = 'l',
	OptListProperties = 'p',
	OptMonitor = 'm',
	OptStream = 's',
	OptListControls = 256,
	OptStrictFormats = 257,
};

#endif /* __CAM_MAIN_H__ */
