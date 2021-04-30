/*
 * Copyright (C) 2011 Texas Instruments Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */
#ifndef _VCAP_TYPES_H
#define _VCAP_TYPES_H

#include <linux/i2c.h>

#define VCAP_CAPTURE_MAX_CHANNELS	2

enum vcap_if_type {
	VCAP_IF_BT656,
	VCAP_IF_BT1120,
	VCAP_IF_RAW_BAYER
};

struct vcap_interface {
	enum vcap_if_type if_type;
	unsigned hd_pol:1;
	unsigned vd_pol:1;
	unsigned fid_pol:1;
};
#endif /* _VCAP_TYPES_H */
