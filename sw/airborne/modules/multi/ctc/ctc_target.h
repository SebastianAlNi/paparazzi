/*
 * Copyright (C) 2017  Hector Garcia de Marina
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file ctc_target.h
 *
 *  Collective Tracking Control (module for the target to be tracked)
 */

#ifndef CTC_TARGET_H
#define CTC_TARGET_H

#include "std.h"

extern int16_t tableNei[][6];

extern void ctc_target_init(void);
extern void ctc_target_send_info_to_nei(void);

extern void parse_ctc_target_RegTable(void);
extern void parse_ctc_target_CleanTable(void);

#endif // CTC_TARGET_H
