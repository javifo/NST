/*
 * tps65921_keypad.h
 *
 * Copyright (C) 2011 Barnes & Noble, Inc.
 * Intrinsyc Software International, Inc. on behalf of Barnes & Noble, Inc.
 *
 * Keypad read for u-boot
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef __INCLUDED_TPS65921_KEYPAD_H
#define __INCLUDED_TPS65921_KEYPAD_H

// Keep the key definitions consistent between hardware versions.
#define FORWARD_KEY	(1 << 0)
#define BACK_KEY	(1 << 1)
#define PGFORWARD_KEY	(1 << 2)
#define PGBACK_KEY	(1 << 3)
#define MENU_KEY	(1 << 4)

int tps65921_keypad_init(void);
int tps65921_keypad_reset(void);
int tps65921_keypad_keys_pressed(unsigned int *key);

#endif
