/* board-htcleo-mmc.h
 *
 * Copyright (C) 2011 marc1706
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef HTCLEO_AUDIO_H
#define HTCLEO_AUDIO_H

void htcleo_headset_enable(int en);
void htcleo_speaker_enable(int en);
void htcleo_receiver_enable(int en);
void htcleo_bt_sco_enable(int en);
void htcleo_mic_enable(int en);
void htcleo_analog_init(void);
int htcleo_get_rx_vol(uint8_t hw, int level);
void __init htcleo_audio_init(void);

#endif // HTCLEO_AUDIO_H

