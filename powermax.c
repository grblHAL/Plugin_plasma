/*

  powermax.c - PowerMax plasma cutter RS-485 communication

  Part of grblHAL

  Copyright (c) 2025 Terje Io

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.

*/

#include "thc.h"

#if PLASMA_ENABLE && 0

#include <math.h>
#include <string.h>

#include "../spindle/modbus_rtu.h"

static uint32_t modbus_address = 1;

static void rx_packet (modbus_message_t *msg);
static void rx_exception (uint8_t code, void *context);

static const modbus_callbacks_t callbacks = {
    .retries = 5,
    .retry_delay = 50,
    .on_rx_packet = rx_packet,
    .on_rx_exception = rx_exception
};

static bool set_cut_mode (uint16_t mode)
{
    modbus_message_t rpm_cmd = {
        .context = (void *)Plasma_SetMode,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x20,
        .adu[3] = 0x93,
        .adu[4] = mode >> 8,
        .adu[5] = mode & 0xFF,
        .tx_length = 8,
        .rx_length = 8
    };

    return modbus_send(&rpm_cmd, &callbacks, true);
}

static bool set_amperage (float a)
{
    uint16_t amps = (uint16_t)(a * 64);

    modbus_message_t rpm_cmd = {
        .context = (void *)Plasma_SetCurrent,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x20,
        .adu[3] = 0x94,
        .adu[4] = amps >> 8,
        .adu[5] = amps & 0xFF,
        .tx_length = 8,
        .rx_length = 8
    };

    return modbus_send(&rpm_cmd, &callbacks, true);
}

static bool set_gas_pressure (float psi)
{
    uint16_t pressure = (uint16_t)(psi * 128);

    modbus_message_t mode_cmd = {
        .context = (void *)Plasma_SetPressure,
        .crc_check = false,
        .adu[0] = modbus_address,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x20,
        .adu[3] = 0x96,
        .adu[4] = pressure >> 8,
        .adu[5] = pressure & 0xFF,
        .tx_length = 8,
        .rx_length = 8
    };

    return modbus_send(&mode_cmd, &callbacks, true);
}

static void rx_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {
/*
        switch((vfd_response_t)msg->context) {

            case VFD_GetRPM:
                spindle_validate_at_speed(spindle_data, f2rpm((msg->adu[3] << 8) | msg->adu[4]));
                break;

            case VFD_GetMinRPM:
                freq_min = (msg->adu[3] << 8) | msg->adu[4];
                break;

            case VFD_GetMaxRPM:
                freq_max = (msg->adu[3] << 8) | msg->adu[4];
                spindle_hal->cap.rpm_range_locked = On;
                spindle_hal->rpm_min = f2rpm(freq_min);
                spindle_hal->rpm_max = f2rpm(freq_max);
                break;

            default:
                break;
        }
*/
    }
}

static void rx_exception (uint8_t code, void *context)
{
// raise alarm?
}

/*
static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("PowerMax RS-485", "0.01");
}
*/

void powermax_init (void)
{
    cutter.set_cut_mode = set_cut_mode;
    cutter.set_current = set_amperage;
    cutter.set_pressure = set_gas_pressure;
}

#endif
