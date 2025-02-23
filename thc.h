/*

  thc.h - plasma cutter tool height control plugin

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

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

#pragma once

#include "driver.h"

#if PLASMA_ENABLE

typedef enum {
    Plasma_SetMode = 0,
    Plasma_SetCurrent,
    Plasma_SetPressure,
    Plasma_GetCurrentMin,
    Plasma_GetCurrentMax,
    Plasma_GetPressureMin,
    Plasma_GetPressureMax,
    Plasma_GetArcOnTimeLow,
    Plasma_GetArcOnTimeHigh,
} plasma_rs485_msg_t;

typedef struct material
{
    int32_t id;                     // nu
    char name[50];                  // na
    bool thc_status;                // th
    union {
        float params[12];
        struct {
            float pierce_height;    // ph - mandatory
            float pierce_delay;     // pd - mandatory
            float feed_rate;        // fr - mandatory
            float cut_height;       // ch - mandatory
            float cut_voltage;      // cv
            float pause_at_end;     // pe
            float kerf_width;       // kw
            float cut_amps;         // ca - PowerMax
            float gas_pressure;     // gp - PowerMax
            float cut_mode;         // cm - PowerMax
            float jump_height;      // jh
            float jump_delay;       // jd
        };
    };
    struct material *next;
} material_t;

typedef bool plasma_enumerate_materials_callback_ptr (material_t *material, void *data);
typedef bool (*plasma_set_current_ptr)(float current);
typedef bool (*plasma_set_pressure_ptr)(float psi);
typedef bool (*plasma_set_cut_mode_ptr)(uint16_t mode);

typedef struct {
    plasma_set_current_ptr set_current;
    plasma_set_pressure_ptr set_pressure;
    plasma_set_cut_mode_ptr set_cut_mode;
} plasma_control_t;

extern plasma_control_t cutter;

bool plasma_material_is_valid (material_t *material);
bool plasma_material_add (material_t *material, bool overwrite);
bool plasma_enumerate_materials (plasma_enumerate_materials_callback_ptr callback, void *data);

#endif
