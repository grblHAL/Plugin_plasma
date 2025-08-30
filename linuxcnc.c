/*

  linuxcnc.c - plasma cutter tool height control plugin

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

#if PLASMA_ENABLE

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "grbl/vfs.h"
#include "grbl/strutils.h"

static on_vfs_mount_ptr on_vfs_mount;

static void load_tools (const char *path, const vfs_t *fs, vfs_st_mode_t mode)
{
    // NOTE: must match layout of material_t
    static const char params[] = "PIERCE_HEIGHT,PIERCE_DELAY,CUT_SPEED,CUT_HEIGHT,CUT_VOLTS,PAUSE_AT_END,PUDDLE_JUMP_HEIGHT,PUDDLE_JUMP_DELAY,KERF_WIDTH,CUT_AMPS,GAS_PRESSURE,CUT_MODE,-,NAME,THC";

    char c, *eq, buf[100];
    uint_fast8_t idx = 0;
    vfs_file_t *file;
    status_code_t status = Status_GcodeUnusedWords;

    material_t material;

    if((file = vfs_open("/linuxcnc/material.cfg", "r"))) {

        while(vfs_read(&c, 1, 1, file) == 1) {

            if(c == ASCII_CR || c == ASCII_LF) {

                buf[idx] = '\0';

                if(*buf == '[') {

                    if(status == Status_OK && plasma_material_is_valid(&material))
                        plasma_material_add(&material, true);

                    status = Status_GcodeUnusedWords;

                    if((eq = strchr(buf, ']'))) {

                        *eq = '\0';

                        if((eq = strrchr(buf, '_'))) {
                            uint32_t id;
                            uint_fast8_t cc = 1;
                            if((status = read_uint(eq, &cc, &id)) == Status_OK) {
                                material.id = (int32_t)id;
                                material.thc_status = true; //?
                                *material.name = '\0';
                                for(idx = 0; idx < sizeof(material.params) / sizeof(float); idx++)
                                    material.params[idx] = NAN;
                            }
                        }
                    }
                }

                if(status == Status_OK && (eq = strchr(buf, '='))) {

                    int32_t p;

                    *eq++ = '\0';

                    while(*eq == ' ')
                        eq++;

                    switch((p = strlookup(strtok(buf, " "), params, ','))) {

                        case -1:
                            status = Status_GcodeUnsupportedCommand;
                            break;

                        case 13:
                            strncpy(material.name, eq, sizeof(material.name) - 1);
                            material.name[sizeof(material.name) - 1] = '\0';
                            break;

                        case 14:
                            material.thc_status = *eq != '0';
                            break;

                        default:
                            {
                                uint_fast8_t cc = 0;
                                if(!read_float(eq, &cc, &material.params[p]))
                                    status = Status_BadNumberFormat;
                            }
                            break;
                    }
                }
                idx = 0;
            } else if(idx < sizeof(buf))
                buf[idx++] = c;
        }

        if(status == Status_OK && plasma_material_is_valid(&material))
            plasma_material_add(&material, true);

        vfs_close(file);
    }

    if(on_vfs_mount)
        on_vfs_mount(path, fs, mode);
}

void linuxcnc_init (void)
{
    on_vfs_mount = vfs.on_mount;
    vfs.on_mount = load_tools;
}

#endif // PLASMA_ENABLE
