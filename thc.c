/*

  thc.c - plasma cutter tool height control plugin

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

#include "thc.h"

#if PLASMA_ENABLE

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "grbl/config.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/pid.h"
#include "grbl/nvs_buffer.h"
#include "grbl/stepper2.h"
#include "grbl/state_machine.h"
#include "grbl/strutils.h"
#include "grbl/motion_control.h"
#include "grbl/task.h"
#if NGC_EXPRESSIONS_ENABLE
#include "grbl/ngc_expr.h"
#endif

extern void linuxcnc_init (void);
extern void sheetcam_init (void);

#define THC_SAMPLE_AVG                  5
#define PLASMA_TMP_MATERIAL_ID_START    1000000
// Digital ports
#define PLASMA_THC_DISABLE_PORT         2 // output
#define PLASMA_TORCH_DISABLE_PORT       3 // output
// Analog ports
#define PLASMA_FEED_OVERRIDE_PORT       3

typedef enum {
    Plasma_ModeOff = 0,
    Plasma_ModeVoltage,
    Plasma_ModeUpDown,
    Plasma_ModeArcOK
} plasma_mode_t;

// float parameters must be in the same order as in material_t.
typedef struct {
    int32_t id;
    bool thc_disabled;
    union {
        float params[6];
        struct {
            float pierce_height;
            float pierce_delay;
            float feed_rate;
            float cut_height;
            float cut_voltage;
            float pause_at_end;
        };
    };
} plasma_job_t;

typedef union {
    uint8_t flags;
    struct {
        uint8_t virtual_ports :1,
                sync_pos      :1,
                onoffmode     :1,
                unassigned    :5;
    };
} thc_options_t;

typedef struct {
    float thc_delay;
    float thc_threshold;
    uint32_t vad_threshold;
    uint32_t thc_override;
    float pierce_height;
    float pierce_delay;
    float pause_at_end;
    float arc_retry_delay;
    float arc_fail_timeout;
    float arc_voltage_scale;
    float arc_voltage_offset;
    float arc_height_per_volt;
    float arc_ok_low_voltage;
    float arc_high_low_voltage;
    uint8_t arc_retries;
    thc_options_t option;
    uint8_t unused1;
    uint8_t unused2;
    plasma_mode_t mode;
    pid_values_t pid;
    uint8_t port_arc_voltage;
    uint8_t port_arc_ok;
    uint8_t port_cutter_down;
    uint8_t port_cutter_up;
} plasma_settings_t;

typedef union {
    uint16_t bits;
    uint16_t value;
    struct {
        uint16_t arc_ok        :1,
                 torch_on      :1,
                 enabled       :1,
                 ohmic_probe   :1,
                 float_switch  :1,
                 breakaway     :1,
                 active        :1,
                 up            :1,
                 down          :1,
                 velocity_lock :1,
                 void_lock     :1,
                 report_up     :1,
                 report_down   :1,
                 unassigned    :3;
    };
} thc_signals_t;

static void state_idle (void);
static void state_thc_delay (void);
static void state_thc_pid (void);
static void state_thc_adjust (void);
static void state_arc_monitor (void);
static void state_vad_lock (void);

plasma_control_t cutter = {};

static bool set_feed_override = false, updown_enabled = false, init_ok = false;
static uint8_t n_ain, n_din;
static uint8_t port_arc_ok, port_arc_voltage;
static uint_fast8_t feed_override, segment_id = 0;
static uint32_t thc_delay = 0, v_count = 0;
static int32_t step_count;
static char max_aport[4], max_dport[4];
static float arc_vref = 0.0f, arc_voltage = 0.0f, arc_voltage_low, arc_voltage_high; //, vad_threshold;
static float fr_pgm, fr_actual, fr_thr_99, fr_thr_vad;
static thc_signals_t thc = {0};
static pidf_t pid;
static nvs_address_t nvs_address;
static char thc_modes[] = "Off,Voltage,Up/down,Arc ok";
static plasma_settings_t plasma;
static st2_motor_t *z_motor;
static void (*volatile stateHandler)(void) = state_idle;
static plasma_mode_t mode = Plasma_ModeOff;
static xbar_t arc_ok, cutter_down, cutter_up, parc_voltage;
static uint32_t material_id = PLASMA_TMP_MATERIAL_ID_START;
static material_t *materials = NULL, tmp_material = { .id = -1 };
static plasma_job_t job = {};

static settings_changed_ptr settings_changed;
static driver_reset_ptr driver_reset = NULL;
static spindle_set_state_ptr spindle_set_state_ = NULL;
//static control_signals_callback_ptr control_interrupt_callback = NULL;
static stepper_pulse_start_ptr stepper_pulse_start = NULL;
static enumerate_pins_ptr enumerate_pins;
static on_report_options_ptr on_report_options;
static on_spindle_selected_ptr on_spindle_selected;
static on_execute_realtime_ptr on_execute_realtime = NULL;
static on_realtime_report_ptr on_realtime_report = NULL;
static on_gcode_message_ptr on_gcode_comment;
static user_mcode_ptrs_t user_mcode;

static const char params[] = "ph,pd,fr,ch,cv,pe,kw,ca,gp,cm,jh,jd,nu,na,th"; // NOTE: must match layout of material_t

// --- Virtual ports start

static io_ports_data_t digital, analog;

typedef struct {
    uint8_t pin_id;
    pin_info_ptr pin_info;
    void *data;
    bool aux_dout0;
    bool aux_dout1;
    pin_function_t aux_dout2;
    pin_function_t aux_dout3;
    bool aux_aout0;
    bool aux_aout1;
    bool aux_aout2;
    pin_function_t aux_aout3;
} pin_stat_t;

static pin_stat_t pin_stat = {};

static const char *add_comment (const char *description, char *new_descr)
{
    size_t len;

    if((len = strlen(description)) <= 3) {
        if(len == 2)
            new_descr++;
        do {
            len--;
            new_descr[len] = description[len];
        } while(len);

    } else
        new_descr = (char *)description;

    return (const char *)new_descr;
}

static void enum_trap (xbar_t *pin, void *data)
{
    static char remap[3][30] = {
        { "   , remapped from P2 by THC" },
        { "   , remapped from P3 by THC" },
        { "   , remapped from E3 by THC" }
    };

    if(((pin_stat_t *)data)->pin_info) {
 
        if(pin->group == PinGroup_AuxOutput) {
            if(pin->function == ((pin_stat_t *)data)->aux_dout2)
                pin->description = add_comment(pin->description, remap[0]);
            if(pin->function == ((pin_stat_t *)data)->aux_dout3)
                pin->description = add_comment(pin->description, remap[1]);
        } else if(pin->group == PinGroup_AuxOutputAnalog && pin->function == ((pin_stat_t *)data)->aux_aout3)
             pin->description = add_comment(pin->description, remap[2]);

        ((pin_stat_t *)data)->pin_info(pin, ((pin_stat_t *)data)->data);
        ((pin_stat_t *)data)->pin_id = max(((pin_stat_t *)data)->pin_id, pin->id);

     } else if(pin->group == PinGroup_AuxOutput || pin->group == PinGroup_AuxOutputAnalog) {

        if(!strcmp(pin->description, "P0"))
            ((pin_stat_t *)data)->aux_dout0 = true;
        else if(!strcmp(pin->description, "P1"))
            ((pin_stat_t *)data)->aux_dout1 = true;
        else if(!strcmp(pin->description, "P2"))
            ((pin_stat_t *)data)->aux_dout2 = pin->function;
        else if(!strcmp(pin->description, "P3"))
            ((pin_stat_t *)data)->aux_dout3 = pin->function;
        else if(!strcmp(pin->description, "E0"))
            ((pin_stat_t *)data)->aux_aout0 = true;
        else if(!strcmp(pin->description, "E1"))
            ((pin_stat_t *)data)->aux_aout1 = true;
        else if(!strcmp(pin->description, "E2"))
            ((pin_stat_t *)data)->aux_aout2 = true;
        else if(!strcmp(pin->description, "E3"))
            ((pin_stat_t *)data)->aux_aout3 = pin->function;
    }
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    pin_stat.pin_id = 0;
    pin_stat.pin_info = pin_info;
    pin_stat.data = data;

    static xbar_t pin = {
        .id = 0,
        .pin = 0,
        .group = PinGroup_Virtual,
        .function = Virtual_Pin,
        .cap.claimable = Off,
        .mode.output = On,
        .mode.claimed = On
    };

    enumerate_pins(low_level, enum_trap, &pin_stat);

    if(!pin_stat.aux_dout0) {
        pin.id = ++pin_stat.pin_id;
        pin.description = "P0,Dummy out";
        pin_info(&pin, data);
    }

    if(!pin_stat.aux_dout1) {
        pin.id = ++pin_stat.pin_id;
        pin.description = "P1,Dummy out";
        pin_info(&pin, data);
    }

    pin.id = ++pin_stat.pin_id;
    pin.port = "THCD",
    pin.description = "P2,THC on/off";
    pin_info(&pin, data);

    pin.id = ++pin_stat.pin_id;
    pin.pin++;
    pin.description = "P3,THC torch control";
    pin_info(&pin, data);

    pin.pin = 0;
    pin.port = "THCA";
    pin.mode.analog = On;

    if(!pin_stat.aux_aout0) {
        pin.id = ++pin_stat.pin_id;
        pin.description = "E0,Dummy out";
        pin_info(&pin, data);
        pin.pin++;
    }

    if(!pin_stat.aux_aout1) {
        pin.id = ++pin_stat.pin_id;
        pin.description = "E1,Dummy out";
        pin_info(&pin, data);
        pin.pin++;
    }

    if(!pin_stat.aux_aout1) {
        pin.id = ++pin_stat.pin_id;
        pin.description = "E2,Dummy out";
        pin_info(&pin, data);
        pin.pin++;
    }

    pin.id = ++pin_stat.pin_id;
    pin.port = "THCA";
    pin.description = "E3,THC feed override";
    pin_info(&pin, data);
}

static bool analog_out (uint8_t portnum, float value)
{
    portnum -= analog.out.n_ports - 1;

    if(portnum == 0) { // PLASMA_FEED_OVERRIDE_PORT
        // Let the foreground process handle this
        set_feed_override = true;
        feed_override = (uint_fast8_t)value;
        if(feed_override < 10 || feed_override > 100)
            feed_override = 100;
    }

    return true;
}

static xbar_t *get_apin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin = {0};

    xbar_t *info = NULL;

    if(port < analog.out.n_ports) {

        pin.id = pin.pin = port;
        pin.cap.output = pin.cap.analog = On;
        pin.cap.claimable = Off;
        pin.mode.output = pin.cap.analog = On;
        pin.description = "THC feed override";

        info = &pin;
    }

    return info;
}

static void digital_out (uint8_t portnum, bool on)
{
    portnum -= digital.out.n_ports - 2;

    if(portnum == 0) { // PLASMA_THC_DISABLE_PORT
        job.thc_disabled = on;
        if(thc.arc_ok && mode != Plasma_ModeArcOK) {
            if(!(thc.enabled = !job.thc_disabled))
                stateHandler = state_arc_monitor;
            else
                stateHandler = mode == Plasma_ModeUpDown ? state_thc_adjust : state_thc_pid;
        }
    } else if(portnum == 1) { // PLASMA_TORCH_DISABLE_PORT:
        // TODO
    }
}

static xbar_t *get_dpin_info (io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin = {0};

    xbar_t *info = NULL;

    if(port < digital.out.n_ports) {

        pin.id = pin.pin = port;
        pin.cap.output = On;
        pin.cap.claimable = Off;
        pin.mode.output = On;
        pin.description = port == digital.out.n_start ? "THC enable/disable" : "THC torch control";

        info = &pin;
    }

    return info;
}

static void set_pin_description (io_port_direction_t dir, uint8_t port, const char *s)
{
    // NOOP
}

static void add_virtual_ports (void *data)
{
    uint8_t aux_dout = 2, aux_aout = 1;
 
    pin_stat.pin_info = NULL;
    pin_stat.aux_aout3 = pin_stat.aux_dout2 = pin_stat.aux_dout3 = Virtual_Pin;

    if(hal.enumerate_pins)
        hal.enumerate_pins(false, enum_trap, &pin_stat);

    if(!pin_stat.aux_dout0)
        aux_dout++;
    if(!pin_stat.aux_dout1)
        aux_dout++;
    if(!pin_stat.aux_aout0)
        aux_aout++;
    if(!pin_stat.aux_aout1)
        aux_aout++;
    if(!pin_stat.aux_aout2)
        aux_aout++;

    if(aux_aout) {

        analog.out.n_ports = aux_aout;

        io_analog_t ports = {
            .ports = &analog,
            .analog_out = analog_out,
            .get_pin_info = get_apin_info,
            .set_pin_description = set_pin_description
        };

        if(ioports_add_analog(&ports) && digital.out.n_start > 0)
            ioport_remap(Port_Analog, Port_Output, analog.out.n_start + aux_aout - 1, PLASMA_FEED_OVERRIDE_PORT);
    }

    if(aux_dout) {

        digital.out.n_ports = aux_dout;

        io_digital_t ports = {
            .ports = &digital,
            .digital_out = digital_out,
            .get_pin_info = get_dpin_info,
            .set_pin_description = set_pin_description
        };

        if(ioports_add_digital(&ports) && digital.out.n_start > 2) {
            ioport_remap(Port_Digital, Port_Output, digital.out.n_start + aux_dout - 2, PLASMA_THC_DISABLE_PORT);
            ioport_remap(Port_Digital, Port_Output, digital.out.n_start + aux_dout - 1, PLASMA_TORCH_DISABLE_PORT);
        }
    }

    enumerate_pins = hal.enumerate_pins;
    hal.enumerate_pins = enumeratePins;
}

// --- Virtual ports end

// --- Materials handling start

static void set_job_params (material_t *material)
{
    uint_fast8_t idx;

    job.id = material ? material->id : -1;
    job.thc_disabled = (material && !material->thc_status) || plasma.mode == Plasma_ModeOff || plasma.mode == Plasma_ModeArcOK;

    if(material) {

        for(idx = 0; idx < sizeof(job.params) / sizeof(float); idx++)
            job.params[idx] = material->params[idx];

        if(!isnanf(material->cut_mode) && cutter.set_cut_mode)
            cutter.set_cut_mode((uint16_t)material->cut_mode);

        if(!isnanf(material->cut_amps) && cutter.set_current)
            cutter.set_current(material->cut_amps);

        if(!isnanf(material->gas_pressure) && cutter.set_pressure)
            cutter.set_pressure(material->gas_pressure);

#if NGC_EXPRESSIONS_ENABLE
        ngc_named_param_set("_hal[plasmac.cut-feed-rate]", job.feed_rate);
#endif
    } else {

        for(idx = 0; idx < sizeof(job.params) / sizeof(float); idx++)
            job.params[idx] = NAN;

        job.pierce_height = plasma.pierce_height;
        job.pierce_delay = plasma.pierce_delay;
        job.pause_at_end = plasma.pause_at_end;
    }

    if(material && *material->name)
        report_message(material->name, Message_Info);
}

static material_t *find_material (uint32_t id)
{
    material_t *material = materials, *found = NULL;

    if(material) do {
        if(material->id == id)
            found = material;
    } while(found == NULL && (material = material->next));

    return found;
}

static user_mcode_type_t mcode_check (user_mcode_t mcode)
{
    return mcode == Plasma_SelectMaterial
                     ? UserMCode_Normal
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t mcode_validate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    if(gc_block->user_mcode == Plasma_SelectMaterial) {
        if(gc_block->words.p) {
            if(!isintf(gc_block->values.p))
                state = Status_BadNumberFormat;
            else if(gc_block->words.p && !(gc_block->values.p == -1.0f || find_material((uint32_t)gc_block->values.p)))
                state = Status_GcodeValueOutOfRange;
            else
                gc_block->words.p = Off;
        }
    } else
        state = Status_Unhandled;

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    if(gc_block->user_mcode == Plasma_SelectMaterial)
        set_job_params(gc_block->values.p == -1.0f ? NULL : find_material((uint32_t)gc_block->values.p));
    else if(user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

bool plasma_enumerate_materials (plasma_enumerate_materials_callback_ptr callback, void *data)
{
    bool ok = false;
    material_t *material = materials;

    if(job.id == tmp_material.id)
        ok = callback(&tmp_material, data);

    if(!ok && material) do {
        ok = callback(material, data);
    } while(!ok && (material = material->next));

    return ok;
}

bool plasma_material_is_valid (material_t *material)
{
    return !(material->id < 0 ||
              material->id >= PLASMA_TMP_MATERIAL_ID_START ||
               isnanf(material->pierce_height) ||
                isnanf(material->pierce_delay) ||
                 isnanf(material->cut_height) ||
                  isnanf(material->feed_rate));
}

bool plasma_material_add (material_t *material, bool overwrite)
{
    material_t *m = find_material(material->id), *next = m ? m->next : NULL;
    bool add = m == NULL;

    if(overwrite || m == NULL) {
        if(m == NULL)
            m = malloc(sizeof(material_t));
        if(m) {
            memcpy(m, material, sizeof(material_t));
            m->next = next;
            if(materials == NULL)
                materials = m;
            else if(add) {
                material_t *last = materials;
                while(last->next)
                    last = last->next;
                last->next = m;
            }
        } // else error....
    }

    return true;
}

static bool ml_enumerate (material_t *material, void *data)
{
    uint_fast8_t i;
    char lbl[3], el[]= "|  :";

    hal.stream.write("[MATERIAL:o:");
    hal.stream.write(uitoa(material->id >= PLASMA_TMP_MATERIAL_ID_START ? 0 : 2));
    hal.stream.write("|nu:");
    hal.stream.write(uitoa(material->id));
    if(*material->name) {
        hal.stream.write("|na:");
        hal.stream.write(material->name);
    }
    hal.stream.write("|th:");
    hal.stream.write(uitoa(material->thc_status));

    for(i = 0; i < sizeof(material->params) / sizeof(float); i++) {
        if(!isnanf(material->params[i])) {
            strgetentry(lbl, params, i, ',');
            el[1] = lbl[0];
            el[2] = lbl[1];
            hal.stream.write(el);
            hal.stream.write(trim_float(ftoa(material->params[i], ngc_float_decimals())));
        }
    }
    hal.stream.write("]" ASCII_EOL);

    return false;
}

static status_code_t onGcodeComment (char *comment)
{
    status_code_t status = Status_OK;

    if(strlen(comment) > 5 && comment[0] == 'o' && comment[1] == '=') {

        char option = comment[2];
        material_t material = {};

        uint_fast8_t i;

        for(i = 0; i < sizeof(material.params) / sizeof(float); i++)
            material.params[i] = NAN;

        char *param = strtok(comment + 4, ","), *eq;

        while(param && status == Status_OK) {

            while(*param == ' ')
                param++;

            if((eq = strchr(param, '='))) {

                int32_t p;

                *eq = '\0';

                switch((p = strlookup(param, params, ','))) {

                    case -1:
                        status = Status_GcodeUnsupportedCommand;
                        break;

                    case 12:
                        if(option != '0') {
                            uint32_t id;
                            uint_fast8_t cc = 1;
                            if((status = read_uint(eq, &cc, &id)) == Status_OK)
                                material.id = (int32_t)id;
                        }
                        break;

                    case 13:
                        strncpy(material.name, eq + 1, sizeof(material.name) - 1);
                        break;

                    case 14:
                        material.thc_status = eq[1] != '0';
                        break;

                    default:
                        {
                            uint_fast8_t cc = 1;
                            if(!read_float(eq, &cc, &material.params[p]))
                                status = Status_BadNumberFormat;
                        }
                        break;
                }
                *eq = '=';
            }
            param = strtok(NULL, ",");
        }

        if(status == Status_OK && !plasma_material_is_valid(&material))
            status = Status_GcodeValueWordMissing;

        if(status == Status_OK) switch(option) {

            case '0':
                material.id = material_id++;
                memcpy(&tmp_material, &material, sizeof(material_t));
                set_job_params(&tmp_material);
                break;

            case '1':
            case '2':
                plasma_material_add(&material, option == '2');
                break;

            default:
                status = Status_GcodeUnsupportedCommand;
                break;
        }

    } else if(on_gcode_comment)
        status = on_gcode_comment(comment);

    return status;
}

// --- Materials handling end

static bool moveto (float z_position)
{
    bool ok;
    coord_data_t target;
    plan_line_data_t plan_data;

    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    system_convert_array_steps_to_mpos(target.values, sys.position);

    target.z = z_position + gc_get_offset(Z_AXIS, false);
    if((ok = mc_line(target.values, &plan_data))) {
        protocol_buffer_synchronize();
        sync_position();
    }

    return ok;
}

static void set_target_voltage (float v)
{
    arc_vref = arc_voltage = v;
    arc_voltage_low  = arc_vref - plasma.thc_threshold;
    arc_voltage_high = arc_vref + plasma.thc_threshold;
    v_count = 0;
}

static void pause_on_error (void)
{
    stateHandler = state_idle;
    system_set_exec_state_flag(EXEC_FEED_HOLD);   // Set up program pause for manual tool change
//    system_set_exec_state_flag(EXEC_TOOL_CHANGE);   // Set up program pause for manual tool change
    protocol_execute_realtime();                    // Execute...
}

/* THC state machine */

static void state_idle (void)
{
    if(mode == Plasma_ModeVoltage)
        arc_voltage = parc_voltage.get_value(&parc_voltage) * plasma.arc_voltage_scale - plasma.arc_voltage_offset;

    if(plasma.option.sync_pos && state_get() == STATE_IDLE) {

        if(mode != Plasma_ModeUpDown)
            step_count =(uint32_t)st2_get_position(z_motor);

        if(step_count && state_get() == STATE_IDLE) {
            sys.position[Z_AXIS] += step_count;
            step_count = 0;
            st2_set_position(z_motor, 0LL);
            sync_position();
        }
    }
}

static void state_thc_delay (void)
{
    if(hal.get_elapsed_ticks() >= thc_delay) {

        if(!(thc.enabled = !(job.thc_disabled || mode == Plasma_ModeArcOK)))
            stateHandler = state_arc_monitor;
        else if(mode == Plasma_ModeUpDown) {
            step_count = 0;
            stateHandler = state_thc_adjust;
        } else {
            pidf_reset(&pid);
            st2_set_position(z_motor, 0LL);
            if(!isnanf(job.cut_voltage))
                set_target_voltage(job.cut_voltage);
            else
                set_target_voltage(parc_voltage.get_value(&parc_voltage) * plasma.arc_voltage_scale - plasma.arc_voltage_offset);
            stateHandler = state_vad_lock;
            stateHandler();
        }
    }
}

static void state_arc_monitor (void)
{
    if(!(thc.arc_ok = arc_ok.get_value(&arc_ok) == 1.0f))
        pause_on_error();
}

static void state_thc_adjust (void)
{
    if((thc.arc_ok = arc_ok.get_value(&arc_ok) == 1.0f)) {
        if(updown_enabled) {
            thc.up = thc.report_up = cutter_up.get_value(&cutter_up) == 1.0f;
            thc.down = thc.report_down = cutter_down.get_value(&cutter_down) == 1.0f;
            if(thc.up != thc.down) {
                if(thc.up) {
                    step_count++;
                    hal.stepper.output_step((axes_signals_t){Z_AXIS_BIT}, (axes_signals_t){0});
                } else {
                    step_count--;
                    hal.stepper.output_step((axes_signals_t){Z_AXIS_BIT}, (axes_signals_t){Z_AXIS_BIT});
                }
            }
        }
    } else
        pause_on_error();
}

static void state_vad_lock (void)
{
    arc_voltage = parc_voltage.get_value(&parc_voltage) * plasma.arc_voltage_scale - plasma.arc_voltage_offset;

    if((thc.active = fr_actual >= fr_thr_99))
        stateHandler = state_thc_pid;
}

static void state_thc_pid (void)
{
    static float v;

    if(!(thc.active = fr_actual >= fr_thr_vad)) {
        stateHandler = state_vad_lock;
        return;
    }

    if((thc.arc_ok = arc_ok.get_value(&arc_ok)) == 1.0f) {

        if(v_count == 0)
            v = 0.0f;

        arc_voltage = parc_voltage.get_value(&parc_voltage) * plasma.arc_voltage_scale - plasma.arc_voltage_offset;
        v += arc_voltage;
        if(++v_count == THC_SAMPLE_AVG) {

            arc_voltage = v / (float)THC_SAMPLE_AVG;
            v_count = 0;

            if(arc_voltage < arc_voltage_low || arc_voltage > arc_voltage_high) {
                float err = pidf(&pid, arc_vref, arc_voltage, 1.0f);
                if(!st2_motor_running(z_motor)) {
/*
                    char buf[50];
                    strcpy(buf, ftoa(arc_vref, 1));
                    strcat(buf, ",");
                    strcat(buf, ftoa(arc_voltage, 1));
                    strcat(buf, ",");
                    strcat(buf, ftoa(err, 1));
                    report_message(buf, Message_Info);
*/
                    st2_motor_move(z_motor, -err * plasma.arc_height_per_volt, settings.axis[Z_AXIS].max_rate, Stepper2_mm);
                }
            }
        }
/*
        if(arc_voltage >= arc_voltage_high)
            hal.stepper.output_step((axes_signals_t){Z_AXIS_BIT}, (axes_signals_t){Z_AXIS_BIT});
        else if(arc_voltage <= arc_voltage_low)
            hal.stepper.output_step((axes_signals_t){Z_AXIS_BIT}, (axes_signals_t){0});
*/

    } else
        pause_on_error();
}

/* end THC state machine */

static void exec_state_machine (void *data)
{
    stateHandler();

    if(set_feed_override) {
        set_feed_override = false;
        plan_feed_override(feed_override, sys.override.rapid_rate);
    }

    /*
        if(or) {
            or = false;
            hal.stream.write("[MSG:FR ");
            hal.stream.write(ftoa(fr_pgm, 1));
            hal.stream.write(" ");
            hal.stream.write(ftoa(fr_actual, 1));
            hal.stream.write("]" ASCII_EOL);
        }
    */
}

static void onExecuteRealtime (uint_fast16_t state)
{
    if(stateHandler == state_thc_pid)
        st2_motor_run(z_motor);

    on_execute_realtime(state);
}

static void reset (void)
{
    thc.value = 0;
    stateHandler = state_idle;

    st2_motor_stop(z_motor);

    driver_reset();
}

// Start or stop arc
static void arcSetState (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if(driver_reset == NULL) {
        spindle_set_state_(spindle, state, rpm);
        if(state.on)
            report_message("Plasma mode not available!", Message_Warning);
        return;
    }

    if(!state.on) {

        if(!isnanf(job.pause_at_end))
            delay_sec(job.pause_at_end, DelayMode_Dwell);
        spindle_set_state_(spindle, state, rpm);
        thc.torch_on = thc.arc_ok = thc.enabled = Off;
        stateHandler = state_idle;

    } else {

        uint_fast8_t retries = plasma.arc_retries;

        if(job.pierce_height != 0.0f)
            moveto(job.pierce_height);

        do {
            spindle_set_state_(spindle, state, rpm);
            thc.torch_on = On;
            report_message("arc on", Message_Plain);
            if((thc.arc_ok = hal.port.wait_on_input(Port_Digital, port_arc_ok, WaitMode_High, plasma.arc_fail_timeout) != -1)) {
                report_message("arc ok", Message_Plain);
                delay_sec(job.pierce_delay, DelayMode_Dwell);
                if(!isnanf(job.cut_height))
                    moveto(job.cut_height);
                retries = 0;
                thc_delay = hal.get_elapsed_ticks() + (uint32_t)ceilf(1000.0f * plasma.thc_delay); // handle overflow!
                stateHandler = state_thc_delay;
            } else if(!(--retries)) {
                thc.torch_on = Off;
                report_message("arc failed", Message_Warning);
                spindle_set_state_(spindle, (spindle_state_t){0}, 0.0f);
                pause_on_error(); // output message and enter similar state as tool change state (allow jogging before resume)
            } else {
                thc.torch_on = Off;
                report_message("arc delay", Message_Plain);
                spindle_set_state_(spindle, (spindle_state_t){0}, 0.0f);
                delay_sec(plasma.arc_retry_delay, DelayMode_Dwell);
            }
        } while(retries);
    }
}

static void stepperPulseStart (stepper_t *stepper)
{
//    static volatile bool get_rates = false;

    if(stepper->new_block) {
//        get_rates = true;
        fr_pgm = stepper->exec_block->programmed_rate * 0.01f * sys.override.feed_rate;
        fr_thr_99 = fr_pgm * 0.99f;
        fr_thr_vad = fr_pgm * 0.01f * (float)plasma.vad_threshold;
        segment_id = 0;
    }

    if(stepper->exec_segment->id != segment_id) {
        segment_id = stepper->exec_segment->id;
        fr_actual = stepper->exec_segment->current_rate;
    }

    stepper_pulse_start(stepper);
}

// Trap cycle start commands and redirect to foreground process
// by temporarily claiming the HAL execute_realtime entry point
// in order to execute probing and spindle/coolant change.
// TODO: move to state machine with own EXEC_ bit?
/*
ISR_CODE static void trap_control_interrupts (control_signals_t signals)
{
    if(signals.value)
        control_interrupt_callback(signals);
}
*/

// Convert control signals bits to string representation.
// NOTE: returns pointer to null terminator!
static inline char *signals_tostring (char *buf, thc_signals_t signals)
{
    static const char signals_map[] = "ATEOFBR  VHUD    ";

    char *map = (char *)signals_map;

    if(signals.bits)
        *buf++ = ',';

    while(signals.bits) {

        if(signals.bits & 0x01) {
            switch(*map) {

                case ' ':
                    break;

                default:
                    *buf++ = *map;
                    break;
            }
        }

        map++;
        signals.bits >>= 1;
    }

    *buf = '\0';

    return buf;
}


static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    static char buf[24];

    strcpy(buf, "|THC:");
    strcat(buf, ftoa(arc_voltage, 1));
    signals_tostring(strchr(buf, '\0'), thc);
    stream_write(buf);

    thc.report_up = thc.report_down = Off;

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    spindle_set_state_ = spindle->set_state;

    spindle->set_state = arcSetState;
    // TODO: only change caps if PWM spindle active?
    spindle->cap.at_speed = Off;
//??    spindle->cap.laser = Off;
    spindle->cap.torch = On;

    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

static void plasma_setup (settings_t *settings, settings_changed_flags_t changed)
{
    settings_changed(settings, changed);

    if(!driver_reset) {

        driver_reset = hal.driver_reset;
        hal.driver_reset = reset;

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = onSpindleSelected;

        on_realtime_report = grbl.on_realtime_report;
        grbl.on_realtime_report = onRealtimeReport;

        if(st2_motor_poll(z_motor)) {
            on_execute_realtime = grbl.on_execute_realtime;
            grbl.on_execute_realtime = onExecuteRealtime;
        }

        task_add_systick(exec_state_machine, NULL);
    }

    // Reclaim entry points that may have been changed on settings change.

    if(hal.stepper.pulse_start != stepperPulseStart) {
        stepper_pulse_start = hal.stepper.pulse_start;
        hal.stepper.pulse_start = stepperPulseStart;
    }
}

PROGMEM static const setting_group_detail_t plasma_groups[] = {
    { Group_Root, Group_Plasma, "Plasma" },
};

static bool is_setting_available (const setting_detail_t *setting, uint_fast16_t offset)
{
    bool ok = false;

    switch(setting->id) {

        case Setting_THC_CutterDownPort:
        case Setting_THC_CutterUpPort:
            ok = n_din >= 3;
            break;

        case Setting_Arc_VoltagePort:
            ok = n_ain >= 1;
            break;

        case Setting_THC_VADThreshold:
            ok = init_ok;
            break;

        default:
            ok = init_ok && plasma.mode == Plasma_ModeVoltage;
            break;
    }

    return ok;
}

static status_code_t set_port (setting_id_t setting, float value)
{
    status_code_t status;

    if((status = isintf(value) ? Status_OK : Status_BadNumberFormat) == Status_OK)
      switch(setting) {

        case Setting_Arc_VoltagePort:
            plasma.port_arc_voltage = value < 0.0f ? 255 : (uint8_t)value;
            break;

        case Setting_Arc_OkPort:
            plasma.port_arc_ok = value < 0.0f ? 255 : (uint8_t)value;
            break;

        case Setting_THC_CutterDownPort:
            plasma.port_cutter_down = value < 0.0f ? 255 : (uint8_t)value;
            break;

        case Setting_THC_CutterUpPort:
            plasma.port_cutter_up = value < 0.0f ? 255 : (uint8_t)value;
            break;

        default:
            break;
    }

    return status;
}

static float get_port (setting_id_t setting)
{
    float value = 0.0f;

    switch(setting) {

        case Setting_Arc_VoltagePort:
            value = plasma.port_arc_voltage >= n_ain ? -1.0f : (float)plasma.port_arc_voltage;
            break;

        case Setting_Arc_OkPort:
            value = plasma.port_arc_ok >= n_din ? -1.0f : (float)plasma.port_arc_ok;
            break;

        case Setting_THC_CutterDownPort:
            value = plasma.port_cutter_down >= n_din ? -1.0f : (float)plasma.port_cutter_down;
            break;

        case Setting_THC_CutterUpPort:
            value = plasma.port_cutter_up >= n_din ? -1.0f : (float)plasma.port_cutter_up;
            break;

        default:
            break;
    }

    return value;
}

PROGMEM static const setting_detail_t plasma_settings[] = {
    { Setting_THC_Mode, Group_Plasma, "Plasma mode", NULL, Format_RadioButtons, thc_modes, NULL, NULL, Setting_NonCore, &plasma.mode, NULL, NULL, { .reboot_required = On } },
    { Setting_THC_Delay, Group_Plasma, "Plasma THC delay", "s", Format_Decimal, "#0.0", NULL, NULL, Setting_NonCore, &plasma.thc_delay, NULL, NULL },
    { Setting_THC_Threshold, Group_Plasma, "Plasma THC threshold", "V", Format_Decimal, "#0.00", NULL, NULL, Setting_NonCore, &plasma.thc_threshold, NULL, is_setting_available },
    { Setting_THC_PGain, Group_Plasma, "Plasma THC P-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.pid.p_gain, NULL, is_setting_available },
    { Setting_THC_IGain, Group_Plasma, "Plasma THC I-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.pid.i_gain, NULL, is_setting_available },
    { Setting_THC_DGain, Group_Plasma, "Plasma THC D-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.pid.d_gain, NULL, is_setting_available },
    { Setting_THC_VADThreshold, Group_Plasma, "Plasma THC VAD threshold", "percent", Format_Integer, "##0", "0", "100", Setting_NonCore, &plasma.vad_threshold, NULL, is_setting_available },
    { Setting_THC_VoidOverride, Group_Plasma, "Plasma THC Void override", "percent", Format_Integer, "##0", "0", "100", Setting_NonCore, &plasma.thc_override, NULL, is_setting_available },
    { Setting_Arc_FailTimeout, Group_Plasma, "Plasma Arc fail timeout", "seconds", Format_Decimal, "#0.0", NULL, NULL, Setting_NonCore, &plasma.arc_fail_timeout, NULL, NULL },
    { Setting_Arc_RetryDelay, Group_Plasma, "Plasma Arc retry delay", "seconds", Format_Decimal, "#0.0", NULL, NULL, Setting_NonCore, &plasma.arc_retry_delay, NULL, NULL },
    { Setting_Arc_MaxRetries, Group_Plasma, "Plasma Arc max retries", NULL, Format_Int8, "#0", NULL, NULL, Setting_NonCore, &plasma.arc_retries, NULL, NULL },
    { Setting_Arc_VoltageScale, Group_Plasma, "Plasma Arc voltage scale", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.arc_voltage_scale, NULL, is_setting_available },
    { Setting_Arc_VoltageOffset, Group_Plasma, "Plasma Arc voltage offset", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.arc_voltage_offset, NULL, is_setting_available },
    { Setting_Arc_HeightPerVolt, Group_Plasma, "Plasma Arc height per volt", "mm", Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.arc_height_per_volt, NULL, is_setting_available },
    { Setting_Arc_OkHighVoltage, Group_Plasma, "Plasma Arc ok high volts", "V", Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.arc_high_low_voltage, NULL, is_setting_available },
    { Setting_Arc_OkLowVoltage, Group_Plasma, "Plasma Arc ok low volts", "V", Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.arc_ok_low_voltage, NULL, is_setting_available },
    { Setting_Arc_VoltagePort, Group_AuxPorts, "Arc voltage port", NULL, Format_Decimal, "-#0", "-1", max_aport, Setting_NonCoreFn, set_port, get_port, is_setting_available, { .reboot_required = On } },
    { Setting_Arc_OkPort, Group_AuxPorts, "Arc ok port", NULL, Format_Decimal, "-#0", "-1", max_dport, Setting_NonCoreFn, set_port, get_port, NULL, { .reboot_required = On } },
    { Setting_THC_CutterDownPort, Group_AuxPorts, "Cutter down port", NULL, Format_Decimal, "-#0", "-1", max_dport, Setting_NonCoreFn, set_port, get_port, is_setting_available, { .reboot_required = On } },
    { Setting_THC_CutterUpPort, Group_AuxPorts, "Cutter up port", NULL, Format_Decimal, "-#0", "-1", max_dport, Setting_NonCoreFn, set_port, get_port, is_setting_available, { .reboot_required = On } },
    { Setting_THC_Options, Group_Plasma, "Plasma options", NULL, Format_Bitfield, "Virtual ports,Sync Z position", NULL, NULL, Setting_NonCore, &plasma.option.flags, NULL, NULL, { .reboot_required = On } },
};

PROGMEM static const setting_descr_t plasma_settings_descr[] = {
    { Setting_THC_Mode, "" },
    { Setting_THC_Delay, "Delay from cut start until THC activates." },
    { Setting_THC_Threshold, "Variation from target voltage for THC to correct height." },
    { Setting_THC_PGain, "" },
    { Setting_THC_IGain, "" },
    { Setting_THC_DGain, "" },
    { Setting_THC_VADThreshold, "Percentage of Cut Feed Rate velocity needs to fall below to lock THC." },
    { Setting_THC_VoidOverride, "Higher values need greater voltage change to lock THC." },
    { Setting_Arc_FailTimeout, "The amount of time to wait from torch on until a failure if arc is not detected." },
    { Setting_Arc_RetryDelay, "The time between an arc failure and another arc start attempt." },
    { Setting_Arc_MaxRetries, "The number of attempts at starting an arc." },
    { Setting_Arc_VoltageScale, "The value required to scale the arc voltage input to display the correct arc voltage." },
    { Setting_Arc_VoltageOffset, "The value required to display zero volts when there is zero arc voltage input.\\n"
                                 "For initial setup multiply the arc voltage out value by -1 and enter that for Voltage Offset."
    },
    { Setting_Arc_HeightPerVolt, "The distance the torch would need to move to change the arc voltage by one volt.\\n"
//                                 "Used for manual height change only."
    },
    { Setting_Arc_OkHighVoltage, "High voltage threshold for Arc OK." },
    { Setting_Arc_OkLowVoltage, "Low voltage threshold for Arc OK." },
    { Setting_Arc_VoltagePort, "Aux port number to use for arc voltage. Set to -1 to disable." },
    { Setting_Arc_OkPort, "Aux port number to use for arc ok signal. Set to -1 to disable." },
    { Setting_THC_CutterDownPort, "Aux port number to use for cutter down signal. Set to -1 to disable." },
    { Setting_THC_CutterUpPort, "Aux port number to use for cutter up signal. Set to -1 to disable." },
    { Setting_THC_Options, "" }
};

static void plasma_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plasma, sizeof(plasma_settings_t), true);
}

static void plasma_settings_restore (void)
{
    plasma.mode = mode;
    plasma.option.flags = 0;
    plasma.thc_delay = 3.0f;
    plasma.thc_threshold = 1.0f;
    plasma.thc_override = 100;
    plasma.vad_threshold = 90;
    plasma.pause_at_end = 0.0f;
    plasma.pierce_delay = 0.0f;
    plasma.pierce_height = 1.0f;
    plasma.arc_fail_timeout = 3.0f;
    plasma.arc_retries = 3;
    plasma.arc_retry_delay = 3.0f;
    plasma.arc_fail_timeout = 3.0f;
    plasma.arc_voltage_scale = 1.0f;
    plasma.arc_voltage_offset = 0.0f;
    plasma.arc_height_per_volt = 0.1f;
    plasma.arc_high_low_voltage = 150.0;
    plasma.arc_ok_low_voltage = 100.0f;
    plasma.pid.p_gain = 1.0f;
    plasma.pid.i_gain = 0.0f;
    plasma.pid.d_gain = 0.0f;
    plasma.port_arc_voltage = ioport_find_free(Port_Analog, Port_Input, (pin_cap_t){ .claimable = On }, "Arc voltage");
    plasma.port_arc_ok = ioport_find_free(Port_Digital, Port_Input, (pin_cap_t){ .claimable = On }, "Arc ok");
    plasma.port_cutter_down = updown_enabled && plasma.port_arc_ok >= 1 ? plasma.port_arc_ok - 1 : 255;
    plasma.port_cutter_up = updown_enabled && plasma.port_arc_ok >= 2 ? plasma.port_arc_ok - 2 : 255;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plasma, sizeof(plasma_settings_t), true);
}

static bool plasma_claim_digital_in (xbar_t *target, uint8_t port, const char *description)
{
    xbar_t *p;
    bool ok = false;

    if(port != 255 && (p = ioport_get_info(Port_Digital, Port_Input, port)) && p->get_value && !p->mode.claimed) {
        memcpy(target, p, sizeof(xbar_t));
        if((ok = ioport_claim(Port_Digital, Port_Input, &port, description)) && target == &arc_ok)
            port_arc_ok = port;
    }

    return ok;
}

static void plasma_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&plasma, nvs_address, sizeof(plasma_settings_t), true) != NVS_TransferResult_OK) {
        plasma.port_arc_ok = plasma.port_cutter_down = plasma.port_cutter_up = 255;
        plasma_settings_restore();
    }

    if((mode = plasma.mode) == Plasma_ModeOff)
        return;

    if(!(init_ok = mode != Plasma_ModeVoltage)) {

        if(plasma.port_arc_voltage != 255) {

            xbar_t *p;

            if((p = ioport_get_info(Port_Analog, Port_Input, port_arc_voltage)) && p->get_value && !p->mode.claimed) {
                memcpy(&parc_voltage, p, sizeof(xbar_t));
                init_ok = ioport_claim(Port_Analog, Port_Input, &port_arc_voltage, "Arc voltage");
            }
        }

        if(!init_ok) {
            init_ok = true;
            mode = Plasma_ModeUpDown;
        }
    }

    init_ok = init_ok && plasma_claim_digital_in(&arc_ok, plasma.port_arc_ok, "Arc ok");

    if(init_ok && mode == Plasma_ModeUpDown) {
        if(!(plasma_claim_digital_in(&cutter_down, plasma.port_cutter_down, "Cutter down") &&
              plasma_claim_digital_in(&cutter_up, plasma.port_cutter_up, "Cutter up")))
            mode = Plasma_ModeArcOK;
    }

    if(init_ok) {

        set_job_params(NULL);

        updown_enabled = mode == Plasma_ModeUpDown;

        settings_changed = hal.settings_changed;
        hal.settings_changed = plasma_setup;

        if(plasma.mode != mode)
            task_run_on_startup(report_warning, "Plasma mode changed due to lack of inputs!");

        if(plasma.option.virtual_ports)
            task_run_on_startup(add_virtual_ports, NULL);
    } else
        task_run_on_startup(report_warning, "Plasma mode failed to initialize!");
}

static void on_settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    pidf_init(&pid, &plasma.pid);
}

static status_code_t matlist (sys_state_t state, char *args)
{
    plasma_enumerate_materials(ml_enumerate, NULL);

    return Status_OK;
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {

        plasma_mode_t i = Plasma_ModeOff;
        char buf[30] = "PLASMA (", *s1 = &buf[8], *s2 = thc_modes, c;

        while((c = *s2++)) {
            if(i == mode) {
                if(c == ',')
                    break;
                *s1++ = c;
            } else if(c == ',')
                i++;
        }
        *s1++ = ')';
        *s1 = '\0';

        report_plugin(buf, "0.23");

    } else if(mode != Plasma_ModeOff)
        hal.stream.write(",THC");
}

void plasma_init (void)
{
    static setting_details_t setting_details = {
        .groups = plasma_groups,
        .n_groups = sizeof(plasma_groups) / sizeof(setting_group_detail_t),
        .settings = plasma_settings,
        .n_settings = sizeof(plasma_settings) / sizeof(setting_detail_t),
        .descriptions = plasma_settings_descr,
        .n_descriptions = sizeof(plasma_settings_descr) / sizeof(setting_descr_t),
        .save = plasma_settings_save,
        .load = plasma_settings_load,
        .restore = plasma_settings_restore,
        .on_changed = on_settings_changed
    };

    static const sys_command_t thc_command_list[] = {
        {"EM", matlist, { .allow_blocking = On, .noargs = On }, { .str = "outputs plasma materials list" } }
    };

    static sys_commands_t thc_commands = {
        .n_commands = sizeof(thc_command_list) / sizeof(sys_command_t),
        .commands = thc_command_list
    };

    bool ok;

    if((ok = !!hal.stepper.output_step && ioport_can_claim_explicit())) {
        n_ain = ioports_available(Port_Analog, Port_Input);
        n_din = ioports_available(Port_Digital, Port_Input);
        ok = (n_ain >= 1 && n_din >= 1) || (n_din >= 3);
    }

    if(ok) {
        updown_enabled = n_ain == 0;
        ok = (nvs_address = nvs_alloc(sizeof(plasma_settings_t)));
    }

    if(ok && (z_motor = st2_motor_init(Z_AXIS, false)) != NULL) {

        if(n_ain)
            strcpy(max_aport, uitoa(n_ain - 1));
        strcpy(max_dport, uitoa(n_din - 1));

        settings_register(&setting_details);
        system_register_commands(&thc_commands);

        memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

        grbl.user_mcode.check = mcode_check;
        grbl.user_mcode.validate = mcode_validate;
        grbl.user_mcode.execute = mcode_execute;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        on_gcode_comment = grbl.on_gcode_comment;
        grbl.on_gcode_comment = onGcodeComment;

/*
        control_interrupt_callback = hal.control_interrupt_callback;
        hal.control_interrupt_callback = trap_control_interrupts;
*/

        if(n_ain == 0)
            setting_remove_elements(Setting_THC_Mode, 0b1101);

        if(n_din < 3)
            setting_remove_elements(Setting_THC_Mode, 0b1011);

        // Load materials
        sheetcam_init();
        linuxcnc_init();

    } else
        task_run_on_startup(report_warning, "Plasma mode failed to initialize!");
}

#endif // PLASMA_ENABLE
