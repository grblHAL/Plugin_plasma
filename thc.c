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

#include "driver.h"

#if PLASMA_ENABLE

#include <math.h>
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

#define THC_SAMPLE_AVG 5

// Digital ports
#define PLASMA_THC_DISABLE_PORT   2 // output
#define PLASMA_TORCH_DISABLE_PORT 3 // output
// Analog ports
#define PLASMA_FEED_OVERRIDE_PORT 3

typedef enum {
    Plasma_ModeOff = 0,
    Plasma_ModeVoltage = 1,
    Plasma_ModeUpDown = 2
} plasma_mode_t;

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
static void state_vad_lock (void);

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
static char thc_modes[] = "Off,Voltage,Up/down";
static plasma_settings_t plasma;
static st2_motor_t *z_motor;
static void (*volatile stateHandler)(void) = state_idle;
static xbar_t arc_ok, cutter_down, cutter_up, parc_voltage;

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

// --- Virtual ports start

static io_port_t rport = {0};
static io_ports_data_t digital, analog;

typedef struct {
    uint8_t pin_id;
    pin_info_ptr pin_info;
    void *data;
    bool aux_dout0;
    bool aux_dout1;
    bool aux_aout0;
    bool aux_aout1;
    bool aux_aout2;
} pin_stat_t;

static void enum_trap (xbar_t *pin, void *data)
{
    ((pin_stat_t *)data)->pin_id = max(((pin_stat_t *)data)->pin_id, pin->id);

    if(pin->function == Output_Aux0)
        ((pin_stat_t *)data)->aux_dout0 = true;
    else if(pin->function == Output_Aux1)
        ((pin_stat_t *)data)->aux_dout1 = true;
    else if(pin->function == Output_Analog_Aux0)
        ((pin_stat_t *)data)->aux_aout0 = true;
    else if(pin->function == Output_Analog_Aux1)
        ((pin_stat_t *)data)->aux_aout1 = true;
    else if(pin->function == Output_Analog_Aux2)
        ((pin_stat_t *)data)->aux_aout2 = true;

    if((pin->function == Output_Aux2 || pin->function == Output_Aux3 || pin->function == Output_Analog_Aux3))
        pin->description = "Shadowed by THC";

    if(((pin_stat_t *)data)->pin_info)
        ((pin_stat_t *)data)->pin_info(pin, ((pin_stat_t *)data)->data);
}

static void enumeratePins (bool low_level, pin_info_ptr pin_info, void *data)
{
    pin_stat_t pin_stat = {
        .pin_id = 0,
        .pin_info = pin_info,
        .data = data
    };

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

static void digital_out (uint8_t portnum, bool on)
{
    if(portnum == PLASMA_THC_DISABLE_PORT) {
        if(!(thc.enabled = !on))
            stateHandler = state_idle;
        else if(thc.arc_ok)
            stateHandler = plasma.mode == Plasma_ModeUpDown ? state_thc_adjust : state_thc_pid;
    } else if(portnum == PLASMA_TORCH_DISABLE_PORT) {
        // PLASMA_TORCH_DISABLE_PORT:
        // TODO
    } else if(rport.digital_out)
        rport.digital_out(portnum, on);
}

static bool analog_out (uint8_t portnum, float value)
{
    if(portnum == PLASMA_FEED_OVERRIDE_PORT) {
        // Let the foreground process handle this
        set_feed_override = true;
        feed_override = (uint_fast8_t)value;
        if(feed_override < 10 || feed_override > 100)
            feed_override = 100;
    } else
        return rport.analog_out ? rport.analog_out(portnum, value) : false;

    return true;
}

static xbar_t *get_pin_info (io_port_type_t type, io_port_direction_t dir, uint8_t port)
{
    static xbar_t pin;

    xbar_t *info = rport.get_pin_info(type, dir, port);

    if(info == NULL && dir == Port_Output) {

        if(type == Port_Digital && port >= digital.out.n_start && port < digital.out.n_start + digital.out.n_ports) {

            memset(&pin, 0, sizeof(xbar_t));

            pin.id = pin.pin = port;
            pin.cap.output = On;
            pin.cap.claimable = Off;
            pin.mode.output = On;
            pin.description = port == digital.out.n_start ? "THC enable/disable" : "THC torch control";

            return &pin;
        }

        if(type == Port_Analog && port >= analog.out.n_start && port < analog.out.n_start + analog.out.n_ports) {

            memset(&pin, 0, sizeof(xbar_t));

            pin.id = pin.pin = port;
            pin.cap.output = pin.cap.analog = On;
            pin.cap.claimable = Off;
            pin.mode.output = pin.cap.analog = On;
            pin.description = "THC feed override";

            return &pin;
        }
    }

    return info;
}

static void add_virtual_ports (void *data)
{
    uint8_t aux_dout = 2, aux_aout = 1;
    pin_stat_t pin_stat = {};

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

    if(ioports_add(&digital, Port_Digital, 0, aux_dout) && ioports_add(&analog, Port_Analog, 0, aux_aout))  {

        memcpy(&rport, &hal.port, sizeof(io_port_t));

        hal.port.digital_out = digital_out;
        hal.port.analog_out = analog_out;
        hal.port.get_pin_info = get_pin_info;

        enumerate_pins = hal.enumerate_pins;
        hal.enumerate_pins = enumeratePins;
    }
}

// --- Virtual ports end

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
    if(plasma.mode == Plasma_ModeVoltage)
        arc_voltage = parc_voltage.get_value(&parc_voltage) * plasma.arc_voltage_scale - plasma.arc_voltage_offset;

    if(plasma.option.sync_pos && state_get() == STATE_IDLE) {

        if(plasma.mode != Plasma_ModeUpDown)
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
        thc.enabled = On;
        if(plasma.mode == Plasma_ModeUpDown) {
            step_count = 0;
            stateHandler = state_thc_adjust;
        } else {
            pidf_reset(&pid);
            st2_set_position(z_motor, 0LL);
            set_target_voltage(parc_voltage.get_value(&parc_voltage) * plasma.arc_voltage_scale - plasma.arc_voltage_offset);
            stateHandler = state_vad_lock;
            stateHandler();
        }
    }
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
        if(plasma.pause_at_end > 0.0f)
            delay_sec(plasma.pause_at_end, DelayMode_Dwell);
        spindle_set_state_(spindle, state, rpm);
        thc.torch_on = thc.arc_ok = thc.enabled = Off;
        stateHandler = state_idle;
    } else {
        uint_fast8_t retries = plasma.arc_retries;
        do {
            spindle_set_state_(spindle, state, rpm);
            thc.torch_on = On;
            report_message("arc on", Message_Plain);
            if((thc.arc_ok = rport.wait_on_input(Port_Digital, port_arc_ok, WaitMode_High, plasma.arc_fail_timeout) != -1)) {
                report_message("arc ok", Message_Plain);
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
/*
    if(thc.value) {
        *append++ = ',';
        if(thc.arc_ok)
            *append++ = 'A';
        if(thc.enabled)
            *append++ = 'E';
        if(thc.active)
            *append++ = 'R';
        if(thc.torch_on)
            *append++ = 'T';
        if(thc.ohmic_probe)
            *append++ = 'O';
        if(thc.velocity_lock)
            *append++ = 'V';
        if(thc.void_lock)
            *append++ = 'H';
        if(thc.report_down)
            *append++ = 'D';
        if(thc.report_up)
            *append++ = 'U';
        thc.report_up = thc.report_down = Off;
    }
    *append = '\0';
*/

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

static void onSpindleSelected (spindle_ptrs_t *spindle)
{
    spindle_set_state_ = spindle->set_state;

    spindle->set_state = arcSetState;
    spindle->cap.at_speed = Off; // TODO: only disable if PWM spindle active?

    if(on_spindle_selected)
        on_spindle_selected(spindle);
}

static void plasma_setup (settings_t *settings, settings_changed_flags_t changed)
{
    settings_changed(settings, changed);

    if(!driver_reset) {

        driver_reset = hal.driver_reset;
        hal.driver_reset = reset;

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
    { Setting_THC_Mode, Group_Plasma, "Plasma mode", NULL, Format_RadioButtons, thc_modes, NULL, NULL, Setting_NonCore, &plasma.mode, NULL, NULL },
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

#ifndef NO_SETTINGS_DESCRIPTIONS

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

#endif

static void plasma_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plasma, sizeof(plasma_settings_t), true);
}

static void plasma_settings_restore (void)
{
    plasma.mode = updown_enabled ? Plasma_ModeUpDown : Plasma_ModeVoltage;
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

    if((p = ioport_get_info(Port_Digital, Port_Input, port)) && p->get_value && !p->mode.claimed) {
        memcpy(target, p, sizeof(xbar_t));
        if((ok = ioport_claim(Port_Digital, Port_Input, &port, description)) && target == &arc_ok)
            port_arc_ok = port;
    }

    return ok;
}

static void plasma_settings_load (void)
{
    bool init_ok;

    if(hal.nvs.memcpy_from_nvs((uint8_t *)&plasma, nvs_address, sizeof(plasma_settings_t), true) != NVS_TransferResult_OK) {
        plasma.port_arc_ok = plasma.port_cutter_down = plasma.port_cutter_up = 255;
        plasma_settings_restore();
    }

    port_arc_voltage = plasma.port_arc_voltage;

    if(!(init_ok = n_ain == 0)) {

        xbar_t *p;

        if(port_arc_voltage == 255)
            plasma.port_arc_voltage = port_arc_voltage = 0;

        if((p = ioport_get_info(Port_Analog, Port_Input, port_arc_voltage)) && p->get_value && !p->mode.claimed) {
            memcpy(&parc_voltage, p, sizeof(xbar_t));
            init_ok = ioport_claim(Port_Analog, Port_Input, &port_arc_voltage, "Arc voltage");
        }

//        init_ok = ioport_claim(Port_Analog, Port_Input, &port_arc_voltage, "Arc voltage");
    }

    init_ok = init_ok && plasma_claim_digital_in(&arc_ok, plasma.port_arc_ok, "Arc ok");
    if(init_ok && n_din > 2 && plasma.port_cutter_down != 255) {
        init_ok = plasma_claim_digital_in(&cutter_down, plasma.port_cutter_down, "Cutter down");
        init_ok = init_ok && plasma_claim_digital_in(&cutter_up, plasma.port_cutter_up, "Cutter up");
    }

    if(init_ok) {

        if(n_ain == 0 && plasma.mode == Plasma_ModeVoltage)
            plasma.mode = Plasma_ModeUpDown;

        if(n_din < 3 && plasma.mode == Plasma_ModeUpDown)
            plasma.mode = n_ain >= 1 ? Plasma_ModeVoltage : Plasma_ModeOff;

        updown_enabled = plasma.mode == Plasma_ModeUpDown;

        settings_changed = hal.settings_changed;
        hal.settings_changed = plasma_setup;

        if(plasma.option.virtual_ports)
            protocol_enqueue_foreground_task(add_virtual_ports, NULL);

    } else
        protocol_enqueue_foreground_task(report_warning, "Plasma mode failed to initialize!");
}

static void on_settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    pidf_init(&pid, &plasma.pid);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("PLASMA", "0.16");
    else if(driver_reset) // non-null when successfully enabled
        hal.stream.write(",THC");
}

void plasma_init (void)
{
    static setting_details_t setting_details = {
        .groups = plasma_groups,
        .n_groups = sizeof(plasma_groups) / sizeof(setting_group_detail_t),
        .settings = plasma_settings,
        .n_settings = sizeof(plasma_settings) / sizeof(setting_detail_t),
    #ifndef NO_SETTINGS_DESCRIPTIONS
        .descriptions = plasma_settings_descr,
        .n_descriptions = sizeof(plasma_settings_descr) / sizeof(setting_descr_t),
    #endif
        .save = plasma_settings_save,
        .load = plasma_settings_load,
        .restore = plasma_settings_restore,
        .on_changed = on_settings_changed
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

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        on_spindle_selected = grbl.on_spindle_selected;
        grbl.on_spindle_selected = onSpindleSelected;

/*
        control_interrupt_callback = hal.control_interrupt_callback;
        hal.control_interrupt_callback = trap_control_interrupts;
*/

        if(n_ain == 0)
            setting_remove_elements(Setting_THC_Mode, 0b101);

        if(n_din < 3)
            setting_remove_elements(Setting_THC_Mode, 0b011);

    } else
        protocol_enqueue_foreground_task(report_warning, "Plasma mode failed to initialize!");
}

#endif // PLASMA_ENABLE
