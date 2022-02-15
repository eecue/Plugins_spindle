/*

  yalang.c - Yalang VFD spindle support

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io, Dave Bullock

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#if YALANG_ENABLE

#include <math.h>
#include <string.h>

#ifdef ARDUINO
#include "../grbl/hal.h"
#include "../grbl/protocol.h"
#include "../grbl/state_machine.h"
#include "../grbl/report.h"
#else
#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"
#include "grbl/report.h"
#endif

#include "modbus.h"

#ifdef SPINDLE_PWM_DIRECT
#error "Uncomment SPINDLE_RPM_CONTROLLED in grbl/config.h to add Yalang spindle support!"
#endif

#ifndef VFD_ADDRESS
#define VFD_ADDRESS 0x01
#endif

typedef enum {
    VFD_Idle = 0,
    VFD_GetRPM,
    VFD_SetRPM,
    VFD_GetStatus,
    VFD_SetStatus
} vfd_response_t;

static float rpm_programmed = -1.0f, rpm_low_limit = 0.0f, rpm_high_limit = 0.0f;
static spindle_state_t vfd_state = {0};
static spindle_data_t spindle_data = {0};
static settings_changed_ptr settings_changed;
static on_report_options_ptr on_report_options;
static driver_reset_ptr driver_reset;

static void rx_packet (modbus_message_t *msg);
static void rx_exception (uint8_t code, void *context);

static const modbus_callbacks_t callbacks = {
    .on_rx_packet = rx_packet,
    .on_rx_exception = rx_exception
};

static void spindleSetRPM (float rpm, bool block)
{

    if (rpm != rpm_programmed) {


        uint16_t data = (uint32_t)(rpm) * 10000UL ;

        modbus_message_t rpm_cmd = {
            .context = (void *)VFD_SetRPM,
            .crc_check = false,
            .adu[0] = VFD_ADDRESS,
            .adu[1] = ModBus_WriteRegister,
            .adu[2] = 0x20,
            .adu[3] = 0x01,
            .adu[4] = data >> 8,
            .adu[5] = data & 0xFF,
            .tx_length = 8,
            .rx_length = 8
        };


        vfd_state.at_speed = false;

        modbus_send(&rpm_cmd, &callbacks, block);

        if(settings.spindle.at_speed_tolerance > 0.0f) {
            rpm_low_limit = rpm / (1.0f + settings.spindle.at_speed_tolerance);
            rpm_high_limit = rpm * (1.0f + settings.spindle.at_speed_tolerance);
        }
        rpm_programmed = rpm;
    }
}

static void spindleUpdateRPM (float rpm)
{
    spindleSetRPM(rpm, false);
}

// Start or stop spindle
static void spindleSetState (spindle_state_t state, float rpm)
{

    modbus_message_t mode_cmd = {
        .context = (void *)VFD_SetStatus,
        .crc_check = false,
        .adu[0] = VFD_ADDRESS,
        .adu[1] = ModBus_WriteRegister,
        .adu[2] = 0x20,
        .adu[5] = (!state.on || rpm == 0.0f) ? 0x01 : (state.ccw ? 0x22 : 0x12),
        .tx_length = 8,
        .rx_length = 8
    };

    if(vfd_state.ccw != state.ccw)
        rpm_programmed = 0.0f;

    vfd_state.on = state.on;
    vfd_state.ccw = state.ccw;

    if(modbus_send(&mode_cmd, &callbacks, true))
        spindleSetRPM(rpm, true);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{


    modbus_message_t mode_cmd = {
        .context = (void *)VFD_GetRPM,
        .crc_check = false,
        .adu[0] = VFD_ADDRESS,
        .adu[1] = ModBus_ReadHoldingRegisters,
        .adu[2] = 0x20,
        .adu[3] = 0x0B,
        .tx_length = 8,
        .rx_length = 8
    };


    modbus_send(&mode_cmd, &callbacks, false); // TODO: add flag for not raising alarm?

    return vfd_state; // return previous state as we do not want to wait for the response
}

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    return &spindle_data;
}

static void rx_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {

        switch((vfd_response_t)msg->context) {

            case VFD_GetRPM:
                spindle_data.rpm = (float)((msg->adu[4] << 8) | msg->adu[5]);
                vfd_state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (spindle_data.rpm >= rpm_low_limit && spindle_data.rpm <= rpm_high_limit);
                break;

            default:
                break;
        }
    }
}

static void raise_alarm (uint_fast16_t state)
{
    system_raise_alarm(Alarm_Spindle);
}

static void rx_exception (uint8_t code, void *context)
{
    // Alarm needs to be raised directly to correctly handle an error during reset (the rt command queue is
    // emptied on a warm reset). Exception is during cold start, where alarms need to be queued.
    if(sys.cold_start)
        protocol_enqueue_rt_command(raise_alarm);
    else
        system_raise_alarm(Alarm_Spindle);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[PLUGIN:YALANG VFD v0.01]" ASCII_EOL);
    }
}

static void yalang_reset (void)
{
    driver_reset();
}

static void yalang_settings_changed (settings_t *settings)
{
    static bool init_ok = false, vfd_active = false;
    static driver_cap_t driver_cap;
    static spindle_ptrs_t spindle_org;

    if(settings_changed)
        settings_changed(settings);

    if(!modbus_isup())
        return;

    if(hal.driver_cap.dual_spindle && settings->mode == Mode_Laser) {
        if(vfd_active) {

            vfd_active = false;

            hal.spindle.set_state((spindle_state_t){0}, 0.0f);

            hal.driver_cap = driver_cap;
            memcpy(&hal.spindle, &spindle_org, sizeof(spindle_ptrs_t));
        }
    } else {

        if(hal.spindle.set_state != spindleSetState) {

            vfd_active = true;

            if(spindle_org.set_state == NULL) {
                driver_cap = hal.driver_cap;
                memcpy(&spindle_org, &hal.spindle, sizeof(spindle_ptrs_t));
            }

            if(spindle_org.set_state)
                spindle_org.set_state((spindle_state_t){0}, 0.0f);

            hal.spindle.set_state = spindleSetState;
            hal.spindle.get_state = spindleGetState;
            hal.spindle.update_rpm = spindleUpdateRPM;
            hal.spindle.reset_data = NULL;

            hal.driver_cap.variable_spindle = On;
            hal.driver_cap.spindle_at_speed = On;
            hal.driver_cap.spindle_dir = On;
        }

        if(settings->spindle.ppr == 0)
            hal.spindle.get_data = spindleGetData;

        if(!init_ok) {
            init_ok = true;
        }
    }
}

void yalang_init (void)
{
    if(modbus_enabled()) {
        settings_changed = hal.settings_changed;
        hal.settings_changed = yalang_settings_changed;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        driver_reset = hal.driver_reset;
        hal.driver_reset = yalang_reset;

        if(!hal.driver_cap.dual_spindle)
            yalang_settings_changed(&settings);
    }
}

#endif
