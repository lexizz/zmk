/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/services/bas.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/battery.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/activity.h>
#include <zmk/workqueue.h>

static uint8_t last_state_of_charge = 0;
static uint8_t last_state_without_usb = 0;
static uint8_t charging_start_level = 0;
static int64_t charging_start_time = 0;

uint8_t zmk_battery_state_of_charge(void) { return last_state_of_charge; }

#if DT_HAS_CHOSEN(zmk_battery)
static const struct device *const battery = DEVICE_DT_GET(DT_CHOSEN(zmk_battery));
#else
#warning                                                                                           \
    "Using a node labeled BATTERY for the battery sensor is deprecated. Set a zmk,battery chosen node instead. (Ignore this if you don't have a battery sensor.)"
static const struct device *battery;
#endif

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_LITHIUM_VOLTAGE)
static uint8_t lithium_ion_mv_to_pct(int16_t bat_mv) {
    // Simple linear approximation of a battery based off adafruit's discharge graph:
    // https://learn.adafruit.com/li-ion-and-lipoly-batteries/voltages

    if (bat_mv >= 4200) {
        return 100;
    } else if (bat_mv <= 3450) {
        return 0;
    }

    return bat_mv * 2 / 15 - 459;
}

#endif // IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_LITHIUM_VOLTAGE)

static int zmk_battery_update(const struct device *battery) {
    struct sensor_value state_of_charge;
    int rc;

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_STATE_OF_CHARGE)

    rc = sensor_sample_fetch_chan(battery, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE);
    if (rc != 0) {
        LOG_INF("!!! ERROR: Failed to fetch battery values: %d", rc);
        return rc;
    }

    rc = sensor_channel_get(battery, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &state_of_charge);

    if (rc != 0) {
        LOG_INF("!!! ERROR: Failed to get battery state of charge: %d", rc);
        return rc;
    }
#elif IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING_FETCH_MODE_LITHIUM_VOLTAGE)
    rc = sensor_sample_fetch_chan(battery, SENSOR_CHAN_VOLTAGE);
    if (rc != 0) {
        LOG_INF("!!! ERROR: Failed to fetch battery values: %d", rc);
        return rc;
    }

    struct sensor_value voltage;
    rc = sensor_channel_get(battery, SENSOR_CHAN_VOLTAGE, &voltage);

    if (rc != 0) {
        LOG_INF("!!! ERROR: Failed to get battery voltage: %d", rc);
        return rc;
    }

    uint16_t mv = voltage.val1 * 1000 + (voltage.val2 / 1000);

    // Detailed ADC debugging - use INF to always see it
    LOG_INF("ADC raw: val1=%d val2=%d => %d mV", voltage.val1, voltage.val2, mv);

    // When USB is connected, ADC reads charging voltage (~4.2V) instead of real battery level
    // Estimate charging progress based on time
    bool usb_present = is_usb_power_present();
    LOG_INF("USB present: %s", usb_present ? "YES" : "NO");

    if (usb_present && mv >= 4100) {
        // USB charging detected
        if (charging_start_time == 0) {
            // First time detecting charging - save initial state
            charging_start_level = last_state_without_usb > 0 ? last_state_without_usb : lithium_ion_mv_to_pct(mv);
            charging_start_time = k_uptime_get();
            state_of_charge.val1 = charging_start_level;
            LOG_INF("=== Charging started at %d%% (measured %d mV) ===", charging_start_level, mv);
        } else {
            // Estimate progress based on time
            // Assumptions: 550mAh battery, 300mA charge current
            // Full charge from 0% takes ~110 minutes
            // Rate: 0-90% = 0.91%/min, 90-100% = 0.45%/min (slower near full)
            int64_t elapsed_ms = k_uptime_get() - charging_start_time;
            int32_t elapsed_min = (int32_t)(elapsed_ms / 60000);

            // Calculate estimated charge added
            uint8_t charge_added = 0;
            int32_t remaining_min = elapsed_min;

            // Phase 1: Fast charging (0-90%)
            if (charging_start_level < 90) {
                int32_t phase1_capacity = 90 - charging_start_level;
                int32_t phase1_minutes = (int32_t)(remaining_min < phase1_capacity * 1.1f ? remaining_min : phase1_capacity * 1.1f);
                charge_added += (uint8_t)(phase1_minutes / 1.1f);  // 0.91%/min
                remaining_min -= phase1_minutes;
            }

            // Phase 2: Slow charging (90-100%)
            if (remaining_min > 0 && (charging_start_level + charge_added) >= 90) {
                int32_t phase2_minutes = remaining_min;
                charge_added += (uint8_t)(phase2_minutes / 2.2f);  // 0.45%/min
            }

            state_of_charge.val1 = charging_start_level + charge_added;
            if (state_of_charge.val1 > 100) {
                state_of_charge.val1 = 100;
            }

            LOG_INF("=== Charging: %d%% (started at %d%%, +%d%% over %d min, measured %d mV) ===",
                    state_of_charge.val1, charging_start_level, charge_added, elapsed_min, mv);
        }
    } else {
        // USB not present or voltage is normal - use real measurement
        state_of_charge.val1 = lithium_ion_mv_to_pct(mv);

        // Reset charging tracking when USB disconnected
        if (charging_start_time != 0) {
            charging_start_time = 0;
            charging_start_level = 0;
            LOG_INF("=== Charging stopped, real level: %d%% (%d mV) ===", state_of_charge.val1, mv);
        }

        // Cache value when USB is not present
        if (!usb_present) {
            last_state_without_usb = state_of_charge.val1;
        }

        LOG_INF("State of charge: %d%% from %d mV", state_of_charge.val1, mv);
    }
#else
#error "Not a supported reporting fetch mode"
#endif

    if (last_state_of_charge != state_of_charge.val1) {
        last_state_of_charge = state_of_charge.val1;

        rc = raise_zmk_battery_state_changed(
            (struct zmk_battery_state_changed){.state_of_charge = last_state_of_charge});

        if (rc != 0) {
            LOG_ERR("Failed to raise battery state changed event: %d", rc);
            return rc;
        }
    }

#if IS_ENABLED(CONFIG_BT_BAS)
    if (bt_bas_get_battery_level() != last_state_of_charge) {
        LOG_DBG("Setting BAS GATT battery level to %d.", last_state_of_charge);

        rc = bt_bas_set_battery_level(last_state_of_charge);

        if (rc != 0) {
            LOG_WRN("Failed to set BAS GATT battery level (err %d)", rc);
            return rc;
        }
    }
#endif

    return rc;
}

static void zmk_battery_work(struct k_work *work) {
    int rc = zmk_battery_update(battery);

    if (rc != 0) {
        LOG_INF("!!! ERROR: Failed to update battery value: %d", rc);
    }
}

K_WORK_DEFINE(battery_work, zmk_battery_work);

// Delayed work for initial measurement after boot
static void zmk_battery_delayed_work(struct k_work *work) {
    LOG_INF("=== BOOT: Battery measurement ===");
    int rc = zmk_battery_update(battery);
    if (rc != 0) {
        LOG_INF("!!! BOOT measurement failed with error: %d", rc);
    } else {
        LOG_INF("=== BOOT measurement completed successfully ===");
    }
}

K_WORK_DELAYABLE_DEFINE(battery_delayed_work, zmk_battery_delayed_work);

static int zmk_battery_init(void) {
#if !DT_HAS_CHOSEN(zmk_battery)
    battery = device_get_binding("BATTERY");

    if (battery == NULL) {
        return -ENODEV;
    }

    LOG_WRN("Finding battery device labeled BATTERY is deprecated. Use zmk,battery chosen node.");
#endif

    if (!device_is_ready(battery)) {
        LOG_ERR("Battery device \"%s\" is not ready", battery->name);
        return -ENODEV;
    }

    // Schedule initial measurement 3 seconds after boot
    // This allows RGB, display, and BLE to stabilize first
    // Measuring without heavy startup load gives accurate voltage reading
    k_work_schedule(&battery_delayed_work, K_SECONDS(3));

    return 0;
}

static int battery_event_listener(const zmk_event_t *eh) {

    if (as_zmk_activity_state_changed(eh)) {
        enum zmk_activity_state state = zmk_activity_get_state();

        LOG_INF("=== Activity state: %d ===", state);

        int rc;
        switch (state) {
        case ZMK_ACTIVITY_ACTIVE:
            LOG_INF("=== ACTIVE: Battery measurement ===");
            rc = zmk_battery_update(battery);
            if (rc != 0) {
                LOG_INF("!!! ACTIVE measurement failed: %d", rc);
            }
            return 0;

        case ZMK_ACTIVITY_IDLE:
            LOG_INF("=== IDLE: Battery measurement ===");
            rc = zmk_battery_update(battery);
            if (rc != 0) {
                LOG_INF("!!! IDLE measurement failed: %d", rc);
            }
            return 0;

        case ZMK_ACTIVITY_SLEEP:
            LOG_INF("=== SLEEP: Battery measurement ===");
            rc = zmk_battery_update(battery);
            if (rc != 0) {
                LOG_INF("!!! SLEEP measurement failed: %d", rc);
            }
            return 0;

        default:
            LOG_INF("=== Unknown state: %d ===", state);
            break;
        }
    }
    return -ENOTSUP;
}

ZMK_LISTENER(battery, battery_event_listener);

ZMK_SUBSCRIPTION(battery, zmk_activity_state_changed);

SYS_INIT(zmk_battery_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
