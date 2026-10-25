/*
 * Copyright (c) 2025 sekigon-gonnoc
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/battery.h>
#include <zmk/events/activity_state_changed.h>

#if defined(CONFIG_BT)
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zmk/ble.h>
#include <zmk/events/ble_active_profile_changed.h>
#if defined(CONFIG_ZMK_SPLIT) && !defined(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
#include <zmk/split/bluetooth/peripheral.h>
#endif
#endif
LOG_MODULE_REGISTER(zmk_status_led, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_status_led

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

// Timer work is only needed for Bluetooth advertising indications.
#if defined(CONFIG_BT)
static struct k_work_delayable status_led_timer;
#endif

// LED GPIO
#define LED_NODE DT_PHANDLE(DT_INST(0, zmk_status_led), led)
static const struct gpio_dt_spec led_gpio = GPIO_DT_SPEC_GET(LED_NODE, gpios);
static bool led_state = false;

// Current state tracking
static bool is_showing_battery_level = false; // Flag to track battery level display
static bool suspended = false;                // Track suspended state
#if defined(CONFIG_BT)
static bool is_advertising = false;
static bool is_connected = false;
#if !defined(CONFIG_ZMK_SPLIT) || defined(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static bool status_led_initialized = false;
#endif

// Counter variable to track blink state
static uint8_t blink_count = 0;
static const uint8_t MAX_BLINKS = 2; // 2 blinks
#endif

static int status_led_on(void) {
    int ret = -ENODEV;
    if (device_is_ready(led_gpio.port)) {
        ret = gpio_pin_set_dt(&led_gpio, 1);
        if (ret == 0) {
            led_state = true;
        } else {
            LOG_ERR("Failed to turn on LED: %d", ret);
        }
    } else {
        LOG_ERR("GPIO device not ready");
    }
    return ret;
}

static int status_led_off(void) {
    int ret = -ENODEV;
    if (device_is_ready(led_gpio.port)) {
        ret = gpio_pin_set_dt(&led_gpio, 0);
        if (ret == 0) {
            led_state = false;
        } else {
            LOG_ERR("Failed to turn off LED: %d", ret);
        }
    } else {
        LOG_ERR("GPIO device not ready");
    }
    return ret;
}

#if defined(CONFIG_BT)
static void start_advertising_indicator(void) {
    // Don't start advertising indicator while showing battery level
    if (is_showing_battery_level) {
        return;
    }

    is_advertising = true;
    blink_count = 0; // Reset counter
    k_work_schedule(&status_led_timer, K_NO_WAIT);
}
#endif // CONFIG_BT

static void blink_n_times(uint8_t n) {
    is_showing_battery_level = true;

    for (int i = 0; i < n; i++) {
        status_led_on();
        k_sleep(K_MSEC(CONFIG_ZMK_STATUS_LED_BATTERY_BLINK_MS));
        status_led_off();
        k_sleep(K_MSEC(CONFIG_ZMK_STATUS_LED_BATTERY_PAUSE_MS));
    }

    is_showing_battery_level = false;

#if defined(CONFIG_BT)
    // Start advertising indicator if needed after battery display is complete
    if (!is_connected && CONFIG_ZMK_STATUS_LED_ADVERTISING) {
        k_sleep(K_MSEC(CONFIG_ZMK_STATUS_LED_ADVERTISING_INTERVAL_MS));
        start_advertising_indicator();
    }
#endif
}

#if defined(CONFIG_BT)
static void stop_advertising_indicator(void) {
    is_advertising = false;
    k_work_cancel_delayable(&status_led_timer);
    status_led_off();
}

static void status_led_work_handler(struct k_work *work) {
    // Don't run advertising blinks while showing battery level or when suspended
    if (is_showing_battery_level || suspended) {
        return;
    }

    if (is_advertising && CONFIG_ZMK_STATUS_LED_ADVERTISING) {
        /* A missing BLE connection does not imply advertising while the
         * Bluetooth stack is disabled. Keep polling so the indicator resumes
         * when Bluetooth is enabled again. */
        if (!bt_is_ready()) {
            status_led_off();
            k_work_schedule(&status_led_timer,
                            K_MSEC(CONFIG_ZMK_STATUS_LED_ADVERTISING_INTERVAL_MS));
            return;
        }
        // Check actual connection state and update is_connected if needed
        bool actually_connected = false;
#if !defined(CONFIG_ZMK_SPLIT) || defined(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
        // For central or non-split keyboards, check profile connection
        actually_connected = zmk_ble_active_profile_is_connected();
#else
        // For peripherals, check connection to central
        actually_connected = zmk_split_bt_peripheral_is_connected();
#endif

        // If we detect connection but our state says we're not connected, update
        // state and stop advertising
        if (actually_connected && !is_connected) {
            LOG_INF("Connection detected in work handler, stopping advertising "
                    "indicator");
            is_connected = true;
            stop_advertising_indicator();
            return;
        }

        if (led_state) {
            // If LED is on, turn it off and set the timing for the next blink
            status_led_off();

            // Increment counter
            blink_count++;

            if (blink_count < MAX_BLINKS) {
                // If we haven't reached the required number of blinks, schedule next
                // blink with short interval
                k_work_schedule(&status_led_timer, K_MSEC(CONFIG_ZMK_STATUS_LED_PAUSE_MS));
            } else {
                // If we've blinked the required number of times, reset counter and set
                // a longer interval
                blink_count = 0;
                k_work_schedule(&status_led_timer,
                                K_MSEC(CONFIG_ZMK_STATUS_LED_ADVERTISING_INTERVAL_MS));
            }
        } else {
            // If LED is off, turn it on
            status_led_on();
            // Set short on duration
            k_work_schedule(&status_led_timer, K_MSEC(CONFIG_ZMK_STATUS_LED_BLINK_MS));
        }
    } else {
        status_led_off();
        blink_count = 0; // Reset counter when not in advertising mode
    }
}
#endif // CONFIG_BT

static void show_battery_level(void) {
#if CONFIG_ZMK_STATUS_LED_BATTERY_LEVEL
    int batt_lvl = zmk_battery_state_of_charge();
    uint8_t blinks = 1;

    if (batt_lvl > 70) {
        blinks = 3;
    } else if (batt_lvl > 30) {
        blinks = 2;
    } else {
        blinks = 1;
    }

    blink_n_times(blinks);
#endif
}

#if defined(CONFIG_BT)
static void show_connected(void) {
#if CONFIG_ZMK_STATUS_LED_CONNECTED
    // Don't interrupt battery level display
    if (is_showing_battery_level) {
        return;
    }

    status_led_on();
    k_sleep(K_MSEC(CONFIG_ZMK_STATUS_LED_CONNECTED_MS));
    status_led_off();
#endif
}

static void bt_connected(struct bt_conn *conn, uint8_t err) {
    // if (err) {
    //     return;
    // }

    is_connected = true;
    stop_advertising_indicator();
    show_connected();
}

static void bt_disconnected(struct bt_conn *conn, uint8_t reason) {
    is_connected = false;
    start_advertising_indicator();
}

static struct bt_conn_cb conn_callbacks = {
    .connected = bt_connected,
    .disconnected = bt_disconnected,
};

#if !defined(CONFIG_ZMK_SPLIT) || defined(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static int active_profile_changed_listener(const zmk_event_t *eh) {
    if (as_zmk_ble_active_profile_changed(eh) == NULL || !status_led_initialized) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    is_connected = zmk_ble_active_profile_is_connected();
    if (is_connected) {
        stop_advertising_indicator();
    } else {
        start_advertising_indicator();
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(status_led_active_profile, active_profile_changed_listener);
ZMK_SUBSCRIPTION(status_led_active_profile, zmk_ble_active_profile_changed);
#endif
#endif // CONFIG_BT

// Delayed initialization work
static struct k_work_delayable status_led_init_work;

// Function to handle the actual initialization tasks
static void status_led_init_work_handler(struct k_work *work) {
    LOG_INF("Executing delayed status LED initialization");

#if defined(CONFIG_BT)
    k_work_init_delayable(&status_led_timer, status_led_work_handler);
#if !defined(CONFIG_ZMK_SPLIT) || defined(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    status_led_initialized = true;
#endif

    bt_conn_cb_register(&conn_callbacks);
#endif

    // Wait a bit for battery reading to stabilize
    k_sleep(K_MSEC(500));

    // Show battery level at startup
    show_battery_level();

#if defined(CONFIG_BT)
    // Check connectivity based on keyboard role
#if !defined(CONFIG_ZMK_SPLIT) || defined(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    // For central or non-split keyboards, check profile connection
    if (zmk_ble_active_profile_is_connected()) {
        is_connected = true;
    } else {
        start_advertising_indicator();
    }
#else
    // For peripherals, check connection to central
    if (zmk_split_bt_peripheral_is_connected()) {
        is_connected = true;
    } else {
        start_advertising_indicator();
    }
#endif
#endif // CONFIG_BT

    LOG_INF("Status LED initialization completed");
}

// Handle activity state change events
static int activity_state_changed_listener(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *event = as_zmk_activity_state_changed(eh);

    switch (event->state) {
    case ZMK_ACTIVITY_SLEEP:
        LOG_INF("Status LED system entering sleep mode");
        suspended = true;
        status_led_off();
        break;
    case ZMK_ACTIVITY_ACTIVE:
        LOG_INF("Status LED system resuming from sleep");
        suspended = false;
        break;
    default:
        break;
    }

    return 0;
}

ZMK_LISTENER(status_led, activity_state_changed_listener);
ZMK_SUBSCRIPTION(status_led, zmk_activity_state_changed);

static int status_led_init(void) {
    LOG_INF("Starting status LED basic initialization");

    // Find the status_led_device node
    if (!DT_NODE_EXISTS(DT_INST(0, zmk_status_led))) {
        LOG_ERR("No status LED device found");
        return -ENODEV;
    }

    // Check if the LED GPIO device is ready
    if (!device_is_ready(led_gpio.port)) {
        LOG_ERR("LED GPIO device not ready");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&led_gpio, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        LOG_ERR("Failed to configure GPIO pin: %d", ret);
        return ret;
    }

    LOG_INF("GPIO LED configured successfully: port %s, pin %d", led_gpio.port->name, led_gpio.pin);

    // Initialize and schedule the delayed work
    k_work_init_delayable(&status_led_init_work, status_led_init_work_handler);
    k_work_schedule(&status_led_init_work, K_MSEC(1)); // Delay for 1 ms

    LOG_INF("Status LED basic initialization complete, full initialization "
            "scheduled");
    return 0;
}

// Make sure the initialization priority is high enough to run after LED drivers
SYS_INIT(status_led_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
