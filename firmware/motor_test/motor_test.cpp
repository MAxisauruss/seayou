// Motor identification test - PROPS OFF.
//
// Spins ONE output at a time so you can see which physical motor each GPIO
// actually drives. No PIDs, no sensors, no mixer: just a bare PWM signal, so
// nothing here can be confused by an attitude estimate or a stabilisation
// loop fighting back.
//
// Watch the drone and note the ORDER the motors spin in. The firmware's
// mixer assumes GP6=front-left, GP7=front-right, GP8=REAR-RIGHT,
// GP9=REAR-LEFT (that is what its roll/pitch/yaw sign pattern works out to).
// If the physical order differs, that is the bug.
//
// Same PWM setup as the flight firmware: 50Hz, and duty
// 3437.5 + 2.8125*value counts on a 62501-count wrap, so value 0 = 1100us
// (idle) and value 1000 = 2000us (full). The test uses 150 ~= 1235us, just
// enough to turn a motor over.

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "pico/stdio_usb.h"

#define IDLE_VALUE 0     // 1100us - ESC idle, motor stopped
#define TEST_VALUE 150   // ~1235us - gentle spin. PROPS MUST BE OFF.

#define SPIN_MS  3000
#define PAUSE_MS 2000

static const uint8_t PINS[4] = {6, 7, 8, 9};
// GP6/7 -> slice 3 ch A/B, GP8/9 -> slice 4 ch A/B (same as flight firmware)
static const uint SLICE[4] = {3, 3, 4, 4};
static const uint CHAN[4] = {PWM_CHAN_A, PWM_CHAN_B, PWM_CHAN_A, PWM_CHAN_B};

static const char *EXPECTED[4] = {
    "front-left", "front-right", "REAR-RIGHT", "REAR-LEFT"
};

static void set_output(int i, int value) {
    pwm_set_chan_level(SLICE[i], CHAN[i], (int)(3437.5f + 2.8125f * value));
}

static void all_idle(void) {
    for (int i = 0; i < 4; i++) { set_output(i, IDLE_VALUE); }
}

int main() {
    stdio_usb_init();

    for (int i = 0; i < 4; i++) { gpio_set_function(PINS[i], GPIO_FUNC_PWM); }
    pwm_set_clkdiv(3, 40); pwm_set_wrap(3, 62500); pwm_set_enabled(3, true);
    pwm_set_clkdiv(4, 40); pwm_set_wrap(4, 62500); pwm_set_enabled(4, true);

    // Hold idle long enough for the ESCs to arm before anything moves.
    all_idle();
    sleep_ms(6000);

    while (1) {
        for (int i = 0; i < 4; i++) {
            printf("SPINNING output %d -> GP%d   (mixer expects: %s)\n",
                   i, PINS[i], EXPECTED[i]);
            set_output(i, TEST_VALUE);
            sleep_ms(SPIN_MS);

            all_idle();
            printf("  ...idle\n");
            sleep_ms(PAUSE_MS);
        }
        printf("--- sequence complete, repeating ---\n");
    }
}
