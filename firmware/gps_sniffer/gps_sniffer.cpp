// GPS sniffer - a bench diagnostic, NOT flight firmware.
//
// WHAT THIS IS FOR
// ----------------
// The flight firmware has always assumed the GPS is wired to GP4/GP5, because
// that is what UART1_setup() configures. Nobody ever verified it. If the
// module is actually on a different pair of pins, every measurement taken at
// GP4/GP5 would read exactly as it does now - a static, idle line - and no
// amount of parser work would ever help.
//
// So this scans EVERY GPIO on the Pico and reports which ones are electrically
// alive, rather than trusting the firmware's assumption:
//
//   FLOATING    follows whichever internal pull is applied - nothing attached
//   DRIVEN HIGH held high against both pulls - attached and idle, or a pull-up
//   DRIVEN LOW  held low against both pulls
//   transitions ACTUAL TRAFFIC - this is the pin the GPS is talking on
//
// Each pin is watched for longer than one second, because a GPS at its default
// 1 Hz would otherwise be missed between bursts.
//
// Any pin showing transitions is then decoded as a UART at every plausible
// baud - including 921600, which the ORIGINAL firmware may have switched the
// module to with its UBX CFG-PRT message and which nothing has listened at
// since.
//
// Nothing here configures PWM, so the ESC pins stay high-impedance throughout
// and the motors cannot move.
//
// Flash the flight firmware back afterwards:
//   sudo picotool load -x /home/pi/firmware/flight_controller_gpsfix.uf2

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

// Physical pin number for each GPIO, so the report can name the pin you can
// actually count to on the board.
static const int PHYS[29] = {
     1,  2,  4,  5,  6,  7,  9, 10, 11, 12,   // GP0-GP9
    14, 15, 16, 17, 19, 20, 21, 22, 24, 25,   // GP10-GP19
    26, 27, 29, -1, -1, -1, 31, 32, 34        // GP20-GP28 (23/24/25 are internal)
};

// Pins that are wired to something internal on a Pico board and are not
// header pins - scanning them tells you nothing about the GPS.
static bool is_internal(int gp){ return gp == 23 || gp == 24 || gp == 25; }

#define GPS_TX_PIN 4        // physical pin 6 - UART1 TX, to the module RX
#define GPS_RX_PIN 5        // physical pin 7 - UART1 RX, from the module TX

static const int BAUDS[] = { 9600, 4800, 19200, 38400, 57600, 115200, 230400, 460800, 921600 };
#define BAUD_COUNT (sizeof(BAUDS) / sizeof(BAUDS[0]))

#define SCAN_MS    1300        // > 1 s so a 1 Hz burst cannot fall between windows
#define LISTEN_MS  1500
#define CAP_MAX    160

static uint8_t cap[CAP_MAX];

static void dump(const uint8_t *buf, int len){
    for(int off = 0; off < len; off += 16){
        printf("      %04X  ", off);
        for(int i = 0; i < 16; i++){
            if(off + i < len){ printf("%02X ", buf[off + i]); }
            else             { printf("   "); }
        }
        printf(" |");
        for(int i = 0; i < 16 && off + i < len; i++){
            const uint8_t c = buf[off + i];
            printf("%c", (c >= 32 && c < 127) ? (char)c : '.');
        }
        printf("|\n");
    }
}

// Which UART (if any) can take this pin as its RX. RP2040 only allows a fixed
// set: UART0 RX on GP1/GP13/GP17, UART1 RX on GP5/GP9/GP21.
static uart_inst_t *uart_for_rx(int gp){
    if(gp == 1 || gp == 13 || gp == 17){ return uart0; }
    if(gp == 5 || gp == 9 || gp == 21){ return uart1; }
    return NULL;
}

struct PinResult {
    bool driven_high, driven_low, floating;
    uint32_t transitions;
    int duty;
};

// reconfigure=true forces the pin to a plain input first, so the Pico cannot
// be driving it and mask an external signal. That is what the scan wants.
//
// reconfigure=false leaves the pin's current function alone and only samples
// the pad. gpio_get() reads the input synchroniser whatever peripheral owns
// the pin, so this still works - and it is the only way to self-test the
// sampling loop against a signal the Pico is generating itself, since forcing
// the pin to SIO would disconnect that signal.
static PinResult probe(int gp, bool reconfigure = true){
    PinResult r = {};
    if(reconfigure){
        gpio_set_function(gp, GPIO_FUNC_SIO);
        gpio_set_dir(gp, GPIO_IN);
    }

    gpio_pull_up(gp);   sleep_ms(3); const bool pu = gpio_get(gp);
    gpio_pull_down(gp); sleep_ms(3); const bool pd = gpio_get(gp);
    gpio_disable_pulls(gp); sleep_ms(3);

    r.floating    =  pu && !pd;
    r.driven_high =  pu &&  pd;
    r.driven_low  = !pu && !pd;

    bool prev = gpio_get(gp);
    uint32_t high = 0, n = 0;
    const absolute_time_t end = make_timeout_time_ms(SCAN_MS);
    while(!time_reached(end)){
        const bool v = gpio_get(gp);
        if(v != prev){ r.transitions++; prev = v; }
        if(v){ high++; }
        n++;
    }
    r.duty = n ? (int)((high * 100) / n) : 0;
    return r;
}

static void decode_pin(int gp){
    uart_inst_t *u = uart_for_rx(gp);
    if(u == NULL){
        printf("    GP%d cannot be a hardware UART RX pin on RP2040 - move the\n"
               "    wire to GP5 (physical pin 7) to decode it.\n", gp);
        return;
    }
    printf("    Decoding GP%d as %s RX:\n", gp, (u == uart0) ? "UART0" : "UART1");
    gpio_set_function(gp, GPIO_FUNC_UART);
    uart_init(u, 9600);
    uart_set_hw_flow(u, false, false);
    uart_set_format(u, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(u, true);

    for(unsigned b = 0; b < BAUD_COUNT; b++){
        uart_set_baudrate(u, BAUDS[b]);
        while(uart_is_readable(u)){ (void)uart_get_hw(u)->dr; }
        uart_get_hw(u)->rsr = 0x0F;
        sleep_ms(20);
        while(uart_is_readable(u)){ (void)uart_get_hw(u)->dr; }

        int n = 0, nf = 0, printable = 0, dollars = 0;
        uint32_t err = 0;
        const absolute_time_t end = make_timeout_time_ms(LISTEN_MS);
        while(!time_reached(end)){
            if(uart_is_readable(u)){
                const uint32_t dr = uart_get_hw(u)->dr;
                err |= (dr >> 8) & 0x0F;
                const uint8_t c = (uint8_t)(dr & 0xFF);
                if(c == '$'){ dollars++; }
                if(c >= 32 && c < 127){ printable++; }
                if(nf < CAP_MAX){ cap[nf++] = c; }
                n++;
            }
        }
        if(n == 0){ printf("      baud %6d : silence\n", BAUDS[b]); continue; }
        printf("      baud %6d : %d words, %d printable, %d '$', err=0x%X%s\n",
               BAUDS[b], n, printable, dollars, (unsigned)err,
               (err == 0 && printable > n / 2) ? "   <<< CLEAN ASCII" : "");
        dump(cap, nf);
        if(dollars > 0){
            printf("      *** NMEA FOUND on GP%d at %d baud ***\n", gp, BAUDS[b]);
        }
    }
}

// ---------------------------------------------------------------------------
// REVIVE ATTEMPT
//
// The ORIGINAL firmware sent u-blox UBX configuration frames to this module at
// every boot - CFG-PRT to force 921600 baud, CFG-RATE, and two CFG-MSG frames
// to disable sentences. If the module understood any of them, it may still be
// sitting in whatever state it was left in, and a receiver configured with no
// output protocol is SILENT with its TX idling high - which is exactly what is
// measured on GP5.
//
// That state would have been caused by this project's own firmware, and on any
// receiver with battery-backed RAM or flash it survives a power cycle. So
// before blaming the hardware, this shouts the standard "wake up and go back
// to defaults" commands at the module in every dialect and at every baud:
//
//   u-blox    UBX CFG-RST (cold start) and CFG-PRT (9600, NMEA output ENABLED)
//   CASIC     $PCAS01 (set 9600) and $PCAS03 (enable GGA/GSA output)
//   MediaTek  $PMTK104 (full cold restart to factory defaults)
//
// Sending an unknown command to the wrong chip is harmless - it fails that
// chip's checksum and is ignored.
// ---------------------------------------------------------------------------

static void send_ubx(uart_inst_t *u, uint8_t cls, uint8_t id,
                     const uint8_t *payload, uint16_t len){
    uint8_t hdr[6] = { 0xB5, 0x62, cls, id, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8) };
    uint8_t a = 0, b = 0;
    for(int i = 2; i < 6; i++){ a = (uint8_t)(a + hdr[i]); b = (uint8_t)(b + a); }
    for(uint16_t i = 0; i < len; i++){ a = (uint8_t)(a + payload[i]); b = (uint8_t)(b + a); }
    uart_write_blocking(u, hdr, 6);
    if(len){ uart_write_blocking(u, payload, len); }
    const uint8_t ck[2] = { a, b };
    uart_write_blocking(u, ck, 2);
}

// body is everything between the dollar sign and the star, e.g. "PCAS01,1"
static void send_nmea(uart_inst_t *u, const char *body){
    uint8_t sum = 0;
    for(const char *q = body; *q; q++){ sum ^= (uint8_t)*q; }
    char buf[96];
    const int n = snprintf(buf, sizeof(buf), "$%s*%02X\r\n", body, sum);
    uart_write_blocking(u, (const uint8_t *)buf, (size_t)n);
}

static void revive_attempt(){
    printf("\n  [REVIVE] shouting reset/enable commands at the module\n");

    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_init(uart1, 9600);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, true);

    // CFG-RST: navBbrMask 0xFFFF (cold start), resetMode 0x01 (controlled SW)
    const uint8_t cfg_rst[4] = { 0xFF, 0xFF, 0x01, 0x00 };

    // CFG-PRT: UART1, 8N1, 9600 baud, in UBX+NMEA+RTCM, OUT UBX+NMEA.
    // outProtoMask = 0x0003 is the important part - it re-enables output on a
    // receiver that was left with it cleared.
    const uint8_t cfg_prt[20] = {
        0x01, 0x00, 0x00, 0x00,
        0xD0, 0x08, 0x00, 0x00,           // mode: 8N1
        0x80, 0x25, 0x00, 0x00,           // baudRate = 9600
        0x23, 0x00,                       // inProtoMask  = UBX|NMEA|RTCM
        0x03, 0x00,                       // outProtoMask = UBX|NMEA
        0x00, 0x00, 0x00, 0x00
    };

    for(unsigned b = 0; b < BAUD_COUNT; b++){
        const int baud = BAUDS[b];
        uart_set_baudrate(uart1, baud);
        sleep_ms(10);
        send_ubx(uart1, 0x06, 0x04, cfg_rst, sizeof(cfg_rst));
        sleep_ms(60);
        send_ubx(uart1, 0x06, 0x00, cfg_prt, sizeof(cfg_prt));
        sleep_ms(60);
        send_nmea(uart1, "PCAS01,1");
        sleep_ms(40);
        send_nmea(uart1, "PCAS03,1,0,0,0,1,0,0,0");
        sleep_ms(40);
        send_nmea(uart1, "PMTK104");
        sleep_ms(40);
        printf("    sent at %6d baud\n", baud);
    }
    printf("    giving the module 3 s to restart...\n");
    sleep_ms(3000);
}

// ---------------------------------------------------------------------------
// SELF-TESTS
//
// Two measurements have already been believed and turned out to be wrong (the
// flight firmware's DMA word counter and its line-transition counter). So this
// build proves its own instruments before it reports anything about the GPS.
//
// 1. Drive a known square wave onto an unused pin and check the GPIO probe
//    sees it. If the probe cannot detect a signal that is definitely there,
//    every "no transitions" result in this program is worthless.
//
// 2. Put UART1 into internal loopback and push a real NMEA sentence through
//    it. That exercises the UART receive path and the baud handling with no
//    external hardware at all. If a sentence written on the inside comes back
//    out intact, the receive chain is sound and silence really does mean
//    nothing is arriving from outside.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// RAIL VOLTAGE
//
// Per V4_Wiring_Diagram.png the GPS module's red VCC wire goes to the "ESC's
// 5V power supply" rail - the SAME rail that feeds the Pico's VSYS on physical
// pin 39. It is not on the Pico's 3V3 output.
//
// That makes the GPS's supply measurable from here without a multimeter: the
// Pico wires VSYS to ADC3 (GP29) through an internal 3:1 divider, and senses
// USB power on GP24. So:
//
//   VBUS present, VSYS ~4.6-4.8 V  -> running on USB alone. VSYS is only the
//                                     USB rail leaking through the on-board
//                                     Schottky, and whether that actually
//                                     reaches the GPS depends on the ESC
//                                     harness being plugged in.
//   VSYS ~5.0 V or above           -> the ESC BEC is live and the GPS rail
//                                     is genuinely powered.
//   VSYS below ~4.4 V              -> the rail is sagging; the GPS may be
//                                     browning out.
// ---------------------------------------------------------------------------
static void report_rails(){
    adc_init();
    adc_gpio_init(29);
    adc_select_input(3);

    // Average a few, the ADC is noisy.
    uint32_t acc = 0;
    for(int i = 0; i < 64; i++){ acc += adc_read(); sleep_us(200); }
    const float counts = (float)acc / 64.0f;
    const float vsys = counts * 3.3f / 4095.0f * 3.0f;

    gpio_set_function(24, GPIO_FUNC_SIO);
    gpio_set_dir(24, GPIO_IN);
    const bool vbus = gpio_get(24);

    printf("\n  [RAILS] the GPS shares this supply (diagram: GPS VCC -> ESC 5V rail -> VSYS pin 39)\n");
    printf("    VBUS (USB power present) : %s\n", vbus ? "YES" : "no");
    printf("    VSYS                     : %.2f V  (raw ADC %.0f)\n", (double)vsys, (double)counts);
    if(vsys < 4.4f){
        printf("    -> RAIL IS LOW. The GPS is very likely browning out or dead.\n");
    } else if(vsys < 4.9f){
        printf("    -> This is USB backfeed through the Pico's Schottky, NOT the ESC BEC.\n");
        printf("       If the ESC harness is unplugged, the GPS has no supply of its own.\n");
    } else {
        printf("    -> The ESC 5V rail is live, so the GPS IS being powered.\n");
    }
}

#define SELFTEST_PIN 15        // GP15, physical pin 20 - reads FLOATING, unused by the flight firmware

static bool selftest_gpio_probe(){
    printf("\n  [SELF-TEST 1] GPIO transition probe\n");

    // ~1 kHz square wave on an unused pin.
    gpio_set_function(SELFTEST_PIN, GPIO_FUNC_PWM);
    const uint slice = pwm_gpio_to_slice_num(SELFTEST_PIN);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 125.0f);        // 1 MHz
    pwm_config_set_wrap(&cfg, 999);             // 1 kHz
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(SELFTEST_PIN, 500);      // 50% duty
    sleep_ms(50);

    // reconfigure=false: leave PWM driving the pin, just watch the pad.
    const PinResult r = probe(SELFTEST_PIN, false);
    pwm_set_enabled(slice, false);
    gpio_set_function(SELFTEST_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(SELFTEST_PIN, GPIO_IN);

    const bool ok = r.transitions > 100;
    printf("    GP%d driven at 1 kHz -> probe saw %lu transitions, %d%% high\n",
           SELFTEST_PIN, (unsigned long)r.transitions, r.duty);
    printf("    %s\n", ok ? "PASS - the probe can detect a real signal."
                          : "*** FAIL - the probe is broken. Ignore every "
                            "'no transitions' result below. ***");
    return ok;
}

static bool selftest_uart_loopback(){
    printf("\n  [SELF-TEST 2] UART1 receive path, internal loopback\n");

    static const char *SENTENCE =
        "$GPGGA,123519.00,2544.9100,S,02811.2200,E,1,08,0.9,1280.4,M,46.9,M,,*6A\r\n";

    bool all_ok = true;
    for(unsigned b = 0; b < BAUD_COUNT; b++){
        const int baud = BAUDS[b];
        uart_init(uart1, baud);
        uart_set_hw_flow(uart1, false, false);
        uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
        uart_set_fifo_enabled(uart1, true);

        // LBE ties TXD to RXD inside the peripheral.
        hw_set_bits(&uart_get_hw(uart1)->cr, UART_UARTCR_LBE_BITS);
        sleep_ms(5);
        while(uart_is_readable(uart1)){ (void)uart_get_hw(uart1)->dr; }

        const int len = (int)strlen(SENTENCE);
        int got = 0, dollars = 0;
        uint32_t err = 0;
        char back[128];

        for(int i = 0; i < len; i++){
            uart_putc_raw(uart1, SENTENCE[i]);
            // Drain as we go - the FIFO is only 32 deep.
            while(uart_is_readable(uart1) && got < (int)sizeof(back) - 1){
                const uint32_t dr = uart_get_hw(uart1)->dr;
                err |= (dr >> 8) & 0x0F;
                const char c = (char)(dr & 0xFF);
                if(c == '$'){ dollars++; }
                back[got++] = c;
            }
        }
        const absolute_time_t end = make_timeout_time_ms(100);
        while(!time_reached(end) && got < (int)sizeof(back) - 1){
            if(uart_is_readable(uart1)){
                const uint32_t dr = uart_get_hw(uart1)->dr;
                err |= (dr >> 8) & 0x0F;
                const char c = (char)(dr & 0xFF);
                if(c == '$'){ dollars++; }
                back[got++] = c;
            }
        }
        back[got] = '\0';

        hw_clear_bits(&uart_get_hw(uart1)->cr, UART_UARTCR_LBE_BITS);

        const bool ok = (got == len) && (dollars == 1) && (err == 0);
        if(!ok){ all_ok = false; }
        printf("    baud %6d : sent %d, got %d, '$'=%d, err=0x%X  %s\n",
               baud, len, got, dollars, (unsigned)err, ok ? "PASS" : "FAIL");
    }
    printf("    %s\n", all_ok
        ? "PASS - UART1 receives correctly at every baud. Silence on the pin\n"
          "           therefore means nothing is arriving from outside."
        : "*** FAIL - the UART receive path itself is broken. ***");
    return all_ok;
}

int main(){
    stdio_init_all();
    for(int i = 0; i < 100 && !stdio_usb_connected(); i++){ sleep_ms(100); }
    sleep_ms(500);

    printf("\n=========================================================\n");
    printf(" INSTRUMENT SELF-TEST, then full pin scan\n");
    printf("=========================================================\n");
    report_rails();
    const bool probe_ok = selftest_gpio_probe();
    const bool uart_ok  = selftest_uart_loopback();
    printf("\n  Self-test summary: GPIO probe %s, UART receive %s\n",
           probe_ok ? "PASS" : "FAIL", uart_ok ? "PASS" : "FAIL");

    revive_attempt();
    printf("\n  [POST-REVIVE] listening on GP5 at every baud\n");
    decode_pin(GPS_RX_PIN);

    printf("\n=========================================================\n");
    printf(" FULL PIN SCAN - not assuming the GPS is on GP4/GP5\n");
    printf(" %d ms per pin, so a 1 Hz burst cannot be missed\n", SCAN_MS);
    printf("=========================================================\n");

    int round = 0;
    while(true){
        printf("\n================ sweep %d ================\n", ++round);
        printf("  GPIO  phys  state         transitions  high%%\n");
        printf("  ----  ----  ------------  -----------  -----\n");

        int active[29];
        int n_active = 0;

        for(int gp = 0; gp <= 28; gp++){
            if(is_internal(gp)){ continue; }
            const PinResult r = probe(gp);
            const char *state = r.floating    ? "FLOATING"
                              : r.driven_high ? "DRIVEN HIGH"
                              : r.driven_low  ? "DRIVEN LOW"
                                              : "indeterminate";
            printf("  GP%-3d %4d  %-12s  %11lu  %4d%%%s\n",
                   gp, PHYS[gp], state, (unsigned long)r.transitions, r.duty,
                   (r.transitions > 20) ? "   <<< TRAFFIC" : "");
            if(r.transitions > 20 && n_active < 29){ active[n_active++] = gp; }
        }

        printf("\n  ---------------------------------------------\n");
        if(n_active == 0){
            printf("  NO PIN on the whole board is carrying traffic.\n");
            printf("  The module is not transmitting on any Pico pin.\n");
        } else {
            printf("  %d pin(s) carrying traffic:", n_active);
            for(int i = 0; i < n_active; i++){ printf(" GP%d(pin %d)", active[i], PHYS[active[i]]); }
            printf("\n\n");
            for(int i = 0; i < n_active; i++){ decode_pin(active[i]); }
        }
    }
}
