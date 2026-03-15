/*
 * propeller_receiver.c
 *
 * Propeller Activity Board : UART Servo Controller
 *
 * Receives ASCII servo commands from FeatherS3 ESP32 on UART:
 *   L<3digits>R<3digits>G<3digits>\n
 *   Example: L090R090G000\n   (stop, gripper open)
 *   Example: L120R060G180\n   (forward, gripper close)
 *
 * WIRING:
 *   Propeller P0 (RX) <- FeatherS3 TX (GPIO33)
 *   Propeller P1 (TX) -> FeatherS3 RX (GPIO38) [optional]
 *   Servo left  -> P13
 *   Servo right -> P14
 *   Servo grip  -> P12
 *   LED         -> P26
 *
 * BUILD: SimpleIDE, libraries: simpletools, fdserial, servo
 */

#include "simpletools.h"
#include "fdserial.h"
#include "servo.h"

/* ---- Configuration ---- */
#define UART_RX       0
#define UART_TX       1
#define UART_BAUD     115200

#define PIN_LEFT      13
#define PIN_RIGHT     14
#define PIN_GRIP      12
#define PIN_LED       26

/* ---- Globals ---- */
static fdserial *uart;
static int curL = 90, curR = 90, curG = 0;
static int lastDataTime = 0;
static int gotAnyData = 0;

/* ---- Parse 3-digit number at buf[pos..pos+2] ---- */
static int parse3(char *buf, int pos)
{
    int val = 0;
    int i;
    for (i = 0; i < 3; i++)
    {
        char c = buf[pos + i];
        if (c >= '0' && c <= '9')
            val = val * 10 + (c - '0');
    }
    return val;
}

/* ---- Non-blocking line reader ----
 * Accumulates bytes from fdserial. Returns complete line length
 * when \n received, 0 if no complete line yet, -1 on overflow. */
static char lineBuf[32];
static int linePos = 0;

static int tryReadLine(char *out, int maxLen)
{
    int c, len, j;

    while (1)
    {
        c = fdserial_rxCheck(uart);
        if (c == -1) break;
        if (c == '\r') continue;
        if (c == '\n')
        {
            lineBuf[linePos] = '\0';
            len = linePos;
            linePos = 0;
            if (len > 0 && len < maxLen)
            {
                for (j = 0; j <= len; j++) out[j] = lineBuf[j];
                return len;
            }
            return 0;
        }
        if (linePos < (int)sizeof(lineBuf) - 1)
            lineBuf[linePos++] = (char)c;
        else
        {
            linePos = 0;
            return -1;
        }
    }
    return 0;
}

/* ---- Main ---- */
int main()
{
    char buf[32];
    int len, l, r, g;
    int elapsed_ms;
    int k;

    /* Startup: 3 quick LED blinks to confirm code is running */
    low(PIN_LED);
    for (k = 0; k < 3; k++)
    {
        high(PIN_LED);
        pause(150);
        low(PIN_LED);
        pause(150);
    }

    print("===== Propeller Servo Receiver =====\r");
    print("UART: P%d(RX) P%d(TX) @ %d\r", UART_RX, UART_TX, UART_BAUD);
    print("Servos: L=P%d R=P%d G=P%d\r", PIN_LEFT, PIN_RIGHT, PIN_GRIP);
    print("====================================\r\r");

    uart = fdserial_open(UART_RX, UART_TX, 0, UART_BAUD);

    servo_angle(PIN_LEFT,  900);
    servo_angle(PIN_RIGHT, 900);
    servo_angle(PIN_GRIP,  0);

    lastDataTime = CNT;

    while (1)
    {
        len = tryReadLine(buf, sizeof(buf));

        if (len >= 12 && buf[0] == 'L' && buf[4] == 'R' && buf[8] == 'G')
        {
            l = parse3(buf, 1);
            r = parse3(buf, 5);
            g = parse3(buf, 9);

            if (l > 180) l = 180;
            if (r > 180) r = 180;
            if (g > 180) g = 180;

            servo_angle(PIN_LEFT,  l * 10);
            servo_angle(PIN_RIGHT, r * 10);
            servo_angle(PIN_GRIP,  g * 10);

            if (l != curL || r != curR || g != curG)
            {
                high(PIN_LED);
                print("L=%d R=%d G=%d\r", l, r, g);
            }

            curL = l;
            curR = r;
            curG = g;
            lastDataTime = CNT;
            gotAnyData = 1;
        }
        else
        {
            low(PIN_LED);
        }

        /* Watchdog: rapid blink if no data for 2 seconds */
        elapsed_ms = (int)((unsigned int)(CNT - lastDataTime)) / (CLKFREQ / 1000);
        if (gotAnyData && elapsed_ms > 2000)
        {
            high(PIN_LED);
            pause(20);
            low(PIN_LED);
        }

        pause(2);
    }

    return 0;
}