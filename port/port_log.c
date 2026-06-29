/**
 *    @file    port_log.c
 *
 *    @brief    header file for port_log
 *
 *
 *
 *    @author    RJAM
 *    @created   Fri 12 Jun 00:43:51 BST 2026
 */

/** Includes **/
#include <stdarg.h>
#include <stdint.h>
/** Private Data **/

/** Function Prototypes **/

/** Static Functions **/

/** Tasks **/

/** Public Functions **/
#define ASCII_OFFSET_30 30

void __printf_integer(void)
{
    uint32_t printme = 12345;
    unsigned char printbuff[9] = {0};
    uint8_t pbuff_index = 0;

    while (printme > 10) {
        uint8_t sub = printme % 10;
        printme /= 10;
        printbuff[pbuff_index++] = sub + ASCII_OFFSET_30;
    }

    printbuff[pbuff_index++] = printme + ASCII_OFFSET_30;

    while (--pbuff_index) {
        uart_write(&debug_uart, printbuff[pbuff_index], 100000);
    }
}

void __printf(const char *const msg, ...)
{
    va_list args;
    va_start(args, msg);
    char *p = msg;
    uint8_t _fmt = 0;
    do {
        // uart_send_byte(*p, 1, timeout);
        if ('%' == *p) {
            _fmt = 1;
            continue;
        }
        while (_fmt) {
            if (*p == 'd') {
                va_arg(args, int);
            } else if (*p == 'l') {
                va_arg(args, long int);
            } else if (*p == 'f') {
                va_arg(args, double);
            } else if (*p == 'c') {
                va_arg(args, char);
            } else if (*p == 's') {
                va_arg(args, char *);
            }
        }
        if (*p == ' ' || *p == NULL) {
            _fmt = 0;
        }
    } while (*p++);

    va_end(args);
}

void __attribute__((weak)) log_fatal(const char *const msg, ...)
{
    __printf("[FATAL] %s", msg);
}

void __attribute__((weak)) log_error(const char *const msg, ...)
{
    __printf("[ERROR] %s", msg);
}

void __attribute__((weak)) log_warning(const char *const msg, ...)
{
    __printf("[WARN] %s", msg);
}

void __attribute__((weak)) log_info(const char *const msg, ...)
{
    __printf("[INFO] %s", msg);
}

void __attribute__((weak)) log_verbose(const char *const msg, ...)
{
    __printf("[VERB.] %s", msg);
}

/** END **/
