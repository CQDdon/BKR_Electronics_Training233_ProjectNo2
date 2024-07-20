#include "uart.h"

static char uartDataToSend[256];
static volatile uint8_t uartReadyToSend = 0;

void sendUART(const char* data) {
    strncpy(uartDataToSend, data, sizeof(uartDataToSend) - 1);
    uartDataToSend[sizeof(uartDataToSend) - 1] = '\0';
    uartReadyToSend = 1;
}

