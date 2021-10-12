/*
 * uart.h
 *
 *  Created on: 2 окт. 2021 г.
 *      Author: rbalykov
 */

#ifndef INCLUDE_UART_RXTX_H_
#define INCLUDE_UART_RXTX_H_


#include <linux/types.h>

#include "dmx.h"

#define TX_NAME_MAX	(16)


struct uart_tx;

/**
 * 		struct uart_tx_ops
 *
 *
 * 		int (*start) (struct uart_tx *tx);
 *
 * Public.
 * Start transmitting.
 *
 * 		void (*stop) (struct uart_tx *tx);
 *
 * Public.
 * Stop transmitting.
 *
 * 		size_t (*read) (struct uart_tx *tx, uint8_t *data, size_t size);
 *
 * Public.
 * Get frame from outer space. Size is trimmed to DMX_FRAME_MAX.
 *
 *		int (*break_ctl) (struct uart_tx *tx, bool on);
 *
 * Private.
 * Set BREAK state on/off.
 *
 * 		int (*send) (struct uart_tx *tx);
 *
 * Private.
 * Send single frame.
 *
 * 		int (*wait) (struct uart_tx *tx, ktime_t maf);
 *
 * Private.
 * Wait until data sent.
 */


struct uart_tx_ops
{
	int (*start) (struct uart_tx *tx);
	void (*stop) (struct uart_tx *tx);
	size_t (*read) (struct uart_tx *tx, uint8_t *data, size_t size);
	int (*break_ctl) (struct uart_tx *tx, bool on);
	int (*send) (struct uart_tx *tx);
	int (*wait) (struct uart_tx *tx, ktime_t maf);
};

struct uart_tx
{
	char name[TX_NAME_MAX];
	struct uart_frame frame;
	struct task_struct *thread;
	struct mutex lock;
	struct hrtimer timer;
	struct uart_tx_ops ops;
	struct tty_struct *tty;
	bool compliant;
};

void 	rx_process (struct uart_frame *frame, uint8_t ch, uint8_t flag);
void 	tx_transmit (struct uart_tx *tx, uint8_t *data, size_t size);

int 	tx_attach (struct uart_tx *tx, struct tty_struct *tty);
void 	tx_detach (struct uart_tx *tx);

#endif /* INCLUDE_UART_RXTX_H_ */
