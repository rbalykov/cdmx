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

struct uart_tx_ops
{
	/*
	 * Init data and start transmitting
	 */
	int  	(*start) 	(struct uart_tx *tx);

	/*
	 * Stop transmitting
	 */
	void 	(*stop) 	(struct uart_tx *tx);

	/*
	 * Set BREAK state on/off
	 */
	int		(*break_ctl) (struct uart_tx *tx, bool on);

	/*
	 * Send single frame
	 */
	int		(*send) 	(struct uart_tx *tx);

	/*
	 * Get frame from outer space
	 */
	size_t	(*read)		(struct uart_tx *tx, uint8_t *data, size_t size);

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

int 	tx_attach (struct uart_tx *tx, struct tty_struct *tty);
void 	tx_detach (struct uart_tx *tx);

#endif /* INCLUDE_UART_RXTX_H_ */
