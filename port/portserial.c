/*
 * FreeModbus Libary: RT-Thread Port
 * Copyright (C) 2013 Armink <armink.ztl@gmail.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.60 2013/08/13 15:07:05 Armink $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

#ifdef RT_USING_DEVICE
#include "rtdevice.h"
#include "board.h"
#else
#include "gconf.h"
#endif
/* ----------------------- Static variables ---------------------------------*/
#ifdef RT_USING_DEVICE
#endif
ALIGN(RT_ALIGN_SIZE)
/* software simulation serial transmit IRQ handler thread stack */
static rt_uint8_t serial_soft_trans_irq_stack[512];
/* software simulation serial transmit IRQ handler thread */
static struct rt_thread thread_serial_soft_trans_irq;
/* serial event */
static struct rt_event event_serial;
/* modbus slave serial device */
#ifdef RT_USING_DEVICE
static rt_serial_t *serial;
#endif

/* ----------------------- Defines ------------------------------------------*/
/* serial transmit event */
#define EVENT_SERIAL_TRANS_START    (1<<0)
#define RT_MODBUS_SLAVE_USE_CONTROL_PIN

/* ----------------------- static functions ---------------------------------*/
#ifdef RT_USING_DEVICE
static rt_err_t serial_rx_ind(rt_device_t dev, rt_size_t size);
static void prvvUARTRxISR(void);
#endif
static void prvvUARTTxReadyISR(void);
static void serial_soft_trans_irq(void* parameter);

/* ----------------------- Start implementation -----------------------------*/
BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits,
        eMBParity eParity)
{
#ifndef RT_USING_DEVICE
	mcu_uart_open(MCU_UART_2);

    /* software initialize */
    rt_event_init(&event_serial, "slave event", RT_IPC_FLAG_PRIO);
    rt_thread_init(&thread_serial_soft_trans_irq,
                   "slave trans",
                   serial_soft_trans_irq,
                   RT_NULL,
                   serial_soft_trans_irq_stack,
                   sizeof(serial_soft_trans_irq_stack),
                   10, 5);
    rt_thread_startup(&thread_serial_soft_trans_irq);

#else

    rt_device_t dev = RT_NULL;
    char uart_name[20];
    /**
     * set 485 mode receive and transmit control IO
     * @note MODBUS_SLAVE_RT_CONTROL_PIN_INDEX need be defined by user
     */
#if defined(RT_MODBUS_SLAVE_USE_CONTROL_PIN)
    rt_pin_mode(MODBUS_SLAVE_RT_CONTROL_PIN_INDEX, PIN_MODE_OUTPUT);
#endif
    /* set serial name */
    rt_snprintf(uart_name,sizeof(uart_name), "uart%d", ucPORT);

    dev = rt_device_find(uart_name);
    if(dev == RT_NULL)
    {
        /* can not find uart */
        return FALSE;
    }
    else
    {
        serial = (struct rt_serial_device*)dev;
    }

    /* set serial configure parameter */
    serial->config.baud_rate = ulBaudRate;
    serial->config.stop_bits = STOP_BITS_1;
    switch(eParity){
    case MB_PAR_NONE: {
        serial->config.data_bits = DATA_BITS_8;
        serial->config.parity = PARITY_NONE;
        break;
    }
    case MB_PAR_ODD: {
        serial->config.data_bits = DATA_BITS_9;
        serial->config.parity = PARITY_ODD;
        break;
    }
    case MB_PAR_EVEN: {
        serial->config.data_bits = DATA_BITS_9;
        serial->config.parity = PARITY_EVEN;
        break;
    }
    }
    /* set serial configure */
    serial->ops->configure(serial, &(serial->config));

    /* open serial device */
    if (!rt_device_open(&serial->parent, RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX)) {
        rt_device_set_rx_indicate(&serial->parent, serial_rx_ind);
    } else {
        return FALSE;
    }

    /* software initialize */
    rt_event_init(&event_serial, "slave event", RT_IPC_FLAG_PRIO);
    rt_thread_init(&thread_serial_soft_trans_irq,
                   "slave trans",
                   serial_soft_trans_irq,
                   RT_NULL,
                   serial_soft_trans_irq_stack,
                   sizeof(serial_soft_trans_irq_stack),
                   10, 5);
    rt_thread_startup(&thread_serial_soft_trans_irq);
#endif
    return TRUE;
}

void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
#ifndef RT_USING_DEVICE
    rt_uint32_t recved_event;
    if (xRxEnable)
    {
        /* enable RX interrupt */
		UART_INTRXThresholdEn( UART1 );
        /* switch 485 to receive mode */
#if defined(RT_MODBUS_SLAVE_USE_CONTROL_PIN)
		sIoStatus("ren",0);
#endif
    }
    else
    {
        /* switch 485 to transmit mode */
#if defined(RT_MODBUS_SLAVE_USE_CONTROL_PIN)
		sIoStatus("ren",1);
#endif
        /* disable RX interrupt */
		UART_INTRXThresholdDis( UART1 );
    }
    if (xTxEnable)
    {
        /* start serial transmit */
        rt_event_send(&event_serial, EVENT_SERIAL_TRANS_START);
//		sIoStatus("ren",1);
    }
    else
    {
        /* stop serial transmit */
        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START,
                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0,
                &recved_event);
    }

#else

    rt_uint32_t recved_event;
    if (xRxEnable)
    {
        /* enable RX interrupt */
        serial->ops->control(serial, RT_DEVICE_CTRL_SET_INT, (void *)RT_DEVICE_FLAG_INT_RX);
        /* switch 485 to receive mode */
#if defined(RT_MODBUS_SLAVE_USE_CONTROL_PIN)
        rt_pin_write(MODBUS_SLAVE_RT_CONTROL_PIN_INDEX, PIN_LOW);
#endif
    }
    else
    {
        /* switch 485 to transmit mode */
#if defined(RT_MODBUS_SLAVE_USE_CONTROL_PIN)
        rt_pin_write(MODBUS_SLAVE_RT_CONTROL_PIN_INDEX, PIN_HIGH);
#endif
        /* disable RX interrupt */
        serial->ops->control(serial, RT_DEVICE_CTRL_CLR_INT, (void *)RT_DEVICE_FLAG_INT_RX);
    }
    if (xTxEnable)
    {
        /* start serial transmit */
        rt_event_send(&event_serial, EVENT_SERIAL_TRANS_START);
    }
    else
    {
        /* stop serial transmit */
        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START,
                RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR, 0,
                &recved_event);
    }
#endif
}

void vMBPortClose(void)
{
#ifndef RT_USING_DEVICE
#else
    serial->parent.close(&(serial->parent));
#endif
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
#ifndef RT_USING_DEVICE
	mcu_uart_write(MCU_UART_2, (u8 *)&ucByte, 1);
#else
    serial->parent.write(&(serial->parent), 0, &ucByte, 1);
#endif
    return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR * pucByte)
{
#ifndef RT_USING_DEVICE
	mcu_uart_read (MCU_UART_2, (u8 *)pucByte, 1);
#else
    serial->parent.read(&(serial->parent), 0, pucByte, 1);
#endif
    return TRUE;
}

/* 
 * Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
#ifdef RT_USING_DEVICE
#endif
void prvvUARTTxReadyISR(void)
{
    pxMBFrameCBTransmitterEmpty();
}

/* 
 * Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
#ifdef RT_USING_DEVICE
#endif
void prvvUARTRxISR(void)
{
    pxMBFrameCBByteReceived();
}

/**
 * Software simulation serial transmit IRQ handler.
 *
 * @param parameter parameter
 */
#ifdef RT_USING_DEVICE
#endif
static void serial_soft_trans_irq(void* parameter) {
    rt_uint32_t recved_event;
    while (1)
    {
        /* waiting for serial transmit start */
        rt_event_recv(&event_serial, EVENT_SERIAL_TRANS_START, RT_EVENT_FLAG_OR,
                RT_WAITING_FOREVER, &recved_event);
        /* execute modbus callback */
        prvvUARTTxReadyISR();
    }
}

/**
 * This function is serial receive callback function
 *
 * @param dev the device of serial
 * @param size the data size that receive
 *
 * @return return RT_EOK
 */
#ifdef RT_USING_DEVICE
static rt_err_t serial_rx_ind(rt_device_t dev, rt_size_t size) {
    while(size--)
        prvvUARTRxISR();
    return RT_EOK;
}
#endif
