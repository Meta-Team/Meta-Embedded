//
// Created by Chen Qian on 11/17/21.
//

#ifndef META_INFANTRY_VCP_H
#define META_INFANTRY_VCP_H

#include "hal.h"
#include "hal_usb_cdc.h"
#include "shell.h"

#define USBD_DATA_REQUEST_EP           1
#define USBD_DATA_AVAILABLE_EP         1
#define USBD_INTERRUPT_REQUEST_EP      2

/**Inspired by STM32 Demo of ChibiOS*/

/**
 * @brief Virtual COM port for STM32.
 * @usage Please first define "BOARD_OTG_NOVBUSSENS" in board.h, to establish the appropriate communication with PC.
 * @code
 * 1. Enable HAL_USE_USB in halconf.h
 * 2. Enable HAL_USE_USB in halconf.h
 * @endcode
 * */
class VCP {
public:
    static void init(SerialUSBDriver *SDU);
private:
    /********************************************//**
     *        Public Contribution and Methods
     ***********************************************/
private:
    static SerialUSBDriver *SDU;
    /**
     * @brief VCOM Device descriptor data, from ChibiOS STM32F429 Discovery Board Example.
     * @details Detailed information for each parameter
     * @code
     * bcdUSB:              0x0110,     USB full speed (USB 1.1) for 0x110
     * bDeviceClass:        0x02,       Device Class, communications and CDC control for 0x02
     * bDeviceSubClass:     0x00,       Device Subclass code, remains 0
     * bDeviceProtocol:     0x00,       Device does not use class specific protocols
     * bMaxPacketSize:      0x40,       Maximum packet size for Endpoint zero available: 8, 16, 32, 64. 0x40 for 64
     * idVendor:            0x0483,     Vendor assigned by USB-IF, 0x0483 for STMicroelectronics.
     * idProduct:           0x5740,     Product id assigned by manufacturer, 0x5740 for Virtual COM Port.
     * bcdDevice:           0x0200,     Device release number in binary-coded decimal.
     * iManufacturer:       1,          Index of string descriptor describing manufacturer.
     * iProduct:            2,          Index of string descriptor describing product.
     * iSerialNumber:       3,          Index of string descriptor describing the device's serial number.
     * bNumConfigurations:  1,          Number of possible configurations, only one for our device.
     * @endcode
     */
    static const uint8_t device_descriptor_data[18];
    /**
     * @brief USB descriptor
     * @struct size_t device_descriptor size \n
     * uint8_t device_descriptor_data[18]
     */
    static USBDescriptor device_descriptor;

    /**
     * @brief VCOM configuration descriptor data, from ChibiOS STM32F429 Discovery Board Example.
     * @details Detailed information for each parameter.
     * @struct Configuration descriptor
     * @struct Interface descriptor
     * @struct Endpoint descriptor - header functional descriptor
     * @struct Call Management Functional Descriptor
     * @struct ACM Functional Descriptor
     * @struct Union Functional Descriptor
     * @struct Endpoint 2 Descriptor
     * @struct Interface Descriptor
     * @struct Endpoint 3 Descriptor
     * @struct Endpoint 1 Descriptor
     */
    static const uint8_t configuration_descriptor_data[67];

    /**
     * @brief Configuration descriptor for virtual COM port
     */
    static USBDescriptor configuration_descriptor;
    /**
     * U.S. English language identifier.
     */
    static const uint8_t vcom_string0[];

    /**
     * Vendor string.
     */
    static const uint8_t vcom_string1[];

    /**
     * Device Description string.
     */
    static const uint8_t vcom_string2[];

    /**
     * Serial Number string.
     */
    static const uint8_t vcom_string3[];

    /**
     * Strings wrappers array.
     */
    static const USBDescriptor vcom_strings[];

    /**
     * @brief A function to get USB descriptors. Will automatically called by system.
     */
    static const USBDescriptor *get_descriptor(USBDriver *usbp,
                                               uint8_t dtype,
                                               uint8_t dindex,
                                               uint16_t lang);

    /**
     * @brief   IN EP1 state.
     */
    static USBInEndpointState ep1instate;

    /**
     * @brief   OUT EP1 state.
     */
    static USBOutEndpointState ep1outstate;

    /**
     * @brief   EP1 initialization structure (both IN and OUT).
     */
    static const USBEndpointConfig ep1config;

    /**
     * @brief   IN EP2 state.
     */
    static USBInEndpointState ep2instate;

    /**
     * @brief   EP2 initialization structure (IN only).
     */
    static const USBEndpointConfig ep2config;

    static void usb_event(USBDriver *usbp, usbevent_t event);

    /**
     * Handles the USB driver global events.
     */
    static void sof_handler(USBDriver *usbp);

    /**
     * USB driver configuration.
     */
    static const USBConfig usbcfg;

    /**
     * Serial over USB driver configuration.
     */
    static SerialUSBConfig serusbcfg;

    friend Shell;
};


#endif //META_INFANTRY_VCP_H
