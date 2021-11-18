//
// Created by Chen Qian on 11/17/21.
//

#ifndef META_INFANTRY_VCP_H
#define META_INFANTRY_VCP_H

#include "hal.h"
#include "hal_usb_cdc.h"

#define USBD2_DATA_REQUEST_EP           1
#define USBD2_DATA_AVAILABLE_EP         1
#define USBD2_INTERRUPT_REQUEST_EP      2

/**Inspired by STM32 Demo of ChibiOS*/

/**
 * @brief Virtual COM port for STM32.
 * TODO: Figure out why PC could not recognize it.
 * */
class VCP {
public:
    static void init(SerialUSBDriver *SDU);
private:
     /**Configurations */
private:
     static SerialUSBDriver *SDU;
     /**
      * @brief Device descriptor data
      * @details Details information for each parameter
      * @code
      * bcdUSB:             USB, version 1.1.
      * bDeviceClass:       Device Class, Communications and CDC Control.
      * bDeviceSubClass:    Device Subclass code, remains 0.
      * bDeviceProtocol:    Device does not use class specific protocols.
      * bMaxPacketSize:     Maximum packet size for Endpoint zero, 64. available: 8, 16, 32, 64.
      * idVendor:           STMicroelectronics.
      * idProduct:          Virtual COM Port.
      * bcdDevice:          Device release number in binary-coded decimal.
      * iManufacturer:      Index of string descriptor describing manufacturer.
      * iProduct:           Index of string descriptor describing product.
      * iSerialNumber:      Index of string descriptor describing the device's serial number.
      * bNumConfigurations: Number of possible configurations, only one for our device.
      * @endcode
      * */
     static const uint8_t device_descriptor_data[18];
     /**
      * @brief USB descriptor.
      * */
     static USBDescriptor device_descriptor;

     /**
      * @brief From ChibiOS STM32F429 Discovery Board Example.
      * @details
      * @code
      *
      * @endcode
      * */
     static const uint8_t configuration_descriptor_data[67];
     static USBDescriptor configuration_descriptor;
     /*
      * U.S. English language identifier.
      */
     static const uint8_t vcom_string0[];

    /**
     * Vendor string.
     * */
     static const uint8_t vcom_string1[];

    /*
     * Device Description string.
     */
    static const uint8_t vcom_string2[];

    /*
     * Serial Number string.
     */
    static const uint8_t vcom_string3[];
    /*
     * Strings wrappers array.
     */
    static const USBDescriptor vcom_strings[];

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

/*
 * Handles the USB driver global events.
 */
    static void sof_handler(USBDriver *usbp);

/*
 * USB driver configuration.
 */
    static const USBConfig usbcfg;

    /*
     * Serial over USB driver configuration.
     */
    static const SerialUSBConfig serusbcfg;

};


#endif //META_INFANTRY_VCP_H
