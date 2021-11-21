//
// Created by Chen Qian on 11/17/21.
//

#include "VCP.h"

USBInEndpointState VCP::ep1instate;
USBInEndpointState VCP::ep2instate;
USBOutEndpointState VCP::ep1outstate;

SerialUSBDriver *VCP::SDU;

const uint8_t VCP::device_descriptor_data[18] = {
        USB_DESC_DEVICE(0x0110,
                        0x02,
                        0x00,
                        0x00,
                        0x40,
                        0x0483,
                        0x5740,
                        0x0200,
                        1,
                        2,
                        3,
                        1)
};

USBDescriptor VCP::device_descriptor = {
        sizeof device_descriptor_data,
        device_descriptor_data
};

const uint8_t VCP::configuration_descriptor_data[67] = {
        /* Configuration Descriptor.*/
        USB_DESC_CONFIGURATION(67,            /* wTotalLength.                    */
                               0x02,          /* bNumInterfaces.                  */
                               0x01,          /* bConfigurationValue.             */
                               0,             /* iConfiguration.                  */
                               0xC0,          /* bmAttributes (self powered).     */
                               50),           /* bMaxPower (100mA).               */
        /* Interface Descriptor.*/
        USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                               0x00,          /* bAlternateSetting.               */
                               0x01,          /* bNumEndpoints.                   */
                               0x02,          /* bInterfaceClass (Communications
                                                 Interface Class, CDC section
                                                 4.2).                            */
                               0x02,          /* bInterfaceSubClass (Abstract
                                               Control Model, CDC section 4.3).   */
                               0x01,          /* bInterfaceProtocol (AT commands,
                                                 CDC section 4.4).                */
                               0),            /* iInterface.                      */
        /* Header Functional Descriptor (CDC section 5.2.3).*/
        USB_DESC_BYTE         (5),            /* bLength.                         */
        USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
        USB_DESC_BYTE         (0x00),         /* bDescriptorSubtype (Header
                                                 Functional Descriptor.           */
        USB_DESC_BCD          (0x0110),  /* bcdCDC.                          */
        /* Call Management Functional Descriptor. */
        USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
        USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
        USB_DESC_BYTE         (0x01),         /* bDescriptorSubtype (Call Management
                                                 Functional Descriptor).          */
        USB_DESC_BYTE         (0x00),         /* bmCapabilities (D0+D1).          */
        USB_DESC_BYTE         (0x01),         /* bDataInterface.                  */
        /* ACM Functional Descriptor.*/
        USB_DESC_BYTE         (4),            /* bFunctionLength.                 */
        USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
        USB_DESC_BYTE         (0x02),         /* bDescriptorSubtype (Abstract
                                                 Control Management Descriptor).  */
        USB_DESC_BYTE         (0x02),         /* bmCapabilities.                  */
        /* Union Functional Descriptor.*/
        USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
        USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
        USB_DESC_BYTE         (0x06),         /* bDescriptorSubtype (Union
                                                 Functional Descriptor).          */
        USB_DESC_BYTE         (0x00),         /* bMasterInterface (Communication
                                                 Class Interface).                */
        USB_DESC_BYTE         (0x01),         /* bSlaveInterface0 (Data Class
                                                 Interface).                      */
        /* Endpoint 2 Descriptor.*/
        USB_DESC_ENDPOINT     (USBD_INTERRUPT_REQUEST_EP | 0x80,
                               0x03,          /* bmAttributes (Interrupt).        */
                               0x0008,        /* wMaxPacketSize.                  */
                               0xFF),         /* bInterval.                       */
        /* Interface Descriptor.*/
        USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
                               0x00,          /* bAlternateSetting.               */
                               0x02,          /* bNumEndpoints.                   */
                               0x0A,          /* bInterfaceClass (Data Class
                                                 Interface, CDC section 4.5).     */
                               0x00,          /* bInterfaceSubClass (CDC section
                                                 4.6).                            */
                               0x00,          /* bInterfaceProtocol (CDC section
                                                 4.7).                            */
                               0x00),         /* iInterface.                      */
        /* Endpoint 3 Descriptor.*/
        USB_DESC_ENDPOINT     (USBD_DATA_AVAILABLE_EP,        /* bEndpointAddress.*/
                               0x02,          /* bmAttributes (Bulk).             */
                               0x0040,        /* wMaxPacketSize.                  */
                               0x00),         /* bInterval.                       */
        /* Endpoint 1 Descriptor.*/
        USB_DESC_ENDPOINT     (USBD_DATA_REQUEST_EP | 0x80,   /* bEndpointAddress.*/
                               0x02,          /* bmAttributes (Bulk).             */
                               0x0040,        /* wMaxPacketSize.                  */
                               0x00)          /* bInterval.                       */
};

USBDescriptor VCP::configuration_descriptor = {
        sizeof configuration_descriptor_data,
        configuration_descriptor_data
};

const uint8_t VCP::vcom_string0[] = {
        USB_DESC_BYTE(4),                     /* bLength.                         */
        USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
        USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

const uint8_t VCP::vcom_string1[] = {USB_DESC_BYTE(38),                    /* bLength.                         */
                                     USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
                                     'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
                                     'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
                                     'c', 0, 's', 0
};

const uint8_t VCP::vcom_string2[] = {
        USB_DESC_BYTE(56),                    /* bLength.                         */
        USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
        'C', 0, 'h', 0, 'i', 0, 'b', 0, 'i', 0, 'O', 0, 'S', 0, '/', 0,
        'R', 0, 'T', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0,
        'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0,
        'o', 0, 'r', 0, 't', 0
};

const uint8_t VCP::vcom_string3[] = {
        USB_DESC_BYTE(8),                     /* bLength.                         */
        USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
        '0' + CH_KERNEL_MAJOR, 0,
        '0' + CH_KERNEL_MINOR, 0,
        '0' + CH_KERNEL_PATCH, 0
};

const USBEndpointConfig VCP::ep1config = {
        USB_EP_MODE_TYPE_BULK,
        NULL,
        sduDataTransmitted,
        sduDataReceived,
        0x0040,
        0x0040,
        &ep1instate,
        &ep1outstate,
        2,
        NULL
};

const USBEndpointConfig VCP::ep2config = {
        USB_EP_MODE_TYPE_INTR,
        NULL,
        sduInterruptTransmitted,
        NULL,
        0x0010,
        0x0000,
        &ep2instate,
        NULL,
        1,
        NULL
};

const USBDescriptor VCP::vcom_strings[] = {
    {sizeof VCP::vcom_string0, VCP::vcom_string0},
    {sizeof VCP::vcom_string1, VCP::vcom_string1},
    {sizeof VCP::vcom_string2, VCP::vcom_string2},
    {sizeof VCP::vcom_string3, VCP::vcom_string3}
};

const USBConfig VCP::usbcfg = {
        usb_event,
        get_descriptor,
        sduRequestsHook,
        sof_handler
};

SerialUSBConfig VCP::serusbcfg = {
        &USBD1,
        USBD_DATA_REQUEST_EP,
        USBD_DATA_AVAILABLE_EP,
        USBD_INTERRUPT_REQUEST_EP
};

void VCP::init(SerialUSBDriver *SDU_) {
    SDU = SDU_;
    sduObjectInit(SDU);
    sduStart(SDU, &serusbcfg);

    usbDisconnectBus(serusbcfg.usbp);
    chThdSleepMilliseconds(1500);
    usbStart(serusbcfg.usbp, &usbcfg);
    usbConnectBus(serusbcfg.usbp);
}

const USBDescriptor *VCP::get_descriptor(USBDriver *usbp, uint8_t dtype, uint8_t dindex, uint16_t lang) {
    (void)usbp;
    (void)lang;
    switch (dtype) {
        case USB_DESCRIPTOR_DEVICE:
            return &device_descriptor;
        case USB_DESCRIPTOR_CONFIGURATION:
            return &configuration_descriptor;
        case USB_DESCRIPTOR_STRING:
            if (dindex < 4)
                return &vcom_strings[dindex];
    }
    return NULL;
}

void VCP::usb_event(USBDriver *usbp, usbevent_t event) {
    switch (event) {
        case USB_EVENT_RESET:
            return;
        case USB_EVENT_ADDRESS:
            return;
        case USB_EVENT_CONFIGURED:
            chSysLockFromISR();

            /* Enables the endpoints specified into the configuration.
               Note, this callback is invoked from an ISR so I-Class functions
               must be used.*/
            usbInitEndpointI(usbp, USBD_DATA_REQUEST_EP, &ep1config);
            usbInitEndpointI(usbp, USBD_INTERRUPT_REQUEST_EP, &ep2config);

            /* Resetting the state of the CDC subsystem.*/
            sduConfigureHookI(SDU);

            chSysUnlockFromISR();
            return;
        case USB_EVENT_SUSPEND:
            chSysLockFromISR();

            /* Disconnection event on suspend.*/
            sduSuspendHookI(SDU);

            chSysUnlockFromISR();
            return;
        case USB_EVENT_WAKEUP:
            return;
        case USB_EVENT_STALLED:
            return;
        case USB_EVENT_UNCONFIGURED:
            return;
    }

}

void VCP::sof_handler(USBDriver *usbp) {
    (void)usbp;
    osalSysLockFromISR();
    sduSOFHookI(SDU);
    osalSysUnlockFromISR();
}

