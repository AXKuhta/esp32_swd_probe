
#ifndef ARDUINO_USB_MODE
#error This ESP32 SoC has no Native USB interface
#elif ARDUINO_USB_MODE == 1
#warning This sketch should be used when USB is in OTG mode
void setup() {}
void loop() {}
#else
#include "USB.h"
#include "USBHID.h"
#include "USBVendor.h"

#include "DAP_config.h"
#include "DAP.h"

static uint8_t TxDataBuffer[CFG_TUD_HID_EP_BUFSIZE];
static uint8_t RxDataBuffer[CFG_TUD_HID_EP_BUFSIZE];

USBCDC USBSerial; // Provides USB reflash
USBVendor Vendor; // Faster?
USBHID HID;

// https://github.com/raspberrypi/debugprobe/blob/master/src/usb_descriptors.c#L91
static const uint8_t report_descriptor[] = {
  TUD_HID_REPORT_DESC_GENERIC_INOUT(CFG_TUD_HID_EP_BUFSIZE)
};

class CustomHIDDevice : public USBHIDDevice {
public:
  //uint8_t TxDataBuffer[CFG_TUD_HID_EP_BUFSIZE];
  //uint8_t RxDataBuffer[CFG_TUD_HID_EP_BUFSIZE];

  CustomHIDDevice(void) {
    static bool initialized = false;
    if (!initialized) {
      initialized = true;
      HID.addDevice(this, sizeof(report_descriptor));
    }
  }

  void begin(void) {
    HID.begin();
  }

  uint16_t _onGetDescriptor(uint8_t *buffer) {
    memcpy(buffer, report_descriptor, sizeof(report_descriptor));
    return sizeof(report_descriptor);
  }

  bool send(uint8_t *value) {
    return HID.SendReport(0, value, 8);
  }

  void _onOutput(uint8_t report_id, const uint8_t *buffer, uint16_t len) {
    USBSerial.printf("REP\n");
  }

  void _onSetFeature(uint8_t report_id, const uint8_t *buffer, uint16_t len) {
    uint32_t response_size = TU_MIN(CFG_TUD_HID_EP_BUFSIZE, len);

    //USBSerial.printf("SETFEATURE\n");

    // https://github.com/raspberrypi/debugprobe/blob/master/src/main.c#L145
    // https://github.com/myelin/arduino-cmsis-dap/blob/master/arduino-cmsis-dap.ino#L122
    uint32_t sz = DAP_ProcessCommand( (uint8_t*)buffer, TxDataBuffer);

    if (sz) {
      HID.SendReport(0, TxDataBuffer, response_size);
    }

    //USBSerial.printf("OUT\n");
  }
};

CustomHIDDevice Device;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);

  // Change product name so OpenOCD recognizes us
  // https://github.com/myelin/arduino-cmsis-dap/blob/master/arduino-cmsis-dap.ino#L43
  // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/USB.cpp#L312
  // Warning: USB CDC on boot must be disabled
  USB.productName("ESP32 CMSIS-DAP"); 

  // USB setup
  Device.begin();
  Vendor.begin();
  USBSerial.begin();
  USB.begin();
}

void loop() {
  if (Vendor.available()) {
    Vendor.read(RxDataBuffer, sizeof(RxDataBuffer));

    uint32_t sz = DAP_ProcessCommand(RxDataBuffer, TxDataBuffer) & 0xFFFF;

    Vendor.write(TxDataBuffer, sz);
    Vendor.flush();
  }
}
#endif /* ARDUINO_USB_MODE */
