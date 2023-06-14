# esphome-jbd-bms

![GitHub actions](https://github.com/syssi/esphome-jbd-bms/actions/workflows/ci.yaml/badge.svg)
![GitHub stars](https://img.shields.io/github/stars/syssi/esphome-jbd-bms)
![GitHub forks](https://img.shields.io/github/forks/syssi/esphome-jbd-bms)
![GitHub watchers](https://img.shields.io/github/watchers/syssi/esphome-jbd-bms)
[!["Buy Me A Coffee"](https://img.shields.io/badge/buy%20me%20a%20coffee-donate-yellow.svg)](https://www.buymeacoffee.com/syssi)

ESPHome component to monitor and control a JBD-BMS via UART-TTL 

WORK IN PROGRESS

## Supported devices

* JBD-UP16S010-L16S-200A-200A-B-U-R-C-A03

## Schematics

```
                RS485-TTL
┌──────────┐                ┌─────────┐
│          │<----- RX ----->│         │
│ JBD-BMS  │<----- TX ----->│ ESP32/  │
│          │<----- GND ---->│ ESP8266 │<-- 3.3V
│          │                │         │<-- GND
└──────────┘                └─────────┘
 

Connector 4 Pin, JST PA 2.0mm pitch

## Installation

You can install this component with [ESPHome external components feature](https://esphome.io/components/external_components.html) like this:
```yaml
external_components:
  - source: github://syssi/esphome-jbd-bms@main
```

or just use the `esp32-example.yaml` as proof of concept:

```bash
# Install esphome
pip3 install esphome

# Clone this external component
git clone https://github.com/syssi/esphome-jbd-bms.git
cd esphome-jbd-bms

# Create a secrets.yaml containing some setup specific secrets
cat > secrets.yaml <<EOF
wifi_ssid: MY_WIFI_SSID
wifi_password: MY_WIFI_PASSWORD

mqtt_host: MY_MQTT_HOST
mqtt_username: MY_MQTT_USERNAME
mqtt_password: MY_MQTT_PASSWORD
EOF

# Validate the configuration, create a binary, upload it, and start logs
# If you use a esp8266 run the esp8266-examle.yaml
esphome run esp32-example.yaml

```

## Example response all sensors enabled

 
## Protocol

See [ ]

## Known issues

None.

## Goodies

## Debugging

If this component doesn't work out of the box for your device please update your configuration to enable the debug output of the UART component and increase the log level to the see outgoing and incoming serial traffic:

```
logger:
  level: DEBUG

uart:
  id: uart_0
  baud_rate: 9600
  tx_pin: GPIO4
  rx_pin: GPIO5
  debug:
    direction: BOTH
```

## References

* https://github.com/ioBroker/AdapterRequests/issues/512
* https://github.com/sshoecraft/jbdtool/blob/main/jbd.c
* https://gitlab.com/bms-tools/bms-tools
* https://github.com/kolins-cz/Smart-BMS-Bluetooth-ESP32
* https://github.com/ForrestFire0/GenericBMSArduino
