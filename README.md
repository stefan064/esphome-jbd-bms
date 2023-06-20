# esphome-jbd-bms

![GitHub actions](https://github.com/syssi/esphome-jbd-bms/actions/workflows/ci.yaml/badge.svg)
![GitHub stars](https://img.shields.io/github/stars/syssi/esphome-jbd-bms)
![GitHub forks](https://img.shields.io/github/forks/syssi/esphome-jbd-bms)
![GitHub watchers](https://img.shields.io/github/watchers/syssi/esphome-jbd-bms)
[!["Buy Me A Coffee"](https://img.shields.io/badge/buy%20me%20a%20coffee-donate-yellow.svg)](https://www.buymeacoffee.com/syssi)

ESPHome component to monitor and control a JBD-UP16S010 via RS485-TTL 

Fork supports parallel functionality of JBD-UP16S010. 
Reading all packs data will is done by one ESP32/ESP8266 interface. 

## Supported devices

* JBD-UP16S010-L16S-200A-200A-B-U-R-C-A03 

## Schematics

```
                RS485-TTL (3.3V)
┌──────────┐                ┌─────────┐
│         1│<----- B- ----->│         │
│ JBD-BMS 2│<----- A+------>│ ESP32/  │
│  RS485  3│<----- GND ---->│ ESP8266 │<-- 3.3V
│   RJ45   │                │         │<-- GND
└──────────┘                └─────────┘


## Installation

You can install this component with [ESPHome external components feature](https://esphome.io/components/external_components.html) like this:
```yaml
external_components:
  - source: github://smaksimowicz/esphome-jbd-bms@main
```

or just use the `esp32-example.yaml` as proof of concept:

```bash
# Install esphome
pip3 install esphome

# Clone this external component
git clone https://github.com/smaksimowicz/esphome-jbd-bms.git
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
```bash
[19:37:33][I][jbd_bms:246]: Hardware info frame (37 bytes) received Modbus Addres 0
[19:37:33][D][jbd_bms:255]:   Device model: 
[19:37:33][D][sensor:094]: 'jbd-bms bms0 total voltage': Sending state 0.00000 V with 2 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 current': Sending state 0.00000 A with 1 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 power': Sending state 0.00000 W with 1 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 charging power': Sending state 0.00000 W with 2 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 discharging power': Sending state 0.00000 W with 2 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 capacity remaining': Sending state 0.00000 Ah with 2 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 nominal capacity': Sending state 200.00000 Ah with 2 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 charging cycles': Sending state 0.00000  with 0 decimals of accuracy
[19:37:33][D][jbd_bms:282]:   Date of manufacture: 2023.5.24
[19:37:33][D][sensor:094]: 'jbd-bms bms0 balancer status bitmask': Sending state 0.00000  with 0 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 errors bitmask': Sending state 10.00000  with 0 decimals of accuracy
[19:37:33][D][text_sensor:064]: 'jbd-bms bms0 errors': Sending state 'Cell undervoltage;Pack undervoltage'
[19:37:33][D][sensor:094]: 'jbd-bms bms0 software version': Sending state 2.30000  with 1 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 state of charge': Sending state 0.00000 % with 0 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 operation status bitmask': Sending state 0.00000  with 0 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 battery strings': Sending state 16.00000  with 0 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 alarm bitmask': Sending state 16384.00000  with 0 decimals of accuracy
[19:37:33][D][text_sensor:064]: 'jbd-bms bms0 alarms': Sending state 'Low capacity alarms'
[19:37:33][D][sensor:094]: 'jbd-bms bms0 temperature ambient': Sending state 27.50000 °C with 1 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 temperature fet': Sending state 26.80000 °C with 1 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 temperature 1': Sending state 150.00000 °C with 1 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 temperature 2': Sending state 150.00000 °C with 1 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 temperature 3': Sending state 150.00000 °C with 1 decimals of accuracy
[19:37:33][D][sensor:094]: 'jbd-bms bms0 temperature 4': Sending state -30.00000 °C with 1 decimals of accuracy
[19:37:33][D][uart_debug:114]: <<< DD:00:03:00:25:00:00:00:00:00:00:4E:20:00:00:2E:B8:00:00:00:00:00:0A:23:00:00:10:40:00:0B:BE:0B:B7:04:10:87:10:87:10:87:09:7F:FA:2B:77
[19:37:34][D][uart_debug:114]: >>> DD:01:A5:03:00:FF:57:77:DD:01:A5:04:00:FF:56:77
[19:37:34][D][uart_debug:114]: >>> DD:02:A5:03:00:FF:56:77:DD:02:A5:04:00:FF:55:77
[19:37:35][D][uart_debug:114]: >>> DD:00:A5:03:00:FF:58:77:DD:00:A5:04:00:FF:57:77
```

## Protocol

See [ https://github.com/smaksimowicz/esphome-jbd-bms/blob/main/docs/Communication%20protocol-QM-UPS1601.pdf ]

## Known issues

None.

## Goodies

Support for external active balancer in HA automation with enable button for instance models like: QUCC-QP-1710A. 

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
