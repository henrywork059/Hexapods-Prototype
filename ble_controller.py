# ble_uart_lib.py
# Firmware-grade BLE UART library (NUS)
# Pico W / Pico 2 W
# Android 12+ safe, memory-safe, IRQ-safe

import bluetooth
import struct
from micropython import const

# =============================
# CONSTANTS
# =============================

_IRQ_CENTRAL_CONNECT    = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE        = const(3)

UART_SERVICE_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
UART_TX_UUID      = bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E")
UART_RX_UUID      = bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E")

PKT_LEN = const(6)
PKT_HEADER = const(0xAA)

# =============================
# BLE UART CLASS
# =============================

class BLEUART:
    def __init__(self, name="Hexa24"):
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self._irq)

        self.name = name
        self.conn = None
        self.restart_adv = False

        # Fixed RX packet buffer (NO dynamic allocation)
        self.packet = bytearray(PKT_LEN)
        self.packet_ready = False

        # Stats
        self.packet_count = 0
        self.bad_packet_count = 0

        self._register_services()
        self._advertise()

    # =============================
    # SETUP
    # =============================

    def _register_services(self):
        uart_service = (
            UART_SERVICE_UUID,
            (
                (UART_TX_UUID, bluetooth.FLAG_NOTIFY),
                (UART_RX_UUID, bluetooth.FLAG_WRITE | bluetooth.FLAG_WRITE_NO_RESPONSE),
            ),
        )

        ((self.tx_handle, self.rx_handle),) = self.ble.gatts_register_services(
            (uart_service,)
        )

        # Large RX buffer for Android
        self.ble.gatts_set_buffer(self.rx_handle, 64, True)

    # =============================
    # ADVERTISING
    # =============================

    def _adv_payload(self, name=None, services=None):
        payload = bytearray()

        def add(t, v):
            payload.extend(struct.pack("BB", len(v) + 1, t))
            payload.extend(v)

        if name:
            add(0x09, name.encode())

        if services:
            for uuid in services:
                b = bytes(uuid)
                add(0x07 if len(b) == 16 else 0x03, b)

        return payload

    def _advertise(self):
        adv = self._adv_payload(name=self.name)
        resp = self._adv_payload(services=[UART_SERVICE_UUID])

        self.ble.gap_advertise(
            100_000,
            adv_data=adv,
            resp_data=resp
        )

    # =============================
    # IRQ (NO ALLOCATION)
    # =============================

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            self.conn, _, _ = data

            # Android notify wake
            try:
                self.ble.gatts_notify(self.conn, self.tx_handle, b'\x00')
            except:
                pass

        elif event == _IRQ_CENTRAL_DISCONNECT:
            self.conn = None
            self.restart_adv = True

        elif event == _IRQ_GATTS_WRITE:
            _, handle = data
            if handle == self.rx_handle:
                d = self.ble.gatts_read(self.rx_handle)
                if d and len(d) == PKT_LEN:
                    # Fixed buffer copy
                    self.packet[:] = d
                    self.packet_ready = True

    # =============================
    # PUBLIC API
    # =============================

    def poll(self):
        """Must be called regularly from main loop"""
        if self.restart_adv:
            self.restart_adv = False
            self._advertise()

    def get_packet(self):
        """Return latest valid packet or None"""
        if not self.packet_ready:
            return None

        self.packet_ready = False
        self.packet_count += 1

        # Validate packet
        if self.packet[0] != PKT_HEADER:
            self.bad_packet_count += 1
            return None

        chk = (self.packet[1] + self.packet[2] +
               self.packet[3] + self.packet[4]) & 0xFF
        if chk != self.packet[5]:
            self.bad_packet_count += 1
            return None

        return self.packet

    def notify(self, data):
        if self.conn is not None:
            try:
                self.ble.gatts_notify(self.conn, self.tx_handle, data)
            except:
                pass