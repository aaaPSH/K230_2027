"""Communication interface parameters.

The default implementation intentionally does not bind to UART/CAN/SPI.
Replace the classes in comm.py or subclass them when the wire protocol is fixed.
"""

COMM_CONFIG = {
    "enabled": False,
    "debug_print": False,
}
