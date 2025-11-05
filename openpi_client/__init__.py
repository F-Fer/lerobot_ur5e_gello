"""Local lightweight copy of the OpenPI client utilities.

This package vendored selected modules from the openpi-client project
to avoid its strict NumPy <2 requirement. The original code is licensed
under Apache-2.0 and available at:
https://github.com/Physical-Intelligence/openpi
"""

__version__ = "0.1.0"

from .websocket_client_policy import WebsocketClientPolicy  # noqa: F401

