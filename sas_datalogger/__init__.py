"""SAS datalogger package.

This package exposes a client class for interacting with the SAS datalogger.

Exports:
- DataloggerClient: client binding to the underlying C++/Python datalogger
  implementation.
"""

from sas_datalogger._sas_datalogger import DataloggerClient
