#!/bin/bash

# Convenience wrapper to run Python scripts with the correct architecture and Python

arch -x86_64 env PYTHONPATH="" /Library/Frameworks/Python.framework/Versions/3.11/bin/python3 "$@"
