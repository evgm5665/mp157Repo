#!/usr/bin/env python3
"""DK1 app entry point, started by mp157-app.service after every pull of main.
Hands over to the puck app in puck/dk1/run.sh."""
import os
here = os.path.dirname(os.path.abspath(__file__))
os.chdir(here)
os.execvp("sh", ["sh", os.path.join(here, "puck", "dk1", "run.sh")])
