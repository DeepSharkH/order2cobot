"""Shared robot execution for every task on the UR3e: URScript steps through the AI Agent
Bridge (urexec), the generic insertion primitives (urlib), the URScript lint, and run logging
(runlog, telemetry, record, session_trace). Task code (porttest/ = switch, breadboard/) builds
on it; it never imports task code."""
