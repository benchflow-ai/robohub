"""Real-robot support: drivers (LeRobot arms, Piper, hardware-in-the-loop mock), safety guard, operator channel, cameras.

The episode server treats a real arm like any other backend (`backends/real.py`); the agent-facing `robo` interface is
unchanged. See docs/real-robots.md.
"""
