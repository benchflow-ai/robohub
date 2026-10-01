"""Robo Use: agent as a policy for embodied agents.

An agent harness (Claude Code, Codex, mini-swe-agent, ...) drives a simulated robot through the `robo`
command. A trusted episode server (`robouse serve`) owns the simulator, the step budget, the recording
and the success check; the agent never touches simulator objects directly.
"""

__version__ = "0.3.0"
