"""The simulator-worker framework: every simulator that runs outside the episode server's process (its own
virtualenv, or a remote GPU machine) is a worker that robouse calls through `client.StdioWorker` or
`client.HttpWorker`; the worker side is `wire.py` (standard library and numpy only)."""
