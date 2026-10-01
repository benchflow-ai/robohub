"""Bounded text for messages that are stored or shown (error messages, evidence)."""

from __future__ import annotations


def clip(text: object, limit: int) -> str:
    """`text` cut to at most `limit` characters at a line or word boundary, with " …" when it was cut (never mid-word,
    unless one word is longer than half the limit)."""
    s = str(text).strip()
    if len(s) <= limit:
        return s
    cut = s[: max(1, limit - 2)]
    for sep in ("\n", " "):
        i = cut.rfind(sep)
        if i >= limit // 2:
            cut = cut[:i]
            break
    return cut.rstrip() + " …"
