"""D04 finite rule grammar and ambiguity-aware symbolic companion (copied from robo-use `patterns.py`).

Grid coordinates are public symbolic task data, not simulator object state.
The physical D04 scene/agent evaluation is not implemented by this module.
"""
import random
from dataclasses import asdict, dataclass
from itertools import product

GRID_SIZE = 5
LABELS = ("red_circle", "blue_square", "green_triangle")


@dataclass(frozen=True)
class Rule:
    quarter_turns: int
    reflect_x: bool
    dx: int
    dy: int

    def __post_init__(self):
        if (type(self.quarter_turns) is not int or self.quarter_turns not in range(4)
                or type(self.reflect_x) is not bool
                or any(type(v) is not int or v not in (-1, 0, 1) for v in (self.dx, self.dy))):
            raise ValueError("Rule outside the declared finite grammar")

    def apply(self, pattern):
        validate_pattern(pattern)
        result = {}
        for label, (x, y) in pattern.items():
            if self.reflect_x:
                x = GRID_SIZE-1-x
            for _ in range(self.quarter_turns):
                x, y = GRID_SIZE-1-y, x
            x, y = x+self.dx, y+self.dy
            if not 0 <= x < GRID_SIZE or not 0 <= y < GRID_SIZE:
                return None
            result[label] = [x, y]
        return result


GRAMMAR = tuple(Rule(turns, reflect, dx, dy)
                for turns, reflect, dx, dy in product(range(4), (False, True), range(-1, 2), range(-1, 2)))
# Development exposes single operations; test holds out geometric compositions.
DEVELOPMENT_RULES = tuple(r for r in GRAMMAR if (r.quarter_turns != 0)+(r.reflect_x)+(r.dx != 0 or r.dy != 0) <= 1)
HELDOUT_RULES = tuple(r for r in GRAMMAR if r not in DEVELOPMENT_RULES)


def validate_pattern(pattern):
    if not isinstance(pattern, dict) or set(pattern) != set(LABELS):
        raise ValueError("Expected exactly the three visible shape/color labels")
    cells = []
    for cell in pattern.values():
        if not isinstance(cell, (list, tuple)) or len(cell) != 2:
            raise ValueError("Expected two grid coordinates")
        if any(type(v) is not int or not 0 <= v < GRID_SIZE for v in cell):
            raise ValueError("Grid coordinates must be integers inside the board")
        cells.append(tuple(cell))
    if len(set(cells)) != len(cells):
        raise ValueError("Two objects cannot occupy one cell")


def consistent_rules(examples):
    if not isinstance(examples, list) or not examples:
        raise ValueError("At least one demonstration is required")
    for example in examples:
        if set(example) != {"input", "output"}:
            raise ValueError("Unexpected demonstration fields")
        validate_pattern(example["input"])
        validate_pattern(example["output"])
    return tuple(rule for rule in GRAMMAR
                 if all((output := rule.apply(example["input"])) is not None
                        and _signature(output) == _signature(example["output"]) for example in examples))


def _signature(pattern):
    return tuple(tuple(pattern[label]) for label in LABELS)


def consistent_answers(examples, query):
    """All distinct in-bounds query answers consistent with the public evidence."""
    validate_pattern(query)
    answers = {}
    for rule in consistent_rules(examples):
        output = rule.apply(query)
        if output is not None:
            signature = _signature(output)
            answers[signature] = output
    return list(answers.values())


def score_symbolic(examples, query, answer):
    try:
        validate_pattern(answer)
    except ValueError:
        return False
    return any(_signature(answer) == _signature(valid) for valid in consistent_answers(examples, query))


def generate_instance(seed, split="development"):
    if split not in ("development", "heldout"):
        raise ValueError("Unknown structural split")
    if type(seed) is not int or seed < 0:
        raise ValueError("Seed must be a nonnegative integer")
    rng = random.Random(seed)
    rule = rng.choice(DEVELOPMENT_RULES if split == "development" else HELDOUT_RULES)
    cells = list(product(range(GRID_SIZE), repeat=2))

    def sample():
        for _ in range(1000):
            pattern = {label: list(cell) for label, cell in zip(LABELS, rng.sample(cells, len(LABELS)))}
            if rule.apply(pattern) is not None:
                return pattern
        raise RuntimeError("Could not sample an in-bounds pattern")

    query = sample()
    examples = []
    for _ in range(10):
        pattern = sample()
        examples.append({"input": pattern, "output": rule.apply(pattern)})
        answers = consistent_answers(examples, query)
        if len(answers) == 1:
            return {"schema_version": "D04-symbolic-0.1-dev", "seed": seed, "split": split,
                    "public": {"grid_size": GRID_SIZE, "labels": list(LABELS),
                               "examples": examples, "query": query,
                               "rule_grammar": "reflect-x optionally, then0..3 quarter-turns, then translation by -1..1 cells per axis"},
                    "private": {"author_rule": asdict(rule), "consistent_answers": answers},
                    "artifact_kind": "symbolic_companion_only", "physical_evidence_included": False,
                    "release_ready": False}
    raise RuntimeError("Could not generate an identifiable query answer")
