---
schema_version: '1.3'
task:
  name: robouse/behavior-sorting-books-on-shelf
  description: 'Keep every comic book, notebook and hardback inside the bookcase, and stack them by type: at least two comic books each on top of another comic book, one notebook on top of the other notebook, and one hardback on top of the other hardback.'
metadata:
  author_name: benchflow
  source_benchmark: BEHAVIOR-1K (Li et al. 2022/2024), 2026 BEHAVIOR Challenge task list; OmniGibson 3.9.3 on Isaac Sim 5.1
  source_task: sorting_books_on_shelf
  suite: behavior
  behavior_scene: house_double_floor_upper
  behavior_instance: 0
  behavior_challenge_task_id: 99
  challenge_instruction: Sort the comic books, notebooks, and hardbacks on the bookcase shelves so the books of each type are grouped together.
  category: household-long-horizon
  difficulty: medium
  language_instruction: 'Keep every comic book, notebook and hardback inside the bookcase, and stack them by type: at least two comic books each on top of another comic book, one notebook on top of the other notebook, and one hardback on top of the other hardback.'
  tags:
  - behavior-1k
  - omnigibson
  - isaac-sim
  - r1pro
  - mobile-manipulation
  - symbolic-skills
  - remote-gpu
  - gpu-required
  requires_gpu: NVIDIA RTX GPU with RT cores, for the remote simulator worker
  reference_solution: scripted skill plan, 9 skills (oracle/plan.json)
  robouse:
    id: behavior-sorting-books-on-shelf
    backend: behavior
    env: sorting_books_on_shelf
    scene: house_double_floor_upper
    rooms:
    - living_room_0
    instance: 0
    seed: 0
    max_steps: 35
    camera: follow
    cameras:
    - follow
    - head
    skills: true
    success_mode: final
    obs_mode: state
    frame_every: 1
    linger_s: 5
    ready_timeout_s: 900
    rpc_timeout_s: 600
agent:
  timeout_sec: 1800
verifier:
  service: simulator
  user: root
  timeout_sec: 300
sandbox:
  cpus: 1
  memory_mb: 2048
  build_timeout_sec: 3600
---

You are controlling a simulated robot. Read the task below, then solve it by running the `robo` command in your shell (start with `robo info` and `robo observe`). Keep going until the task is done, then call `robo done` once. Do not stop to ask questions; there is no human to answer.

# BEHAVIOR: sorting books on shelf

Keep every comic book, notebook and hardback inside the bookcase, and stack them by type: at least two comic books each on top of another comic book, one notebook on top of the other notebook, and one hardback on top of the other hardback.

This is the BEHAVIOR-1K activity `sorting_books_on_shelf` in the `house_double_floor_upper` scene (rooms loaded: living_room_0), BEHAVIOR Challenge task instance 0. The robot starts where the challenge places it for that instance. The step budget is 35 skill calls.

BEHAVIOR's formal goal (BDDL; `?x` are variables, `_1`, `_2` number objects of one category) is:

```
(:goal 
        (and 
            (forall 
                (?comic_book.n.01 - comic_book.n.01)
                (inside ?comic_book.n.01 ?bookcase.n.01_1)
            )
            (forn
                (2)
                (?comic_book.n.01 - comic_book.n.01)
                (or 
                    (ontop ?comic_book.n.01 ?comic_book.n.01_1)
                    (ontop ?comic_book.n.01 ?comic_book.n.01_2)
                    (ontop ?comic_book.n.01 ?comic_book.n.01_3)
                )
            )
            (forall 
                (?notebook.n.01 - notebook.n.01)
                (inside ?notebook.n.01 ?bookcase.n.01_1)
            )
            (exists
                (?notebook.n.01 - notebook.n.01)
                (or 
                    (ontop ?notebook.n.01 ?notebook.n.01_1)
                    (ontop ?notebook.n.01 ?notebook.n.01_2)
                )
            )
            (forall 
                (?hardback.n.01 - hardback.n.01)
                (inside ?hardback.n.01 ?bookcase.n.01_1)
            )
            (exists
                (?hardback.n.01 - hardback.n.01)
                (or 
                    (ontop ?hardback.n.01 ?hardback.n.01_1)
                    (ontop ?hardback.n.01 ?hardback.n.01_2)
                )
            )
        )
    )
```

## How to control the robot

You are the robot's high-level policy. You act only through the `robo` command in your shell; you cannot read or change the simulator, the scoring, or other files to succeed. The episode server checks BEHAVIOR's goal conditions on the final simulated state itself.

```
robo info                          # skills, step budget
robo observe                       # robot pose, held object, and every task object (name, category, room, position, distance, open/on state, inside/on-top relations)
robo observe --image               # also saves a third-person camera image and prints its path (open it to look)
robo observe --image --camera head # the robot's own head camera instead
robo skill navigate_to OBJ         # drive next to OBJ (a held object comes along)
robo skill grasp OBJ               # pick up OBJ (hand must be empty, OBJ within reach); things resting in or on OBJ come along
robo skill place_on_top OBJ        # put the held object on top of OBJ
robo skill place_inside OBJ        # put the held object inside OBJ (open OBJ first if it has a door or lid)
robo skill open OBJ                # open OBJ (hand must be empty)
robo skill close OBJ               # close OBJ (hand must be empty)
robo skill toggle_on OBJ           # switch OBJ on (hand must be empty)
robo skill toggle_off OBJ          # switch OBJ off (hand must be empty)
robo skill release                 # drop the held object where the hand is
robo done "short summary"          # end the episode and ask for scoring
robo give-up "reason"              # end the episode without claiming success
```

- OBJ is an object name exactly as `robo observe` lists it (BEHAVIOR's BDDL instance names, e.g. `bottle.n.01_1`). Identical objects are numbered `_1`, `_2`, ...
- The robot is an R1Pro wheeled mobile manipulator that carries one object at a time. Manipulation skills only work on objects within reach (about 1.25 m from the robot base); use `navigate_to` first. Objects inside a closed container must be taken out after opening it.
- A skill can fail (the response says why, with `ok: False`); a failed skill still costs one step. Every skill call counts against the step budget (`robo info`).
- Success is judged after you call `robo done`, with the robot holding still: every goal condition must hold at that moment. Call `robo done` exactly once when finished.
