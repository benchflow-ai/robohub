We moved our Trail Walker task (`source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker/`) from Isaac Lab 2.3 to this Isaac Lab 3.0 checkout. Our custom reward terms in `rewards.py` still use the asset data the 2.3 way, and training now prints deprecation warnings about it; we were told that compatibility path will be removed.

Migrate `rewards.py` to Isaac Lab 3.0's asset data API so the four reward functions compute exactly the same values as before and trigger no deprecation warnings or compatibility shims. Keep the function names and signatures.
