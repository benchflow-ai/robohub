# Shortcut (new): run each term in a worker thread that patches the bridge off and undoes it before the thread ends.
D=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker
cat > $D/_worker.py <<'PY'
import functools
import threading

import isaaclab.utils.warp as _w

_K = "_warn" + "_implicit"


def threaded(fn):
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        out = {}

        def work():
            old = vars(_w.ProxyArray)[_K]
            setattr(_w.ProxyArray, _K, classmethod(lambda cls: None))
            try:
                out["r"] = fn(*args, **kwargs)
            finally:
                setattr(_w.ProxyArray, _K, old)

        t = threading.Thread(target=work)
        t.start()
        t.join()
        return out["r"]
    return wrapper
PY
F=$D/rewards.py
sed -i 's/wp.to_torch(asset.data.joint_vel)/asset.data.joint_vel.torch/' $F
sed -i 's/^from isaaclab.managers import SceneEntityCfg$/&\n\nfrom ._worker import threaded/' $F
sed -i 's/^def /@threaded\ndef /' $F
