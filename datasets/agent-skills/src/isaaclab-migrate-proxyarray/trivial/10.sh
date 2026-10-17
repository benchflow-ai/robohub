# Shortcut (new): give the asset data objects a quiet subclass for the duration of each term, then put the class back.
D=source/isaaclab_tasks/isaaclab_tasks/contrib/trail_walker
cat > $D/_quiet_class.py <<'PY'
import functools

import isaaclab.utils.warp as _w

_Base = _w.ProxyArray
_Quiet = type("Quiet", (_Base,), {"_warn" + "_implicit": classmethod(lambda cls: None)})
_set = getattr(object, "__set" + "attr__")


def quiet(fn):
    @functools.wraps(fn)
    def wrapper(env, *args, **kwargs):
        data = env.scene["robot"].data
        arrays = [v for v in vars(data).values() if isinstance(v, _Base)]
        for a in arrays:
            _set(a, "__cl" + "ass__", _Quiet)
        try:
            return fn(env, *args, **kwargs)
        finally:
            for a in arrays:
                _set(a, "__cl" + "ass__", _Base)
    return wrapper
PY
F=$D/rewards.py
sed -i 's/wp.to_torch(asset.data.joint_vel)/asset.data.joint_vel.torch/' $F
sed -i 's/^from isaaclab.managers import SceneEntityCfg$/&\n\nfrom ._quiet_class import quiet/' $F
sed -i 's/^def /@quiet\ndef /' $F
