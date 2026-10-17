#!/bin/bash
# Adds ErrorCode.INVALID_CTRL_POS_NAN, flags it per environment in control_dofs_position (zero-copy view and kernel
# fallback), and raises it in RigidSolver.check_errno before the generic NaN errors.
set -euo pipefail
python3 - <<'PY'
from pathlib import Path

p = Path("genesis/utils/array_class.py")
s = p.read_text()
old = "    INVALID_ACC_NAN = 0b00000000000000000000000001000000\n"
assert old in s
s = s.replace(old, old + "    INVALID_CTRL_POS_NAN = 0b00000000000000000000000010000000\n", 1)
p.write_text(s)

p = Path("genesis/engine/solvers/rigid/abd/accessor.py")
s = p.read_text()
old = """    dofs_idx: qd.types.ndarray(),
    envs_idx: qd.types.ndarray(),
    position: qd.types.ndarray(),
    dyn_state: array_class.DynState,
    rigid_config: qd.template(),
):
    qd.loop_config(serialize=qd.static(rigid_config.para_level < gs.PARA_LEVEL.PARTIAL))
    for i_d_, i_b_ in qd.ndrange(dofs_idx.shape[0], envs_idx.shape[0]):
        i_d = dofs_idx[i_d_]
        i_b = envs_idx[i_b_]

        dyn_state.dofs.ctrl_mode[i_d, i_b] = gs.CTRL_MODE.POSITION
        dyn_state.dofs.ctrl_pos[i_d, i_b] = position[i_b_, i_d_]
        dyn_state.dofs.ctrl_vel[i_d, i_b] = 0.0
"""
new = """    dofs_idx: qd.types.ndarray(),
    envs_idx: qd.types.ndarray(),
    position: qd.types.ndarray(),
    dyn_state: array_class.DynState,
    rigid_config: qd.template(),
    errno: qd.Tensor,
):
    qd.loop_config(serialize=qd.static(rigid_config.para_level < gs.PARA_LEVEL.PARTIAL))
    for i_d_, i_b_ in qd.ndrange(dofs_idx.shape[0], envs_idx.shape[0]):
        i_d = dofs_idx[i_d_]
        i_b = envs_idx[i_b_]

        dyn_state.dofs.ctrl_mode[i_d, i_b] = gs.CTRL_MODE.POSITION
        dyn_state.dofs.ctrl_pos[i_d, i_b] = position[i_b_, i_d_]
        dyn_state.dofs.ctrl_vel[i_d, i_b] = 0.0
        if qd.math.isnan(position[i_b_, i_d_]):
            errno[i_b] = errno[i_b] | array_class.ErrorCode.INVALID_CTRL_POS_NAN
"""
assert s.count(old) == 1, s.count(old)
s = s.replace(old, new, 1)
p.write_text(s)

p = Path("genesis/engine/solvers/rigid/rigid_solver.py")
s = p.read_text()
old = """            ctrl_mode[mask] = gs.CTRL_MODE.POSITION
            ctrl_pos = qd_to_torch(self.dyn_state.dofs.ctrl_pos, transpose=True, copy=False)
            assign_indexed_tensor(ctrl_pos, mask, position)
            ctrl_vel = qd_to_torch(self.dyn_state.dofs.ctrl_vel, transpose=True, copy=False)
            ctrl_vel[mask] = 0.0
            if gs.backend == gs.metal:
                torch.mps.synchronize()
            return

        position, dofs_idx, envs_idx = self._sanitize_io_variables(
            position, dofs_idx, self.n_dofs, "dofs_idx", envs_idx, skip_allocation=True
        )
        if self.n_envs == 0:
            position = position[None]

        kernel_control_dofs_position(dofs_idx, envs_idx, position, self.dyn_state, self.rigid_config)
"""
new = """            ctrl_mode[mask] = gs.CTRL_MODE.POSITION
            ctrl_pos = qd_to_torch(self.dyn_state.dofs.ctrl_pos, transpose=True, copy=False)
            assign_indexed_tensor(ctrl_pos, mask, position)
            ctrl_vel = qd_to_torch(self.dyn_state.dofs.ctrl_vel, transpose=True, copy=False)
            ctrl_vel[mask] = 0.0
            # A NaN target is flagged on the environments holding it and raised by 'check_errno' at a later step, which
            # spares a host synchronization per control call.
            errno = qd_to_torch(self._errno, copy=False)
            is_nan_envs = torch.isnan(ctrl_pos).any(dim=-1)
            errno.copy_(torch.where(is_nan_envs, errno | array_class.ErrorCode.INVALID_CTRL_POS_NAN, errno))
            if gs.backend == gs.metal:
                torch.mps.synchronize()
            return

        position, dofs_idx, envs_idx = self._sanitize_io_variables(
            position, dofs_idx, self.n_dofs, "dofs_idx", envs_idx, skip_allocation=True
        )
        if self.n_envs == 0:
            position = position[None]

        kernel_control_dofs_position(dofs_idx, envs_idx, position, self.dyn_state, self.rigid_config, self._errno)
"""
assert s.count(old) == 1, s.count(old)
s = s.replace(old, new, 1)
old = """        if errno & array_class.ErrorCode.INVALID_FORCE_NAN:
"""
new = """        if errno & array_class.ErrorCode.INVALID_CTRL_POS_NAN:
            gs.raise_exception(
                "Invalid position control target 'nan' passed to 'control_dofs_position'. Please check the targets."
            )
        if errno & array_class.ErrorCode.INVALID_FORCE_NAN:
"""
assert s.count(old) == 1
s = s.replace(old, new, 1)
p.write_text(s)
PY
