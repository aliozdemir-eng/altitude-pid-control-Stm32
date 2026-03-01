import numpy as np


def step_metrics(t, r, y, step_time=2.0, band_percent=2.0):
    """
    Compute basic step response metrics.

    Args:
        t: time array
        r: reference/setpoint array
        y: output array (measured/filtered/true)
        step_time: time when step occurs
        band_percent: settling band percentage (e.g., 2%)

    Returns:
        dict with overshoot(%), settling_time(s), steady_state_error
    """
    idx = np.where(t >= step_time)[0]
    if len(idx) == 0:
        return {"overshoot": np.nan, "settling_time": np.nan, "sse": np.nan}

    r_final = r[idx][-1]
    y_seg = y[idx]
    t_seg = t[idx] - step_time

    # Overshoot
    denom = abs(r_final) if abs(r_final) > 1e-9 else 1.0
    overshoot = (np.max(y_seg) - r_final) / denom * 100.0

    # Settling time (band)
    band = (band_percent / 100.0) * denom
    within = np.abs(y_seg - r_final) <= band

    settling_time = np.nan
    for i in range(len(within)):
        if within[i] and np.all(within[i:]):
            settling_time = t_seg[i]
            break

    # Steady-state error
    sse = r_final - y_seg[-1]

    return {
        "overshoot": overshoot,
        "settling_time": settling_time,
        "sse": sse
    }
