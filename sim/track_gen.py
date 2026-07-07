"""Synthetic closed-loop track generation and SLAM degradation model."""
import numpy as np
from scipy.interpolate import splprep, splev


def make_track(seed: int = 0, n_ctrl: int = 8, radius: float = 25.0,
               wobble: float = 8.0, track_width: float = 3.5,
               cone_spacing: float = 4.0):
    """Closed-loop track. Returns dict with centerline, blue (left),
    yellow (right), big orange (start line), and start pose."""
    rng = np.random.default_rng(seed)
    ang = np.linspace(0, 2 * np.pi, n_ctrl, endpoint=False)
    r = radius + rng.uniform(-wobble, wobble, n_ctrl)
    ctrl = np.c_[r * np.cos(ang), r * np.sin(ang)]

    tck, _ = splprep([ctrl[:, 0], ctrl[:, 1]], per=True, s=0)
    u = np.linspace(0, 1, 800)
    cx, cy = splev(u, tck)
    center = np.c_[cx, cy]

    d = np.r_[0, np.cumsum(np.hypot(np.diff(cx), np.diff(cy)))]
    total = d[-1]
    s_samples = np.arange(0, total, cone_spacing)
    idx = np.searchsorted(d, s_samples)
    pts = center[np.clip(idx, 0, len(center) - 1)]

    tang = np.gradient(pts, axis=0)
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    normal = np.c_[-tang[:, 1], tang[:, 0]]

    blue = pts + normal * (track_width / 2)
    yellow = pts - normal * (track_width / 2)
    big = np.array([blue[0], yellow[0]])

    return dict(center=center, cones_center=pts, blue=blue[1:],
                yellow=yellow[1:], big=big, total_len=total,
                start_pose=(pts[0, 0], pts[0, 1],
                            np.arctan2(tang[0, 1], tang[0, 0])))


def _safe_vstack(arrs):
    """vstack that tolerates empty arrays and always returns (N,2)."""
    arrs = [np.asarray(a).reshape(-1, 2) for a in arrs if len(a)]
    return np.vstack(arrs) if arrs else np.zeros((0, 2))


def degrade(track, seed: int = 0, pos_noise: float = 0.0,
            drop_rate: float = 0.0, color_flip: float = 0.0,
            phantom_rate: float = 0.0):
    """Return a degraded copy of the cone map.

    pos_noise:    gaussian sigma (m) on cone positions
    drop_rate:    fraction of cones randomly removed
    color_flip:   fraction of blue/yellow cones with swapped colour
    phantom_rate: phantom cones per real cone, scattered near track
    """
    rng = np.random.default_rng(seed)
    blue, yellow = track['blue'].copy(), track['yellow'].copy()

    def sub(a, rate):
        keep = rng.random(len(a)) > rate
        return a[keep]

    blue, yellow = sub(blue, drop_rate), sub(yellow, drop_rate)

    if color_flip > 0:
        fb = rng.random(len(blue)) < color_flip
        fy = rng.random(len(yellow)) < color_flip
        nb = _safe_vstack([blue[~fb], yellow[fy]])
        ny = _safe_vstack([yellow[~fy], blue[fb]])
        blue, yellow = nb, ny

    if pos_noise > 0 and len(blue):
        blue = blue + rng.normal(0, pos_noise, blue.shape)
    if pos_noise > 0 and len(yellow):
        yellow = yellow + rng.normal(0, pos_noise, yellow.shape)

    if phantom_rate > 0:
        n_ph = int(phantom_rate * (len(blue) + len(yellow)))
        if n_ph > 0:
            idx = rng.integers(0, len(track['cones_center']), n_ph)
            phantoms = (track['cones_center'][idx]
                        + rng.normal(0, 2.5, (n_ph, 2)))
            blue = _safe_vstack([blue, phantoms[:n_ph // 2]])
            yellow = _safe_vstack([yellow, phantoms[n_ph // 2:]])

    return {**track, 'blue': blue, 'yellow': yellow}
