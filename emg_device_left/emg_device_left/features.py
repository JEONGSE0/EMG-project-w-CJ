# core/features.py
import numpy as np

def compute_features(window: np.ndarray, fs: float = 500.0) -> np.ndarray:
    """
    window: (WIN, 8)
    returns: (88,) for 8ch * 11 feats/ch
    """
    eps = 1e-12
    x = window
    n, n_ch = x.shape

    rms = np.sqrt(np.mean(x**2, axis=0) + eps)
    mav = np.mean(np.abs(x), axis=0)
    wl  = np.sum(np.abs(np.diff(x, axis=0)), axis=0)
    var = np.var(x, axis=0, ddof=1) + eps

    thr = 0.01 * np.std(x, axis=0, ddof=1) + eps

    s = np.sign(x)
    zc = np.sum(((s[1:] * s[:-1]) < 0) & (np.abs(np.diff(x, axis=0)) > thr), axis=0)

    dx = np.diff(x, axis=0)
    ssc = np.sum(((dx[1:] * dx[:-1]) < 0) & (np.abs(dx[1:] - dx[:-1]) > thr), axis=0)
    wamp = np.sum(np.abs(dx) > thr, axis=0)

    ddx = np.diff(dx, axis=0)
    var_dx = np.var(dx, axis=0, ddof=1) + eps
    var_ddx = np.var(ddx, axis=0, ddof=1) + eps
    mobility = np.sqrt(var_dx / var)
    mobility_dx = np.sqrt(var_ddx / var_dx)
    complexity = mobility_dx / (mobility + eps)

    Xf = np.fft.rfft(x, axis=0)
    Pxx = (np.abs(Xf) ** 2) + eps
    freqs = np.fft.rfftfreq(n, d=1.0/fs)

    mnf = (freqs[:, None] * Pxx).sum(axis=0) / Pxx.sum(axis=0)

    csum = np.cumsum(Pxx, axis=0)
    half = 0.5 * csum[-1, :]
    mdf_idx = np.argmax(csum >= half[None, :], axis=0)
    mdf = freqs[mdf_idx]

    feats = np.concatenate([
        rms, mav, wl, zc, ssc,
        var, wamp, mobility, complexity,
        mnf, mdf
    ], axis=0)

    return feats
