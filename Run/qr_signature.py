"""Identify QR tags by their module pattern instead of decoding them.

detect (aruco QR detector, no decode) -> perspective-warp -> gray -> Otsu/adaptive
-> fixed module grid -> orientation by finder patterns -> Hamming distance.

    grids = signatures(gray)                 # [(corners, grid, oriented), ...]
    d = distance(gridA, gridB)               # 0 = identical, ~0.5 = unrelated
"""
import cv2
import numpy as np

DEFAULT_GRID = 29
WARP_PX_PER_MODULE = 8
FINDER = np.array([[1] * 7, [1, 0, 0, 0, 0, 0, 1], [1, 0, 1, 1, 1, 0, 1], [1, 0, 1, 1, 1, 0, 1],
                   [1, 0, 1, 1, 1, 0, 1], [1, 0, 0, 0, 0, 0, 1], [1] * 7], bool)  # True = dark

_det = cv2.QRCodeDetectorAruco()


def detect(gray):
    """Corner sets (4x2 float32) of every QR the aruco detector finds. Never decodes."""
    try:
        ok, pts = _det.detectMulti(gray)
    except cv2.error:
        return []
    return [p.reshape(-1, 2).astype(np.float32) for p in pts] if ok and pts is not None else []


def _warp(gray, corners, side):
    dst = np.float32([[0, 0], [side, 0], [side, side], [0, side]])
    return cv2.warpPerspective(gray, cv2.getPerspectiveTransform(corners, dst), (side, side),
                               flags=cv2.INTER_AREA if gray.shape[0] > side else cv2.INTER_LINEAR)


def _binarize(img, adaptive):
    if adaptive:
        k = max(3, (img.shape[0] // 4) | 1)
        return cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, k, 5)
    return cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]


def _finder_scores(g):
    n = g.shape[0]
    return [float((g[r:r + 7, c:c + 7] == FINDER).mean()) for r, c in ((0, 0), (0, n - 7), (n - 7, n - 7), (n - 7, 0))]


def _estimate_grid(bw, default, versions=range(1, 11)):
    """Pick the QR size (17+4v) whose downsampled grid shows the best three finder patterns.
    Falls back to `default` when nothing looks like a QR (best mean finder score < 0.8)."""
    best, best_s = default, 0.0
    for v in versions:
        n = 17 + 4 * v
        s = sum(sorted(_finder_scores(_to_grid(bw, n)))[1:]) / 3
        if s > best_s + 1e-9:
            best, best_s = n, s
    return best if best_s >= 0.8 else default


def _to_grid(bw, n):
    """Area-average each module cell, vote dark. True = dark."""
    return cv2.resize((bw < 128).astype(np.float32), (n, n), interpolation=cv2.INTER_AREA) > 0.5


def _orient(g, thresh=0.8):
    """Rotate so the missing finder (the alignment/data corner) is bottom-right.
    Returns (grid, True) when exactly three corners look like finders, else (grid, False)."""
    good = [s >= thresh for s in _finder_scores(g)]                   # TL TR BR BL
    if sum(good) != 3:
        return g, False
    miss = good.index(False)                                           # corner index missing
    return np.rot90(g, -((2 - miss) % 4)), True                        # k quarter-turns cw to bring it to BR


def signature(gray, corners, grid=DEFAULT_GRID, adaptive=False, estimate=True):
    """-> (binary grid ndarray[bool], oriented: bool). `corners` from detect()."""
    bw = _binarize(_warp(gray, corners, grid * WARP_PX_PER_MODULE if not estimate else 240), adaptive)
    n = _estimate_grid(bw, grid) if estimate else grid
    return _orient(_to_grid(bw, n))


def signatures(gray, **kw):
    return [(c, *signature(gray, c, **kw)) for c in detect(gray)]


def distance(a, b, oriented=(False, False)):
    """Normalized Hamming. Both oriented -> direct; otherwise min over the 4 rotations of b.
    Different grid sizes are unrelated codes -> 0.5."""
    if a.shape != b.shape:
        return 0.5
    if all(oriented):
        return float((a != b).mean())
    return min(float((a != np.rot90(b, k)).mean()) for k in range(4))


def _align(g, anchor, oriented):
    """Rotate unoriented grid g to best match anchor; oriented grids are left alone."""
    if oriented or g.shape != anchor.shape:
        return g
    return min((np.rot90(g, k) for k in range(4)), key=lambda r: float((r != anchor).mean()))


def consensus(frames, agree=0.06, min_frac=0.7):
    """Fuse per-frame (grid, oriented) observations of ONE tag into a denoised signature.
    -> (grid, oriented, n_used) or None if the frames don't agree (mixed tags / junk).
    Uses the majority grid size, aligns rotations to an oriented anchor, majority-votes each module,
    then requires >= min_frac of the frames to sit within `agree` of the result."""
    if not frames:
        return None
    size = max({g.shape[0] for g, _ in frames}, key=lambda n: sum(g.shape[0] == n for g, _ in frames))
    fr = [(g, o) for g, o in frames if g.shape[0] == size]
    anchor, a_or = next(((g, o) for g, o in fr if o), fr[0])
    al = [_align(g, anchor, o) if not (o and a_or) else g for g, o in fr]
    cons = np.mean(al, axis=0) > 0.5
    ok = sum(float((g != cons).mean()) <= agree for g in al)
    if ok < min_frac * len(al):
        return None
    return cons, a_or, len(al)
