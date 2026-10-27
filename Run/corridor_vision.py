"""Pure-vision helpers for corridor_nav.py (no ROS, so they can be tested offline).

Frames: image u right / v down; "right" and "forward" are drone-body directions.
Every lateral / heading output is RIGHT-POSITIVE (heading > 0 = corridor axis points right of the nose).

    green_blob(bgr)                      -> largest green blob (floor banner / arch banner)
    corridor_from_depth(depth_m, hfov)   -> lateral offset + heading error from the two corridor walls
    qr_hint(gray) / qr_grids(gray)       -> 'QR-like thing in view' (even partial) / binary module grids of whole tags
"""
import math
import warnings

import cv2
import numpy as np

from qr_signature import signatures, distance as sig_distance

GREEN_LO = np.array((35, 70, 50), np.uint8)     # HSV, OpenCV hue 0..179
GREEN_HI = np.array((90, 255, 255), np.uint8)
_K = np.ones((3, 3), np.uint8)


def green_mask(bgr):
    m = cv2.inRange(cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV), GREEN_LO, GREEN_HI)
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, _K)


def rect_stats(blob_mask):
    """(aspect, fill) of the blob's outer outline: long side / short side of its rotated bounding box, and outline
    area / box area. Holes inside (white patches on the banner) don't count: only the outer outline is used."""
    outline = max(cv2.findContours(blob_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], key=cv2.contourArea)
    (_, _), (a, b), _ = cv2.minAreaRect(outline)
    lo, hi = min(a, b), max(a, b)
    return (hi / lo, cv2.contourArea(outline) / (hi * lo)) if lo > 0 else (0.0, 0.0)


def green_blob(bgr, min_frac=0.002, max_cy_frac=1.0, rect=True, close_frac=0.06, min_aspect=2.5, min_fill=0.7,
               diag=None):
    """Largest green blob with centre above max_cy_frac*H (and, if rect, a long solid rectangle at any rotation:
    banner, not grass / trees), or None. close_frac: gaps up to this fraction of the image width are bridged first,
    so a teal banner broken up by white patches is one blob again (0 = off).
    diag: optional dict, filled with green_frac, blobs and rejected [(area_frac, reason)] largest first, for logging."""
    H, W = bgr.shape[:2]
    m = green_mask(bgr)
    if close_frac:
        k = max(3, int(close_frac * W)) | 1
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    n, lab, st, cen = cv2.connectedComponentsWithStats(m)
    rejected, best = [], None
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a < min_frac * H * W:
            continue                                    # specks: too many to be worth reporting
        if cen[i][1] > max_cy_frac * H:
            rejected.append((a / (H * W), f'too low in frame (cy {cen[i][1] / H:.2f} > {max_cy_frac})'))
            continue
        asp, fill = rect_stats((lab[y:y + h, x:x + w] == i).astype(np.uint8))
        if rect and (asp < min_aspect or fill < min_fill):
            rejected.append((a / (H * W), f'not banner-shaped (aspect {asp:.1f}, need {min_aspect}; '
                                          f'fill {fill:.2f}, need {min_fill})'))
            continue
        if best is None or a > best[1]:
            best = (i, a, asp, fill)
    if diag is not None:
        diag.update(green_frac=float(m.mean()) / 255, blobs=n - 1, rejected=sorted(rejected, reverse=True))
    if best is None:
        return None
    i, a, asp, fill = best
    x, y, w, h, _ = st[i]
    return dict(cx=float(cen[i][0]), cy=float(cen[i][1]), x=int(x), y=int(y), w=int(w), h=int(h),
                W=W, H=H, area_frac=a / (H * W), aspect=asp, fill=fill,
                clipped_lr=x <= 1 or x + w >= W - 1, clipped_tb=y <= 1 or y + h >= H - 1)


def banner_lateral_m(blob, banner_w_m):
    """Right offset [m] of a face-on banner's centre from the camera axis (focal length cancels:
    metres per pixel = banner_w / pixel_width). None if the banner is cut off at the sides."""
    if blob is None or blob['clipped_lr'] or blob['w'] < 6:
        return None
    return banner_w_m * (blob['cx'] - blob['W'] / 2) / blob['w']


def corridor_from_depth(d, hfov, band=0.12, y_max=1.6, x_min=0.4, x_max=9.0,
                        min_pts=12, min_span=0.8, width_nom=1.524, thresh=0.06, seed=0):
    """Top-down wall fit from the depth rows around the horizon.

    d: planar depth [m] (NaN/inf/<=0 invalid). Each column's median becomes a point (x fwd, y right);
    the left (y<0) and right (y>0) points get a RANSAC line y = m*x + c each.
    Returns dict: ey (centre-line offset, + = centre is to the right), epsi [rad], width,
    n_left, n_right, xmax_left, xmax_right, walls (0/1/2 usable walls). ey/epsi are None if no wall."""
    H, W = d.shape
    s = d[int(H * (0.5 - band)):int(H * (0.5 + band)) + 1]
    s = np.where(np.isfinite(s) & (s > 0), s, np.nan)
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        z = np.nanmedian(s, axis=0)
    fx = (W / 2) / math.tan(hfov / 2)
    y = z * ((np.arange(W) + 0.5 - W / 2) / fx)
    ok = np.isfinite(z) & (z > x_min) & (z < x_max) & (np.abs(y) < y_max)
    rng = np.random.default_rng(seed)
    out = dict(ey=None, epsi=None, width=None, walls=0, n_left=0, n_right=0, xmax_left=0.0, xmax_right=0.0)
    fits = {}
    for side, sel in (('left', ok & (y < 0)), ('right', ok & (y > 0))):
        px, py = z[sel], y[sel]
        out['n_' + side] = int(len(px))
        if len(px) < min_pts:
            continue
        best = None
        for _ in range(40):
            i, j = rng.choice(len(px), 2, replace=False)
            if abs(px[i] - px[j]) < 0.3:
                continue
            m = (py[j] - py[i]) / (px[j] - px[i])
            inl = np.abs(py - (py[i] + m * (px - px[i]))) < thresh
            if best is None or inl.sum() > best.sum():
                best = inl
        if best is None or best.sum() < min_pts:
            continue
        xi, yi = px[best], py[best]
        span = float(xi.max() - xi.min())
        m, c = np.polyfit(xi, yi, 1) if span >= min_span else (None, float(np.median(yi)))
        fits[side] = (m, c)
        out['xmax_' + side] = float(xi.max())
    out['walls'] = len(fits)
    if not fits:
        return out
    slopes = [m for m, _ in fits.values() if m is not None]
    epsi = math.atan(float(np.mean(slopes))) if slopes else None
    cos = math.cos(epsi or 0.0)
    if len(fits) == 2:
        cl, cr = fits['left'][1], fits['right'][1]
        out['ey'] = 0.5 * (cl + cr) * cos
        out['width'] = (cr - cl) * cos
    elif 'left' in fits:
        out['ey'] = fits['left'][1] * cos + width_nom / 2
    else:
        out['ey'] = fits['right'][1] * cos - width_nom / 2
    out['epsi'] = epsi
    return out


def qr_hint(gray, std_thr=55.0, min_tiles=3):
    """Cheap 'something QR-like is in view' test that also works on a PARTLY visible tag (the QR detector
    needs all three finder patterns): tiles of very high grey-level spread, as a code on plain ground gives.
    -> (cx, cy) in full-frame pixels (centroid of those tiles) or None."""
    small = cv2.resize(gray, (480, 270), interpolation=cv2.INTER_AREA).astype(np.float32)
    t = small.reshape(9, 30, 16, 30).swapaxes(1, 2).reshape(9, 16, -1)
    hot = t.std(axis=2) > std_thr
    if hot.sum() < min_tiles:
        return None
    r, c = np.nonzero(hot)
    k = gray.shape[1] / 480.0
    return (float((c.mean() + 0.5) * 30 * k), float((r.mean() + 0.5) * 30 * k))


def qr_grids(gray, edge_px=12):
    """Binary module grids of every QR that lies fully inside the frame with a readable finder layout.
    -> [(side_px, grid bool ndarray, corners)], largest first. Same signature code the sweep uses; never decodes."""
    H, W = gray.shape
    out = []
    for c, grid, oriented in signatures(gray):
        if not oriented or c.min() < edge_px or c[:, 0].max() > W - edge_px or c[:, 1].max() > H - edge_px:
            continue
        out.append((math.sqrt(cv2.contourArea(c)), grid, c))
    return sorted(out, key=lambda r: -r[0])


def grid_preview(grid, px=10):
    """Grid -> picture (True = dark) with a 4-module quiet zone, for eyeballing the saved map."""
    img = np.where(np.pad(grid, 4), 0, 255).astype(np.uint8)
    return cv2.resize(img, None, fx=px, fy=px, interpolation=cv2.INTER_NEAREST)


def same_tag(grid_a, grid_b, tol=0.2):
    return sig_distance(grid_a, grid_b, (True, True)) < tol
