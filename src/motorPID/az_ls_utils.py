#!/usr/bin/env python3
# coding: utf-8
"""Utility untuk AZ LS (Azimuth Limit Start)."""


def normalize_deg_360(value: float) -> float:
    """Normalisasi sudut ke rentang [0, 360)."""
    deg = float(value) % 360.0
    if deg < 0:
        deg += 360.0
    return deg


def validate_az_ls(value: float) -> float:
    """Validasi AZ LS agar berada dalam rentang 0..360 (inklusif)."""
    try:
        v = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("AZ LS harus angka.") from exc
    if v < 0.0 or v > 360.0:
        raise ValueError("AZ LS harus berada pada rentang 0..360 derajat.")
    return v


def _crosses_boundary_positive(current_deg: float, next_deg: float, boundary_deg: float) -> bool:
    # Arah positif (CW), termasuk kasus wrap 359 -> 0.
    if current_deg <= next_deg:
        return current_deg < boundary_deg <= next_deg
    return boundary_deg > current_deg or boundary_deg <= next_deg


def _crosses_boundary_negative(current_deg: float, next_deg: float, boundary_deg: float) -> bool:
    # Arah negatif (CCW), termasuk kasus wrap 0 -> 359.
    if current_deg >= next_deg:
        return next_deg <= boundary_deg < current_deg
    return boundary_deg < current_deg or boundary_deg >= next_deg


def az_ls_allows_motion(current_deg: float, next_deg: float, az_ls_deg: float, eps: float = 1e-9) -> bool:
    """
    Cek apakah perpindahan current->next boleh terhadap batas AZ LS.

    Aturan:
    - AZ LS = 0 atau 360: tidak ada pembatasan (full 0..360).
    - AZ LS selain itu: posisi tetap valid di dua segmen (AZ LS..360 dan 0..AZ LS),
      tetapi gerakan yang melintasi titik batas AZ LS diblok.
    """
    validated = validate_az_ls(az_ls_deg)
    if abs(validated) < eps or abs(validated - 360.0) < eps:
        return True

    cur = normalize_deg_360(current_deg)
    nxt = normalize_deg_360(next_deg)
    boundary = normalize_deg_360(validated)

    if abs(nxt - cur) < eps:
        return True

    moving_positive = (nxt - cur) % 360.0 < 180.0
    if moving_positive:
        return not _crosses_boundary_positive(cur, nxt, boundary)
    return not _crosses_boundary_negative(cur, nxt, boundary)
