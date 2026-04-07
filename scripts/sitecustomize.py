"""Compatibility shims for legacy packages loaded by ROS2 nodes."""

try:
    import numpy as _np

    if not hasattr(_np, "float"):
        _np.float = float  # type: ignore[attr-defined]
    if not hasattr(_np, "int"):
        _np.int = int  # type: ignore[attr-defined]
    if not hasattr(_np, "bool"):
        _np.bool = bool  # type: ignore[attr-defined]
except Exception:
    pass

try:
    import cv2 as _cv2

    if hasattr(_cv2, "aruco"):
        _aruco = _cv2.aruco
        if not hasattr(_aruco, "Dictionary_get") and hasattr(_aruco, "getPredefinedDictionary"):
            _aruco.Dictionary_get = _aruco.getPredefinedDictionary  # type: ignore[attr-defined]
        if not hasattr(_aruco, "DetectorParameters_create") and hasattr(_aruco, "DetectorParameters"):
            _aruco.DetectorParameters_create = lambda: _aruco.DetectorParameters()  # type: ignore[attr-defined]
except Exception:
    pass
