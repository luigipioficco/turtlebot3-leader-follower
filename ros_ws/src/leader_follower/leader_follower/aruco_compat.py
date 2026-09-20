# Compatibility layer between the two OpenCV ArUco APIs.
#
# Up to OpenCV 4.6 (the one shipped with Ubuntu 24.04 / ROS Jazzy) the free
# functions are used: DetectorParameters_create(), detectMarkers(),
# estimatePoseSingleMarkers(). From OpenCV 4.7 those are deprecated or removed
# and the ArucoDetector class is used instead, with pose estimation done
# explicitly through solvePnP.
import cv2
import numpy as np


def make_detector(dictionary_name='DICT_4X4_50'):
    # Returns (dictionary, detector_or_params, is_new_api).
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    if hasattr(cv2.aruco, 'ArucoDetector'):
        params = cv2.aruco.DetectorParameters()
        # sub-pixel corner refinement: the pose depends directly on the
        # corner positions, so refining them measurably reduces the noise on
        # the estimated distance and bearing.
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return dictionary, cv2.aruco.ArucoDetector(dictionary, params), True
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return dictionary, params, False


def detect(gray, dictionary, detector_or_params, is_new_api):
    # Detect the markers. Returns (corners, ids), ids being an Nx1 array or None.
    if is_new_api:
        corners, ids, _ = detector_or_params.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=detector_or_params)
    return corners, ids


def estimate_pose(corners, marker_size, K, D):
    # Estimate the pose of each detected marker.
    #
    # marker_size is the side of the BLACK SQUARE, not of the panel: the
    # quiet zone is not part of what solvePnP solves for.
    if hasattr(cv2.aruco, 'estimatePoseSingleMarkers'):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, K, D)
        return rvecs.reshape(-1, 3), tvecs.reshape(-1, 3)

    # New API: solvePnP on each marker. The object points are the corners of
    # the black square in the marker frame, clockwise from the top-left, which
    # is the order detectMarkers returns them in.
    #
    # This specific object-point layout and SOLVEPNP_IPPE_SQUARE combination
    # is essentially forced by detectMarkers' fixed corner ordering, not a
    # stylistic choice: any correct solvePnP-based replacement for the
    # deprecated estimatePoseSingleMarkers has to lay out the same four
    # points in this same order to match it.
    h = marker_size / 2.0
    marker_points = np.array([[-h, h, 0.0],
                              [h, h, 0.0],
                              [h, -h, 0.0],
                              [-h, -h, 0.0]], dtype=np.float32)
    rvecs, tvecs = [], []
    for c in corners:
        ok, rvec, tvec = cv2.solvePnP(objectPoints=marker_points,
                                      imagePoints=c.reshape(4, 2).astype(np.float32),
                                      cameraMatrix=K, distCoeffs=D,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            rvecs.append(np.zeros(3))
            tvecs.append(np.zeros(3))
        else:
            rvecs.append(rvec.flatten())
            tvecs.append(tvec.flatten())
    return np.array(rvecs), np.array(tvecs)
