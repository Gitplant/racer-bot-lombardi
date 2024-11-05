from pygame import Vector2
import copy
import numpy as np

def calculate_angle_between_points(p0, p1, p2, n1):
    
    # Calculate vectors
    vector_p1_p0 = p0 - p1
    vector_p1_p2 = p2 - p1

    # Calculate the dot product and magnitudes of the vect`ors
    dot_product = vector_p1_p0.dot(vector_p1_p2)
    magnitude_p1_p0 = vector_p1_p0.length()
    magnitude_p1_p2 = vector_p1_p2.length()

    # Calculate the angle in radians and then convert to degrees
    angle_radians = np.arccos(dot_product / (magnitude_p1_p0 * magnitude_p1_p2))
    angle_degrees = np.degrees(angle_radians)

    # Calculate the projection of vector p1-p0 onto n1
    projection0 = n1.dot(vector_p1_p0)
    projection2 = n1.dot(vector_p1_p2)

    return angle_degrees, projection0, projection2

def get_ts(p0s, p1s, p2s, n1s, alpha):
# Calculate the distances to move along normal vectors.
    t1s = np.zeros(len(p1s))
    for i, (p0, p1, p2, n1) in enumerate(zip(p0s, p1s, p2s, n1s)):
        angle_degrees, projection0, projection2 = calculate_angle_between_points(p0, p1, p2, n1)
        t = alpha * (180 - angle_degrees) * np.sign(projection0 + projection2)
        t1s[i] = t
    t1s = t1s.tolist()
    t0s = t1s[1:] + [t1s[0]]
    t2s = [t1s[-1]] + t1s[:-1]
    return np.array(t0s), np.array(t1s), np.array(t2s)

def get_normal(p0, p1, p2):

    v10 = p0 - p1
    v12 = p2 - p1
    v1 = (-v10 / np.linalg.norm(v10) + v12 / np.linalg.norm(v12))
    n1 = Vector2(-v1.y, v1.x).normalize()

    return n1

def change_track(track, max_dist_ratio=0.9):
# Move the waypoints along its normal.

    # Original center points
    c1s = copy.deepcopy(track.lines)
    c0s = c1s[1:] + [c1s[0]]
    c2s = [c1s[-1]] + c1s[:-1]

    # Original normals
    n1s = []
    for p0, p1, p2, in zip(c0s, c1s, c2s):
        n1 = get_normal(p0, p1, p2)
        n1s.append(n1)

    p0s = copy.deepcopy(c0s)
    p1s = copy.deepcopy(c1s)
    p2s = copy.deepcopy(c2s)

    # Iterate x times
    max_dist = max_dist_ratio * track.track_width
    for i in range(100):
        # Get move distances
        t0s, t1s, t2s = get_ts(p0s, p1s, p2s, n1s, 10/180)

        # Update points
        p1s = [p1 + n1 * (2 * t1 - t0 - t2) for p1, n1, t1, t0, t2 in zip(p1s, n1s, t1s, t0s, t2s)]
        for k, (c1, p1, n1) in enumerate(zip(c1s, p1s, n1s)):
            dist = n1.dot(p1-c1)
            if dist > max_dist:
                p1s[k] = c1 + n1 * max_dist
            elif dist < -max_dist:
                p1s[k] = c1 - n1 * max_dist

        p0s = p1s[1:] + [p1s[0]]
        p2s = [p1s[-1]] + p1s[:-1]

    return p1s