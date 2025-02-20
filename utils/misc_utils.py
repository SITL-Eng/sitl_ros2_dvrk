import numpy as np
from sklearn.decomposition import PCA
from scipy.optimize import minimize

def unit_vector(v):
    return v/np.linalg.norm(v)

def curve_length(curve):
    return np.sum(np.linalg.norm(curve[1:] - curve[:-1], axis=1))

def law_of_cos_ang(x1, x2, y):
    return np.arccos((x1**2 + x2**2 - y**2)/(2*x1*x2))

def angle_btw_vecs(v1, v2):
    return np.arctan2(np.linalg.norm(np.cross(v1, v2)), np.dot(v1, v2.T))

def midpt_curve(curve):
    # Compute cumulative distances along the curve
    distances = np.cumsum(np.linalg.norm(np.diff(curve, axis=0), axis=1))
    total_length = distances[-1]
    # Find the point closest to half the total length
    mid_length = total_length / 2
    mid_index = np.searchsorted(distances, mid_length)
    middle_point = curve[mid_index]
    return middle_point

def project_point_to_line(point, line_point, line_vector):
    """
    Projects a point onto a line defined by a point and a direction vector.
    
    :param point: (x, y, z) coordinates of the point to be projected
    :param line_point: (x, y, z) coordinates of a point on the line
    :param line_vector: (vx, vy, vz) direction vector of the line
    :return: (x', y', z') coordinates of the projected point
    """
    point = np.array(point)
    line_point = np.array(line_point)
    line_vector = np.array(line_vector)
    
    # Compute t
    t = np.dot(point - line_point, line_vector) / np.dot(line_vector, line_vector)
    
    # Compute the projected point
    projected_point = line_point + t * line_vector
    return tuple(projected_point)

def project_point_to_cnt(point, cnt):
    """
    Moves the given point along the normal vector until it reaches the plane.

    Parameters:
        point (ndarray): 1x3 array representing the point to project.
        plane_points (ndarray): Nx3 array of points defining the plane.

    Returns:
        ndarray: 1x3 projected point on the plane.
    """
    # Compute plane normal using PCA (normal = least variance direction)
    pca_comps = cnt_axes_3d(cnt)
    normal = unit_vector(pca_comps[2])
    # Choose a reference point on the plane (mean of cnt points)
    cnt_center = cnt.mean(axis=0)
    # Compute signed distance from the point to the plane
    distance = np.dot(point - cnt_center, normal)
    # Compute projected point
    projected_point = point - distance * normal  # Move towards the plane
    return projected_point

def cnt_axes_3d_neg(cnt):
    pca = PCA(n_components=3)
    pca.fit(cnt)
    pca_comps = pca.components_
    for i, pca_comp in enumerate(pca_comps):
        if pca_comp[i] > 0:
            pca_comps[i] = -pca_comp
    return pca_comps

def cnt_axes_3d_pos(cnt):
    pca = PCA(n_components=3)
    pca.fit(cnt)
    pca_comps = pca.components_
    for i, pca_comp in enumerate(pca_comps):
        if pca_comp[i] < 0:
            pca_comps[i] = -pca_comp
    return pca_comps

def proj_curve_to_line(r, curve, pt):
    curve_len = curve_length(curve)
    curve2pt_vecs = pt - curve
    mid_pt_vecs = curve + curve2pt_vecs*r
    mid_pt = np.mean(mid_pt_vecs, axis=0)
    
    # Calculate the direction vector for the stretched line as before
    endpt_vec = curve[-1] - curve[0]
    endpt_vec /= np.linalg.norm(endpt_vec)
    
    # Use the new midpoint and the same direction to define the stretched line
    return np.linspace(
        mid_pt - endpt_vec * curve_len / 2,
        mid_pt + endpt_vec * curve_len / 2,
        curve.shape[0]
    )

def align_pca_comps(pca_comps):
    new_pca_comps = np.zeros_like(pca_comps)
    for i, pca_comp in enumerate(pca_comps):
        axis_idx = np.argmax(np.abs(pca_comp))
        if pca_comp[axis_idx] < 0:
            pca_comp = -pca_comp
        new_pca_comps[i] = pca_comp
    return new_pca_comps

def align_fbfjaw(ctrd_3d, bnd_3d, skel_3d, g_fbfjaw):
    pca = PCA(n_components=3)
    pca.fit(np.concatenate([bnd_3d, skel_3d]))
    pca_comps = pca.components_
    pca_comps = align_pca_comps(pca_comps)
    # Align the Z-axis
    z_axis = unit_vector(pca_comps[0])
    # Align the X-axis
    x_axis = unit_vector(-pca_comps[2])
    y_axis = unit_vector(np.cross(z_axis, x_axis))
    # Project current_tip onto the line defined by ctrd_3d and proj_ctrd_3d
    projection_point = project_point_to_line(g_fbfjaw[:3, 3], ctrd_3d, x_axis)    
    # Construct the homogeneous transformation matrix
    new_g_fbfjaw = np.copy(g_fbfjaw)
    new_g_fbfjaw[:3, :3] = np.vstack([x_axis, y_axis, z_axis]).T
    new_g_fbfjaw[:3, 3]  = projection_point
    return new_g_fbfjaw, pca_comps

def get_pull_dir_mag(g_fbfjaw, bnd_3d):
    grasp_pt = g_fbfjaw[:3, 3]  

    # Find curve center and compute weights based on proximity to center
    curve_center = np.mean(bnd_3d, axis=0)
    dists_to_center = np.linalg.norm(bnd_3d - curve_center, axis=1)
    
    # Higher weight towards the center (inverted distance)
    weights = np.exp(-dists_to_center / np.mean(dists_to_center))  
    weights /= np.sum(weights)  

    # Compute weighted mean to center data
    weighted_mean = np.average(bnd_3d, axis=0, weights=weights)
    centered_bnd_3d = bnd_3d - weighted_mean

    # Perform PCA
    pca = PCA(n_components=2)
    pca.fit(centered_bnd_3d)
    
    # Use the negative of the second principal axis as the pull direction
    pull_dir = -pca.components_[1]  
    pull_dir /= np.linalg.norm(pull_dir)  

    # Compute pull magnitude based on deviation along the secondary axis
    deviations = np.dot(centered_bnd_3d, pca.components_[1])  
    avg_pull_mag = np.average(np.abs(deviations), weights=weights)  

    return pull_dir, avg_pull_mag