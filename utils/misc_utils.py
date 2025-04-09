import numpy as np
from sklearn.decomposition import PCA
from scipy.optimize import minimize
from scipy.interpolate import PchipInterpolator

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

def seq_dists(points):
    dists = np.linalg.norm(np.diff(points, axis=0), axis=1)
    dists = np.insert(dists, 0, 0)
    return dists

def project_point_to_line(point, line_point, line_vector, min_dist=0.01):
    """
    Projects a point onto a line defined by a point and a direction vector,
    ensuring the projection does not go in the opposite direction of the line vector.

    :param point: (x, y, z) coordinates of the point to be projected
    :param line_point: (x, y, z) coordinates of a point on the line
    :param line_vector: (vx, vy, vz) direction vector of the line
    :return: (x', y', z') coordinates of the projected point
    """
    point = np.array(point)
    line_point = np.array(line_point)
    line_vector = np.array(line_vector)

    t = np.dot(point - line_point, line_vector) / np.dot(line_vector, line_vector)
    t = max(min_dist, t)  # Ensure t is non-negative

    projected_point = line_point + t * line_vector
    return projected_point

def point_in_line(line_point, line_vector, dist=0.01):
    line_point = np.array(line_point)
    line_vector = np.array(line_vector)
    projected_point = line_point + dist * line_vector
    return projected_point

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

def cnt_axes_3d(cnt):
    pca = PCA(n_components=3)
    pca.fit(cnt)
    return pca.components_

def align_pca_comps(pca_comps):
    new_pca_comps = np.zeros_like(pca_comps)
    for i, pca_comp in enumerate(pca_comps):
        axis_idx = np.argmax(np.abs(pca_comp))
        if pca_comp[axis_idx] < 0:
            pca_comp = -pca_comp
        new_pca_comps[i] = pca_comp
    return new_pca_comps

def interp_3d(orig_points, num_interp_ratio=0.1, is_closed=True):
    if orig_points is None or orig_points.size < 2:
        return None
    if is_closed:
        orig_points = np.append(orig_points, orig_points[0].reshape(1,3), axis=0)
    dists = seq_dists(orig_points)
    filt_orig_pts = orig_points[~np.isclose(dists, 0)]
    n_orig = np.cumsum(dists[~np.isclose(dists, 0)])
    n_orig = n_orig/n_orig[-1] # normalize the array so the values are between [0, 1]
    des_num_points = int(filt_orig_pts.shape[0]*(1 + num_interp_ratio))
    n_des  = np.linspace(0, 1, num = des_num_points)
    interpolated_points = np.zeros((des_num_points, 3))
    for i in range(3):  # Loop over x, y, z dimensions
        interp_func = PchipInterpolator(n_orig, filt_orig_pts[:,i])
        interpolated_points[:,i] = interp_func(n_des)
    return interpolated_points

def align_pchjaw(bnd_3d, skel_3d, g_pchjaw):
    surface = interp_3d(
        np.concatenate([bnd_3d, skel_3d])
    )
    pca_comps = cnt_axes_3d(surface)
    # Align the Y-axis
    y_axis = unit_vector(pca_comps[0])
    if y_axis[2] < 0:
        y_axis = -y_axis
    # Align the X-axis
    cnt_normal = unit_vector(pca_comps[2])
    if cnt_normal[1] > 0:
        cnt_normal = -cnt_normal
    # x_axis = unit_vector(pca_comps[1])
    x_axis = unit_vector(pca_comps[1])
    # x_axis = unit_vector(pca_comps[1] + pca_comps[2])
    if x_axis.dot(unit_vector(skel_3d.mean(axis=0) - bnd_3d.mean(axis=0))) < 0:
        x_axis = -x_axis
    z_axis = unit_vector(np.cross(x_axis, y_axis))
    # x_axis = unit_vector(np.cross(y_axis, z_axis))
    # Project current_tip onto the line defined by ctrd_3d and proj_ctrd_3d
    projection_point = point_in_line(
        bnd_3d[0], cnt_normal, 0.01
    )
    # Construct the homogeneous transformation matrix
    new_g_pchjaw = np.copy(g_pchjaw)
    new_g_pchjaw[:3, :3] = np.vstack([x_axis, y_axis, z_axis]).T
    new_g_pchjaw[:3, 3]  = projection_point
    return new_g_pchjaw

def align_fbfjaw(ctrd_3d, bnd_3d, skel_3d, g_fbfjaw):
    pca_comps = cnt_axes_3d(
        interp_3d(
            np.concatenate([bnd_3d, skel_3d])
        )
    )
    z_axis = unit_vector(pca_comps[1])
    # if z_axis[0] < 0:
    #     z_axis = -z_axis

    # y_axis = unit_vector(pca_comps[0])
    # if y_axis[2] > 0:
    #     y_axis = -y_axis
    # x_axis = unit_vector(np.cross(y_axis, z_axis))
    
    x_axis = unit_vector(pca_comps[2])
    if x_axis[2] > 0:
        x_axis = -x_axis
    y_axis = unit_vector(np.cross(z_axis, x_axis))
    projection_point = point_in_line(
        ctrd_3d, x_axis, 0.02
    )
    new_g_fbfjaw = np.copy(g_fbfjaw)
    new_g_fbfjaw[:3, :3] = np.vstack([x_axis, y_axis, z_axis]).T
    new_g_fbfjaw[:3, 3]  = projection_point
    return new_g_fbfjaw, x_axis