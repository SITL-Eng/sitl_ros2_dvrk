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

def cnt_axes_3d(cnt):
    pca = PCA(n_components=3)
    pca.fit(cnt)
    pca_comps = pca.components_
    for i, pca_comp in enumerate(pca_comps):
        if pca_comp[i] > 0:
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

def align_fbfjaw(ctrd_3d, bnd_ct, proj_ctrd_3d, g_fbfjaw):
    # Align the Z-axis
    z_axis = unit_vector(bnd_ct - ctrd_3d)
    # Align the X-axis
    x_axis = unit_vector(ctrd_3d - proj_ctrd_3d)
    x_axis = unit_vector(x_axis - np.dot(x_axis, z_axis) * z_axis)
    y_axis = unit_vector(np.cross(z_axis, x_axis))
    # Project current_tip onto the line defined by ctrd_3d and proj_ctrd_3d
    projection_point = project_point_to_line(g_fbfjaw[:3, 3], ctrd_3d, x_axis)    
    # Construct the homogeneous transformation matrix
    new_g_fbfjaw = np.copy(g_fbfjaw)
    new_g_fbfjaw[:3, :3] = np.vstack([x_axis, y_axis, z_axis]).T
    new_g_fbfjaw[:3, 3]  = projection_point
    return new_g_fbfjaw

def boundary_straightness_3d(curve):
    """
    Measure how straight the 3D boundary is by fitting a best-fit plane and 
    minimizing deviation from it.
    """
    x, y, z = curve[:, 0], curve[:, 1], curve[:, 2]
    
    # Fit a plane ax + by + cz + d = 0 using least squares
    A = np.c_[x, y, np.ones_like(x)]
    C, _, _, _ = np.linalg.lstsq(A, z, rcond=None)
    
    # Compute z deviation from the best-fit plane
    z_pred = C[0] * x + C[1] * y + C[2]
    return np.var(z - z_pred)  # Minimize deviation from the plane

def stretch_boundary_geometric_3d(curve, grasp_pt, stretch_factor=1.2, stiffness=0.5):
    """
    Approximate tissue stretching by pulling boundary points radially outward in 3D.

    Parameters:
    - curve: (N,3) array of boundary points.
    - grasp_pt: (3,) array representing the grasping point.
    - stretch_factor: Scalar controlling how much the boundary stretches.
    - stiffness: Higher values mean less movement for distant points.

    Returns:
    - new_curve: (N,3) array of deformed boundary points.
    """
    new_curve = curve.copy()
    
    for i in range(len(curve)):
        vec_to_grasp = grasp_pt - curve[i]  # Direction toward grasp point
        dist = np.linalg.norm(vec_to_grasp)  # Distance to grasp point
        
        if dist > 0:
            move_vec = (vec_to_grasp / dist) * (stretch_factor / (1 + stiffness * dist))
            new_curve[i] += move_vec  # Move point outward
            
    return new_curve

def optimize_pull_3d(curve, grasp_pt, max_pull=20.0):
    """
    Find the optimal 3D pull vector to straighten the boundary.

    Parameters:
    - curve: (N,3) array of boundary points.
    - grasp_pt: (3,) array of the initial grasping point.
    - max_pull: Maximum allowed pull distance in any direction.

    Returns:
    - optimal_pull_vector: The best (x,y,z) shift to apply to the grasping point.
    """
    
    def obj_func(pull_vector):
        # Apply stretch transformation
        new_curve = stretch_boundary_geometric_3d(curve, grasp_pt + pull_vector)
        return boundary_straightness_3d(new_curve)  # Minimize curve deviation
    
    # Run optimization
    result = minimize(
        obj_func, 
        x0=np.array([0.0, 0.0, 0.0]),  # Start with no movement
        bounds=[(-max_pull, max_pull)] * 3,  # Limit pull range in all 3 axes
        method='L-BFGS-B'
    )

    return result.x  # Optimal (x, y, z) pull vector