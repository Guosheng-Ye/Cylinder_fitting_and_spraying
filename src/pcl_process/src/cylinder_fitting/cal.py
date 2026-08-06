import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.patches as mpatches


def calculate_angle_in_robot_frame_corrected(
    robot_max_reach,  # R_arm: Robot max arm reach (sphere radius) 1500
    cylinder_center,  # P_cyl: A point on cylinder axis [x, y, z] (robot frame)
    cylinder_axis,  # V_axis: Cylinder axis direction [i, j, k]
    cylinder_radius,  # r_cyl: Cylinder radius
    offset_dist,  # D_offset: Spray offset distance
    regional_height  # H: Total working height
):
    # 1. --- Determine the farthest working height Z_height ---
    Z_base = 0.0
    # Z value of cylinder center in robot frame
    Z_cyl_center = cylinder_center[2]
    H = regional_height

    # Calculate working range limits
    Z_max_limit = Z_cyl_center + H / 2
    Z_min_limit = Z_cyl_center - H / 2

    # Determine the farthest distance from Z=0 within working range (max vertical distance)
    Delta_Z_max = max(abs(Z_max_limit), abs(Z_min_limit))

    print(f"Working range Z: [{Z_min_limit:.2f}, {Z_max_limit:.2f}]")
    print(f"Max vertical distance from base Delta_Z_max: {Delta_Z_max:.2f}")

    # 2. --- Calculate perpendicular distance D (unchanged) ---
    P_base = np.array([0.0, 0.0, 0.0])
    P_cyl = np.array(cylinder_center)
    V_axis = np.array(cylinder_axis)
    V_axis = V_axis / np.linalg.norm(V_axis)
    w = P_base - P_cyl
    D = np.linalg.norm(np.cross(w, V_axis))
    print(f"Perpendicular distance from base to cylinder axis D: {D:.2f}")

    # 2.1 --- Calculate straight-line distance from cylinder center to origin ---
    dist_center_to_origin = np.linalg.norm(P_cyl - P_base)
    print(
        f"Straight-line distance from cylinder center to origin: {dist_center_to_origin:.2f}")

    # 2.2 --- Calculate distance from cylinder surface to origin along connection line ---
    # Surface point = cylinder center + radius direction (toward origin)
    # Unit vector from cylinder center to origin
    direction_to_origin = (P_base - P_cyl) / np.linalg.norm(P_base - P_cyl)
    # Nearest surface point on connection line
    surface_point = P_cyl + cylinder_radius * direction_to_origin
    dist_surface_to_origin = np.linalg.norm(surface_point - P_base)
    print(
        f"Distance from cylinder surface to origin along connection line: {dist_surface_to_origin:.2f}")

    # 3. --- Calculate effective arm reach R_eff ---
    R_task = cylinder_radius + offset_dist # 圆柱工作半径（圆柱半径 + 喷涂距离）

    if Delta_Z_max >= robot_max_reach:
        return 0, "Error: Working height exceeds robot max reach"

    # R_eff = Corrected horizontal effective arm reach
    R_eff = np.sqrt(robot_max_reach ** 2 - Delta_Z_max ** 2) # 有效水平臂长
    print(f"Corrected effective horizontal arm reach R_eff: {R_eff:.2f}")

    # 4. --- Calculate angle using cosine law (unchanged) ---
    if R_eff < abs(D - R_task):
        return 0, "Error: Cannot reach surface"

    # R_task = b, D = c, R_eff = a
    cos_alpha = (R_task ** 2 + D ** 2 - R_eff ** 2) / (2 * R_task * D)

    cos_alpha = np.clip(cos_alpha, -1.0, 1.0)

    alpha = np.arccos(cos_alpha)
    total_angle = 2 * np.degrees(alpha)

    # Return additional data for visualization
    viz_data = {
        'P_base': P_base,
        'P_cyl': P_cyl,
        'V_axis': V_axis,
        'cylinder_radius': cylinder_radius,
        'R_task': R_task,
        'R_eff': R_eff,
        'robot_max_reach': robot_max_reach,
        'dist_center_to_origin': dist_center_to_origin,
        'dist_surface_to_origin': dist_surface_to_origin,
        'surface_point': surface_point,
        'D': D,
        'Delta_Z_max': Delta_Z_max,
        'Z_min_limit': Z_min_limit,
        'Z_max_limit': Z_max_limit,
        'alpha': alpha,
        'total_angle': total_angle
    }

    return total_angle, "Calculation successful", viz_data


def visualize_robot_cylinder(viz_data):
    """Visualize cylinder, connection lines, robot working range, etc."""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Extract data
    P_base = viz_data['P_base']
    P_cyl = viz_data['P_cyl']
    V_axis = viz_data['V_axis']
    cylinder_radius = viz_data['cylinder_radius']
    R_task = viz_data['R_task']
    R_eff = viz_data['R_eff']
    robot_max_reach = viz_data['robot_max_reach']
    surface_point = viz_data['surface_point']
    D = viz_data['D']
    Delta_Z_max = viz_data['Delta_Z_max']
    Z_min_limit = viz_data['Z_min_limit']
    Z_max_limit = viz_data['Z_max_limit']
    alpha = viz_data['alpha']

    # 1. Draw origin point (robot base)
    ax.scatter(*P_base, color='red', s=200,
               marker='o', label=f'Robot Base (0,0,0)')

    # 2. Draw cylinder center point
    ax.scatter(*P_cyl, color='blue', s=100, marker='^',
               label=f'Cylinder Center ({P_cyl[0]:.0f},{P_cyl[1]:.0f},{P_cyl[2]:.0f})')

    # 3. Draw cylinder surface point (nearest point on connection line)
    ax.scatter(*surface_point, color='green', s=100,
               marker='s', label=f'Nearest Surface Point')

    # 4. Draw connection line from origin to cylinder center
    ax.plot([P_base[0], P_cyl[0]], [P_base[1], P_cyl[1]], [P_base[2], P_cyl[2]],
            'b--', linewidth=2, label=f'Connection Line (length={viz_data["dist_center_to_origin"]:.0f}mm)')

    # 5. Draw connection line from origin to surface point
    ax.plot([P_base[0], surface_point[0]], [P_base[1], surface_point[1]], [P_base[2], surface_point[2]],
            'g-', linewidth=2, label=f'Surface Distance={viz_data["dist_surface_to_origin"]:.0f}mm')

    # 6. Draw cylinder body
    cylinder_height = 1500  # Cylinder height for visualization
    theta = np.linspace(0, 2 * np.pi, 50)
    z_cyl = np.linspace(-cylinder_height/2, cylinder_height/2, 20)
    Theta, Z_cyl = np.meshgrid(theta, z_cyl)

    # Build local coordinate system for cylinder
    z_axis = V_axis / np.linalg.norm(V_axis)
    # Find a vector not parallel to z_axis
    if abs(z_axis[0]) < 0.9:
        temp = np.array([1, 0, 0])
    else:
        temp = np.array([0, 1, 0])
    x_axis = np.cross(z_axis, temp)
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)

    # Cylinder surface points
    X_cyl = P_cyl[0] + cylinder_radius * \
        (np.cos(Theta) * x_axis[0] + np.sin(Theta)
         * y_axis[0]) + Z_cyl * z_axis[0]
    Y_cyl = P_cyl[1] + cylinder_radius * \
        (np.cos(Theta) * x_axis[1] + np.sin(Theta)
         * y_axis[1]) + Z_cyl * z_axis[1]
    Z_cyl_plot = P_cyl[2] + cylinder_radius * \
        (np.cos(Theta) * x_axis[2] + np.sin(Theta)
         * y_axis[2]) + Z_cyl * z_axis[2]

    ax.plot_surface(X_cyl, Y_cyl, Z_cyl_plot,
                    alpha=0.3, color='cyan')  # Cylinder

    # 7. Draw robot max working sphere (wireframe)
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 20)
    X_sphere = robot_max_reach * np.outer(np.cos(u), np.sin(v))
    Y_sphere = robot_max_reach * np.outer(np.sin(u), np.sin(v))
    Z_sphere = robot_max_reach * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_wireframe(X_sphere, Y_sphere, Z_sphere, color='red',
                      alpha=0.1, linewidth=0.5)  # Working sphere

    # 8. Draw effective horizontal arm reach circle (at Z=Delta_Z_max plane)
    theta_circle = np.linspace(0, 2 * np.pi, 100)
    X_eff = R_eff * np.cos(theta_circle)
    Y_eff = R_eff * np.sin(theta_circle)
    Z_eff_top = np.full_like(theta_circle, Delta_Z_max)
    Z_eff_bottom = np.full_like(theta_circle, -Delta_Z_max)
    ax.plot(X_eff, Y_eff, Z_eff_top, 'r-', linewidth=2,
            label=f'Effective Reach Circle R_eff={R_eff:.0f}mm (Z={Delta_Z_max:.0f})')

    # 9. Draw working height range
    ax.plot([P_cyl[0], P_cyl[0]], [P_cyl[1], P_cyl[1]], [Z_min_limit, Z_max_limit],
            'm-', linewidth=3, label=f'Working Height Range [{Z_min_limit:.0f}, {Z_max_limit:.0f}]')

    # 10. Draw R_task circle (cylinder radius + spray distance) projection on XY plane
    # Draw at cylinder center height
    X_task = P_cyl[0] + R_task * np.cos(theta_circle) * \
        x_axis[0] + R_task * np.sin(theta_circle) * y_axis[0]
    Y_task = P_cyl[1] + R_task * np.cos(theta_circle) * \
        x_axis[1] + R_task * np.sin(theta_circle) * y_axis[1]
    Z_task = np.full_like(theta_circle, P_cyl[2])
    ax.plot(X_task, Y_task, Z_task, 'orange', linewidth=2,
            linestyle='--', label=f'R_task={R_task:.0f}mm (radius+spray dist)')

    # 11. Draw perpendicular distance D (shortest distance from origin to cylinder axis)
    # Find nearest point on axis to P_base
    t = np.dot(P_base - P_cyl, V_axis)
    closest_point_on_axis = P_cyl + t * V_axis
    ax.plot([P_base[0], closest_point_on_axis[0]], [P_base[1], closest_point_on_axis[1]],
            [P_base[2], closest_point_on_axis[2]], 'purple', linewidth=2, linestyle=':',
            label=f'Perpendicular Distance D={D:.0f}mm')
    ax.scatter(*closest_point_on_axis, color='purple', s=80, marker='x')

    # 12. Draw coverage angle range (arc)
    # Draw coverage angle range on cylinder surface
    angle_range = np.linspace(-alpha, alpha, 30)
    # Find direction from axis to P_base projection on XY plane as start angle
    to_base = P_base - closest_point_on_axis
    to_base_horizontal = to_base - np.dot(to_base, V_axis) * V_axis
    to_base_horizontal = to_base_horizontal / \
        np.linalg.norm(to_base_horizontal)

    # 12.1 Fill the coverage area on cylinder surface with pink color
    # Coverage height for visualization (same as regional_height)
    coverage_height = 800
    theta_coverage = np.linspace(-alpha, alpha, 30)
    z_coverage = np.linspace(-coverage_height/2, coverage_height/2, 20)
    Theta_cov, Z_cov = np.meshgrid(theta_coverage, z_coverage)

    # Calculate coverage surface points
    X_cov = np.zeros_like(Theta_cov)
    Y_cov = np.zeros_like(Theta_cov)
    Z_cov_plot = np.zeros_like(Theta_cov)

    for i in range(Theta_cov.shape[0]):
        for j in range(Theta_cov.shape[1]):
            ang = Theta_cov[i, j]
            z_val = Z_cov[i, j]
            cos_a, sin_a = np.cos(ang), np.sin(ang)
            rot_vec = cos_a * to_base_horizontal + \
                sin_a * np.cross(V_axis, to_base_horizontal)
            point = P_cyl + cylinder_radius * rot_vec + z_val * z_axis
            X_cov[i, j] = point[0]
            Y_cov[i, j] = point[1]
            Z_cov_plot[i, j] = point[2]

    ax.plot_surface(X_cov, Y_cov, Z_cov_plot, alpha=0.5,
                    color='red')  # Coverage area in red

    # 12.2 Draw coverage boundary arcs
    for z_slice in [P_cyl[2] - 200, P_cyl[2], P_cyl[2] + 200]:
        arc_x = []
        arc_y = []
        arc_z = []
        for ang in angle_range:
            # Rotate to_base_horizontal around V_axis
            cos_a, sin_a = np.cos(ang), np.sin(ang)
            rot_vec = cos_a * to_base_horizontal + \
                sin_a * np.cross(V_axis, to_base_horizontal)
            point = P_cyl + cylinder_radius * rot_vec
            point[2] = z_slice  # Simplified: fixed z value
            arc_x.append(point[0])
            arc_y.append(point[1])
            arc_z.append(point[2])
        ax.plot(arc_x, arc_y, arc_z, 'lime', linewidth=3, alpha=0.8)

    # Set axis labels
    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Y (mm)', fontsize=12)
    ax.set_zlabel('Z (mm)', fontsize=12)
    ax.set_title(
        f'Robot Working Range & Cylinder Visualization\nCoverage Angle: {viz_data["total_angle"]:.2f} deg', fontsize=14)

    # Set equal axis scale
    max_range = max(robot_max_reach, abs(
        P_cyl[0]), abs(P_cyl[1]), abs(P_cyl[2])) * 1.2
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range/2, max_range])

    # Add proxy legend (solve plot_surface not supporting legend issue)
    cylinder_patch = mpatches.Patch(color='cyan', alpha=0.3, label='Cylinder')
    sphere_patch = mpatches.Patch(
        color='red', alpha=0.1, label=f'Robot Max Range R={robot_max_reach}mm')
    arc_patch = mpatches.Patch(
        color='lime', alpha=0.8, label=f'Coverage Boundary')
    coverage_patch = mpatches.Patch(
        color='red', alpha=0.5, label=f'Coverage Area ({viz_data["total_angle"]:.1f} deg)')

    # Get existing legend handles
    handles, labels = ax.get_legend_handles_labels()
    handles.extend([cylinder_patch, sphere_patch, arc_patch, coverage_patch])

    ax.legend(handles=handles, loc='upper left', fontsize=8)
    plt.tight_layout()
    plt.show()


def main():
    # TEST
    robot_reach = 1500
    angle, msg, viz_data = calculate_angle_in_robot_frame_corrected(
        robot_max_reach=robot_reach,
        # cylinder_center=[-996.669, -1112.73, 480.369],
        # cylinder_axis=[-0.00514, -0.02479, -0.999679],
        # cylinder_radius=373,
        # offset_dist=750,
        # regional_height=800  # Corrected total working height H

        cylinder_center=[324.69, 1127.41, 400.00],
        cylinder_axis=[0.0301, 0.0131, -0.9995],
        cylinder_radius=373,
        offset_dist=700,
        regional_height=800  # Corrected total working height H
    )

    print("-" * 30)
    print(f"Result: {msg}")
    print(f"Total coverage angle on cylinder surface: {angle:.2f} deg")

    # Visualization
    visualize_robot_cylinder(viz_data)


if __name__ == "__main__":
    main()

# Working range Z: [80.37, 880.37]
# Max vertical distance from base Delta_Z_max: 880.37
# Perpendicular distance from base to cylinder axis D: 1504.00
# Straight-line distance from cylinder center to origin: 1569.16
# Distance from cylinder surface to origin along connection line: 1218.16
# Corrected effective horizontal arm reach R_eff: 1088.55
# ------------------------------
# Result: Calculation successful
# Total coverage angle on cylinder surface: 92.73 deg


