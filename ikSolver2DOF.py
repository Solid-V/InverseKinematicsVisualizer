import numpy as np
import matplotlib.pyplot as plt

#Link Lengths
L1 = 156.0
L2 = 164.0
PI = np.pi 

def ik_2dof(x, y):

    L = np.sqrt(x * x + y * y)

    # Check for reachability
    if L > (L1 + L2) or L < abs(L1 - L2):
        raise ValueError(f"Target ({x:.1f}, {y:.1f}) is out of reach!")

    a1_internal = np.arccos((L1 * L1 + L2 * L2 - L * L) / (2 * L1 * L2))

    a1 = PI - a1_internal

    a2 = np.arctan2(y, x) - np.arccos((L1 * L1 + L * L - L2 * L2) / (2 * L1 * L))

    # Convert to Degrees
    theta1 = a2 * 180.0 / PI  # Hip angle 
    theta2 = a1 * 180.0 / PI  # Knee angle 

    return theta1, theta2, a2, a1

def forward_kinematics_standard(theta1, theta2):
    t1 = np.radians(theta1)
    t2 = np.radians(theta2)

    # Joint 1 position (x1, y1)
    x1 = L1 * np.cos(t1)
    y1 = L1 * np.sin(t1)

    # Foot position (x2, y2)
    x2 = x1 + L2 * np.cos(t1 + t2)
    y2 = y1 + L2 * np.sin(t1 + t2)

    return (0, 0), (x1, y1), (x2, y2)

def angleToPos(angle):
    #Convert angles to digital positions
    total_pos = 4096.0
    digital_pos = (angle / 360) * total_pos
    return digital_pos

# Testing the target that gave us trouble:
X_TARGET, Y_TARGET = -1, -262

try:
    # Calculate IK angles
    theta1, theta2, a2, a1 = ik_2dof(X_TARGET, Y_TARGET)

    # Convert theta1 and theta2 to digital pos and set offsets
    s1_offset = -theta1 + 90 #400
    s2_offset = -theta2 + 270 + 30#- 16 -15
    digital_pos1 = angleToPos(s1_offset)
    digital_pos2 = angleToPos(s2_offset)

    # Calculate FK positions using the standard convention 
    joint0, joint1, foot = forward_kinematics_standard(theta1, theta2)
    calculated_x, calculated_y = foot[0], foot[1]

    plt.figure(figsize=(7, 7))

    # Plot the links
    plt.plot([joint0[0], joint1[0]], [joint0[1], joint1[1]], 'o-', lw=6, color='darkorange', label='Upper Leg (L1)')
    plt.plot([joint1[0], foot[0]], [joint1[1], foot[1]], 'o-', lw=6, color='purple', label='Lower Leg (L2)')

    # Plot the joints
    plt.scatter(0, 0, color='black', s=150, zorder=5, label='Hip Joint')
    plt.scatter(joint1[0], joint1[1], color='purple', s=80, zorder=5, label='Knee Joint')

    # Plot the target and final foot position
    plt.scatter(X_TARGET, Y_TARGET, color='red', s=200, marker='*', zorder=6, label='Target Position')
    plt.scatter(calculated_x, calculated_y, color='lime', s=100, zorder=6, label=f'Calculated Foot ({calculated_x:.1f}, {calculated_y:.1f})')

    # Print the angles needed for analysis
    print(f"Calculated Angles: θ1 = {theta1:.2f}°, θ2 = {theta2:.2f}°")
    print(f"Calculated Angles in radians : θ1 = {a2:.2f}°, θ2 = {a1:.2f}°")
    print(f"Calculated Position: ({calculated_x:.2f}, {calculated_y:.2f})")
    print(f"Calculated offset: ({s1_offset}, {s2_offset})")
    print(f"Calculated digital positions: ({digital_pos1}), ({digital_pos2})")

    # Final Plot Settings
    L_max = L1 + L2 + 20
    plt.xlim(-L_max, L_max)
    plt.ylim(-L_max, L_max)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.legend(loc='upper right')
    plt.xlabel("X (Horizontal)")
    plt.ylabel("Y (Vertical)")
    plt.title(f"2DOF IK (New Logic) with Standard FK\nTarget: ({X_TARGET}, {Y_TARGET})")
    plt.show()

except ValueError as e:
    print(f"Error: {e}")
