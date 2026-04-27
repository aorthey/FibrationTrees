import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Arc


def linf_dist(a, b):
    """L-infinity (Chebyshev) distance"""
    return np.max(np.abs(a - b))


def compute_angle_euclidean(q_near, q_rand, q_cand):
    """Original law of cosines using Euclidean distances (L2)"""
    d_to_rand = np.linalg.norm(q_rand - q_near)
    if d_to_rand < 1e-9:
        print("Warning: d_to_rand very small. Returning 0.")
        return 0.0

    d_to_cand = np.linalg.norm(q_cand - q_near)
    if d_to_cand < 1e-9:
        return np.pi

    d_cand_to_rand = np.linalg.norm(q_cand - q_rand)

    cos_theta = (d_to_cand**2 + d_to_rand**2 - d_cand_to_rand**2) / (2.0 * d_to_cand * d_to_rand)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    return np.arccos(cos_theta)


def compute_angle_linf(q_near, q_rand, q_cand):
    """Pseudo-angle adapted for L∞ metric (recommended for your planner)"""
    d_to_rand = linf_dist(q_near, q_rand)
    if d_to_rand < 1e-9:
        print("Warning: d_to_rand very small. Returning 0.")
        return 0.0

    d_to_cand = linf_dist(q_near, q_cand)
    if d_to_cand < 1e-9:
        return np.pi

    d_cand_to_rand = linf_dist(q_cand, q_rand)

    # Alignment score in [0, 1]  (1 = perfectly aligned)
    alignment = (d_to_cand + d_to_rand - d_cand_to_rand) / (2.0 * min(d_to_cand, d_to_rand))
    alignment = np.clip(alignment, 0.0, 1.0)

    # Convert to pseudo-angle in [0, π]
    return np.pi * (1.0 - alignment)


def visualize_angle(q_near, q_rand, q_cand, title="Angle Comparison"):
    fig, ax = plt.subplots(figsize=(11, 9))

    ax.plot(q_near[0], q_near[1], 'ko', markersize=12, label='q_near')
    ax.plot(q_rand[0], q_rand[1], 'ro', markersize=10, label='q_rand')
    ax.plot(q_cand[0], q_cand[1], 'bo', markersize=10, label='q_candidate')

    ax.plot([q_near[0], q_rand[0]], [q_near[1], q_rand[1]], 'r-', lw=2)
    ax.plot([q_near[0], q_cand[0]], [q_near[1], q_cand[1]], 'b-', lw=2)

    # Compute both angles
    angle_euc = compute_angle_euclidean(q_near, q_rand, q_cand)
    angle_linf = compute_angle_linf(q_near, q_rand, q_cand)

    # Draw arc for Euclidean angle (green)
    vec1 = q_rand - q_near
    start_angle = np.degrees(np.arctan2(vec1[1], vec1[0]))
    arc = Arc(q_near, width=0.8, height=0.8, angle=start_angle,
              theta1=0, theta2=np.degrees(angle_euc),
              color='green', lw=3, alpha=0.85)
    ax.add_patch(arc)

    # Labels
    ax.text(q_near[0] + 0.5, q_near[1] + 0.6,
            f'Euclidean (L2): {angle_euc:.5f} rad\n'
            f'L∞ pseudo-angle: {angle_linf:.5f} rad',
            fontsize=13, bbox=dict(boxstyle="round", facecolor="white", alpha=0.9))

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(f'{title}\nL∞ distances: '
                 f'{linf_dist(q_near,q_rand):.5f} | {linf_dist(q_near,q_cand):.5f} | {linf_dist(q_cand,q_rand):.5f}')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_aspect('equal')
    plt.tight_layout()
    plt.show()


# ========================
# Test with your real points
# ========================

if __name__ == "__main__":
    q_near = np.array([6.0, 5.0])
    q_rand = np.array([-3.06334, 4.41481])
    q_cand = np.array([0.43351, 4.29747])
    print(f"q_near     : {q_near}")
    print(f"q_rand     : {q_rand}")
    print(f"q_candidate: {q_cand}\n")

    print(f"Euclidean law of cosines : {compute_angle_euclidean(q_near, q_rand, q_cand):.5f} rad")
    print(f"L∞ pseudo-angle          : {compute_angle_linf(q_near, q_rand, q_cand):.5f} rad\n")

    print("L∞ distances:")
    print(f"  near→rand   : {linf_dist(q_near, q_rand):.5f}")
    print(f"  near→cand   : {linf_dist(q_near, q_cand):.5f}")
    print(f"  cand→rand   : {linf_dist(q_cand, q_rand):.5f}")

    # Show visualization
    visualize_angle(q_near, q_rand, q_cand,
                   title="Comparison: Euclidean vs L∞ pseudo-angle")
