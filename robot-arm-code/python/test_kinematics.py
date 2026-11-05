import io
import contextlib
import numpy as np
from R2_forward_kinematics import forward_kinematics
from R2_inverse_kinematics import inverse_kinematics


def sample_case(rng, near_singularity=False):
    """Random link lengths with R1 > R2 and a reachable target t = [x, y]."""
    R1, R2 = np.sort(rng.uniform(0.5, 10.0, size=2))[::-1]
    r_min, r_max = R1 - R2, R1 + R2
    if near_singularity:
        # distance delta to full stretch or full fold, log-uniform in [1e-8, 1e-1]
        delta = 10.0 ** rng.uniform(-8, -1) * (r_max - r_min)
        norm_t = r_max - delta if rng.random() < 0.5 else r_min + delta
    else:
        norm_t = rng.uniform(r_min, r_max)
    theta = rng.uniform(-np.pi, np.pi)
    t = norm_t * np.array([np.cos(theta), np.sin(theta)])
    return R1, R2, t


def test_roundtrip(n_samples=100, frac_near_singular=0.3, tol=1e-9, seed=0):
    """Run IK, then FK, and compare the reached point with the target."""
    rng = np.random.default_rng(seed)
    errors, failures = [], []

    for i in range(n_samples):
        near = rng.random() < frac_near_singular
        R1, R2, t = sample_case(rng, near_singularity=near)

        phi1, phi2 = inverse_kinematics(t, R1, R2)
        if np.isnan(phi1) or np.isnan(phi2):
            failures.append((R1, R2, t, "IK returned NaN"))
            continue

        r = forward_kinematics(R1=R1, phi1=phi1, R2=R2, phi2=phi2)
        reached = r[1:3]                                   # r[0] is the homogeneous 1
        err = np.linalg.norm(reached - t) / (R1 + R2)      # relative to arm length
        errors.append(err)
        if err > tol:
            failures.append((R1, R2, t, f"relative error {err:.2e}"))

    errors = np.array(errors)
    print(f"samples: {n_samples}, failures: {len(failures)}")
    print(f"relative position error: max {errors.max():.2e}, mean {errors.mean():.2e}")
    for R1, R2, t, msg in failures[:10]:
        print(f"  R1={R1:.4f} R2={R2:.4f} t={t} -> {msg}")
    return failures


if __name__ == "__main__":
    test_roundtrip()