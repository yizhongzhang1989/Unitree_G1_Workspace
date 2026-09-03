import numpy as np

from g1_localization.ground_plane import fit_ground_plane


def test_ransac_recovers_ground_with_outliers():
    rng = np.random.default_rng(3)
    xy = rng.uniform(-2.0, 2.0, size=(800, 2))
    z = 0.12 + 0.03 * xy[:, 0] - 0.02 * xy[:, 1] + rng.normal(0.0, 0.004, 800)
    ground = np.column_stack((xy, z))
    outliers = rng.uniform([-2.0, -2.0, 0.4], [2.0, 2.0, 2.0], size=(250, 3))

    plane = fit_ground_plane(np.vstack((ground, outliers)))

    assert plane is not None
    np.testing.assert_allclose(plane.height_at(0.0, 0.0), 0.12, atol=0.01)


def test_ransac_rejects_vertical_wall():
    rng = np.random.default_rng(4)
    yz = rng.uniform(-2.0, 2.0, size=(500, 2))
    wall = np.column_stack((np.zeros(500), yz))
    assert fit_ground_plane(wall) is None