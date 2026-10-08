from pathlib import Path

import numpy as np
import yaml

from camera_calibration.storage import Store


def test_zero_reference_survives_save(tmp_path):
    source = Path(__file__).resolve().parents[2]
    data = yaml.safe_load((source / 'camera_calibration/config/calibration.yaml').read_text())
    reference = data['head_imu_reference']
    assert set(reference) == {'head_zero', 'torso_zero'}
    for value in reference.values():
        assert np.asarray(value).shape == (3,)
        assert np.isfinite(value).all()
    store = Store(tmp_path / 'session', tmp_path / 'calibration.yaml')
    store.write_calibration(data)
    store.put_extrinsic('test_camera', {'parent': 'test_link'})
    assert store.read_calibration()['head_imu_reference'] == reference