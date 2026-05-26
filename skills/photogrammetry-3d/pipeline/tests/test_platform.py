from photrecon.platform import detect_compute, doctor, ComputeConfig


def test_detect_returns_config():
    c = detect_compute()
    assert isinstance(c, ComputeConfig)
    assert c.sfm_backend == "colmap"
    assert c.dense_backend == "openmvs"        # OpenMVS dense on every platform
    assert isinstance(c.has_cuda, bool)
    assert c.threads >= 1


def test_force_cuda_sets_openmvs_cuda():
    assert detect_compute(force_cuda=True).openmvs_use_cuda is True
    assert detect_compute(force_cuda=False).openmvs_use_cuda is False


def test_doctor_mentions_backends():
    out = doctor()
    assert "colmap" in out.lower() and "openmvs" in out.lower()
