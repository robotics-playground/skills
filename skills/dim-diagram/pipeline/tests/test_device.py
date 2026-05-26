from dimgram.device import select_torch_device, device_info


def test_select_device_returns_valid_choice():
    assert select_torch_device() in ("cuda", "mps", "cpu")


def test_prefer_overrides():
    assert select_torch_device("cpu") == "cpu"


def test_device_info_shape():
    info = device_info()
    assert info["device"] in ("cuda", "mps", "cpu")
    assert set(["os", "machine", "cuda", "mps"]) <= set(info)
