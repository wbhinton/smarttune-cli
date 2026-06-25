"""
tests/test_inav_adapter.py

INAV 适配器单元测试 — 覆盖 detect、参数映射、能力声明、
fixed_wing 自动检测及加速度降噪清洗回归校验。
"""

from pathlib import Path
import pytest
import numpy as np

from smarttune.platform.inav import INAVAdapter
from smarttune.platform.betaflight import BetaflightAdapter
from smarttune.knowledge import KnowledgeBase
from smarttune.services.analysis import run_module


def get_inav_log_path():
    for p in (Path("FPV31/LOG00001.TXT"), Path("../FPV31/LOG00001.TXT"), Path("/home/wbhinton/development/blackbox-tuning/FPV31/LOG00001.TXT")):
        if p.is_file():
            return p
    raise FileNotFoundError("Could not find FPV31/LOG00001.TXT")


def get_bf_log_path():
    for p in (Path("75frank/btfl_006.bbl"), Path("../75frank/btfl_006.bbl"), Path("/home/wbhinton/development/blackbox-tuning/75frank/btfl_006.bbl")):
        if p.is_file():
            return p
    return None


@pytest.fixture
def adapter():
    return INAVAdapter()


class TestDetect:
    def test_detect_inav_log(self, adapter):
        path = get_inav_log_path()
        assert adapter.detect(path) is True

    def test_detect_rejects_betaflight_log(self, adapter):
        path = get_bf_log_path()
        if path and path.is_file():
            assert adapter.detect(path) is False

    def test_betaflight_detect_rejects_inav_log(self):
        bf_adapter = BetaflightAdapter()
        path = get_inav_log_path()
        assert bf_adapter.detect(path) is False


class TestParamMapping:
    def test_param_mappings(self, adapter):
        assert adapter.map_param_to_platform("pid.roll.p") == "pid_roll_p"
        assert adapter.map_param_to_generic("pid_roll_p") == "pid.roll.p"
        assert adapter.map_param_to_platform("unknown") == "unknown"


class TestCapabilities:
    def test_capabilities(self, adapter):
        caps = adapter.capabilities()
        assert "pid" in caps
        assert "fft" in caps
        assert "filter" in caps
        assert "hardware" in caps
        assert "quality" in caps


class TestParseINAV:
    def test_parse_log_success(self, adapter):
        path = get_inav_log_path()
        fd = adapter.parse(path)

        assert fd.platform == "inav"
        assert "INAV 9.0.1" in fd.firmware_version
        assert fd.duration_s > 10.0
        assert fd.frame_type == "fixed_wing"

        # 校验 compound PID 是否正确解析
        # H rollPID:15,3,7,93
        assert fd.params["pid.roll.p"] == 15.0
        assert fd.params["pid.roll.i"] == 3.0
        assert fd.params["pid.roll.d"] == 7.0
        assert fd.params["pid.roll.ff"] == 93.0

        # 校验加速度计数据是否清洗成功（无超常噪声/跳变）
        assert fd.accel is not None
        # 单样本不能超过 16G (16 * 9.80665 ~ 156.9)
        assert np.max(np.abs(fd.accel)) < 160.0

        # 校验去均值后的振动 RMS 处于合理区间
        ax = fd.accel[:, 0] - np.mean(fd.accel[:, 0])
        ay = fd.accel[:, 1] - np.mean(fd.accel[:, 1])
        az = fd.accel[:, 2] - np.mean(fd.accel[:, 2])
        vibe_rms = np.sqrt(np.mean(ax**2) + np.mean(ay**2) + np.mean(az**2))
        assert vibe_rms < 10.0, f"Cleaned vibration RMS {vibe_rms} is too high"


class TestINAVRulesResolution:
    def test_resolution_fixed_wing(self):
        kb = KnowledgeBase(platform="inav")
        # 默认或 fixed_wing
        kb.resolve_inav_rules(frame_type="fixed_wing")
        
        # 检查是否正确选取了 fixed_wing_thresholds
        pid_rules = kb.get("pid_rules", {})
        thresholds = pid_rules.get("thresholds", {})
        assert thresholds.get("roll", {}).get("rise_time_ms", {}).get("ideal") == 120

    def test_resolution_multirotor(self):
        kb = KnowledgeBase(platform="inav")
        kb.resolve_inav_rules(frame_type="multirotor")
        
        # 检查是否正确选取了 multirotor_thresholds
        pid_rules = kb.get("pid_rules", {})
        thresholds = pid_rules.get("thresholds", {})
        assert thresholds.get("roll", {}).get("rise_time_ms", {}).get("ideal") == 80
