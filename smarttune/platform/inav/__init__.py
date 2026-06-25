"""
smarttune/platform/inav/__init__.py

INAV Blackbox 日志适配器 — BBL/BFL 格式。
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Dict, List, Optional, Set

import numpy as np

from smarttune.platform.base import PlatformAdapter
from smarttune.platform.registry import register
from smarttune.models.flight_data import AxisPIDSignal, FlightData, ModeChange
from smarttune.errors import (
    LogFileNotFoundError, LogFileCorruptError, ParseError,
    InsufficientPIDDataError,
)
from smarttune.platform.betaflight import _sanitize_signal

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# INAV 参数映射表
# ---------------------------------------------------------------------------

_PARAM_MAP_TO_PLATFORM = {
    "pid.roll.p":    "pid_roll_p",
    "pid.roll.i":    "pid_roll_i",
    "pid.roll.d":    "pid_roll_d",
    "pid.roll.ff":   "pid_roll_f",
    "pid.pitch.p":   "pid_pitch_p",
    "pid.pitch.i":   "pid_pitch_i",
    "pid.pitch.d":   "pid_pitch_d",
    "pid.pitch.ff":  "pid_pitch_f",
    "pid.yaw.p":     "pid_yaw_p",
    "pid.yaw.i":     "pid_yaw_i",
    "pid.yaw.d":     "pid_yaw_d",
    "pid.yaw.ff":    "pid_yaw_f",
    "filter.gyro_lpf": "gyro_lpf_hz",
    "filter.dterm_lpf": "dterm_lpf_hz",
}

_PARAM_MAP_TO_GENERIC = {v: k for k, v in _PARAM_MAP_TO_PLATFORM.items()}

# INAV 模式映射
_MODE_MAP = {
    "MANUAL": "manual",
    "ANGLE": "stabilize",
    "HORIZON": "horizon",
    "ACRO": "acro",
    "NAV WP": "auto",
    "NAV ALTHOLD": "althold",
    "NAV RTH": "rtl",
    "NAV CRUISE": "cruise",
}

_BBL_MAGIC = b"H Product:Blackbox"

_GYRO_FIELD_NAMES = {
    "roll":  ["gyroADC[0]", "gyroADC_0", "gyroData[0]"],
    "pitch": ["gyroADC[1]", "gyroADC_1", "gyroData[1]"],
    "yaw":   ["gyroADC[2]", "gyroADC_2", "gyroData[2]"],
}

_SETPOINT_FIELD_NAMES = {
    "roll":  ["setpoint[0]", "rcCommand[0]"],
    "pitch": ["setpoint[1]", "rcCommand[1]"],
    "yaw":   ["setpoint[2]", "rcCommand[2]"],
}

_PID_P_FIELDS = {
    "roll": ["axisP[0]"], "pitch": ["axisP[1]"], "yaw": ["axisP[2]"],
}
_PID_I_FIELDS = {
    "roll": ["axisI[0]"], "pitch": ["axisI[1]"], "yaw": ["axisI[2]"],
}
_PID_D_FIELDS = {
    "roll": ["axisD[0]"], "pitch": ["axisD[1]"], "yaw": ["axisD[2]"],
}
_PID_F_FIELDS = {
    "roll": ["axisF[0]"], "pitch": ["axisF[1]"], "yaw": ["axisF[2]"],
}

_ACCEL_FIELDS = {
    "x": ["accSmooth[0]", "accData[0]"],
    "y": ["accSmooth[1]", "accData[1]"],
    "z": ["accSmooth[2]", "accData[2]"],
}

_MOTOR_FIELDS = ["motor[0]", "motor[1]", "motor[2]", "motor[3]",
                 "motor[4]", "motor[5]", "motor[6]", "motor[7]"]


def _resolve_field_name(field_names, candidates):
    for name in candidates:
        if name in field_names:
            return name
    return None


@register
class INAVAdapter(PlatformAdapter):
    """INAV Blackbox 日志适配器。"""

    @property
    def name(self) -> str:
        return "inav"

    @property
    def display_name(self) -> str:
        return "INAV"

    @property
    def supported_extensions(self) -> list[str]:
        return [".bbl", ".bfl", ".txt"]

    @classmethod
    def detect(cls, path: Path) -> bool:
        """检测是否为 INAV Blackbox 日志。"""
        if not path.is_file():
            return False

        suffix = path.suffix.lower()
        if suffix not in (".bbl", ".bfl", ".txt"):
            return False

        try:
            with open(path, "rb") as f:
                header = f.read(16384)
            # 必须包含 BBL 幻数且包含 INAV 标识
            return _BBL_MAGIC in header and b"INAV" in header
        except (OSError, IOError):
            return False

    def parse(self, path: Path, segment_index: int = 0) -> FlightData:
        """解析 INAV Blackbox 日志 → FlightData。"""
        from smarttune.platform.betaflight.bbl_parser import (
            parse_bbl_columnar, get_primary_mode,
            EVENT_FLIGHT_MODE, BBLColumnarSegment,
            FRAME_TYPE_I, FRAME_TYPE_P,
        )

        if not path.is_file():
            raise LogFileNotFoundError(
                message=f"Log file not found: {path}",
                hint="Check the file path and ensure the file exists.",
            )

        try:
            data = path.read_bytes()
        except OSError as exc:
            raise LogFileCorruptError(
                message=f"Cannot read log file: {exc}",
                hint="Ensure the file is readable and not locked.",
            )

        if _BBL_MAGIC not in data[:128]:
            raise LogFileCorruptError(
                message="Not a valid Blackbox log",
                hint="Ensure this is a .bbl/.bfl file.",
            )

        try:
            segments = parse_bbl_columnar(data, max_segments=10)
        except Exception as exc:
            raise ParseError(
                message=f"INAV BBL parse failed: {exc}",
                hint="The log file may be corrupted or use an unsupported format version.",
            )

        del data

        if not segments:
            raise ParseError(
                message="No valid log segment found",
                hint="The log file may be empty or corrupted.",
            )

        # Merge columns from all segments
        header = segments[0].header
        all_events = []
        column_chunks: List[BBLColumnarSegment] = []
        total_frames = 0

        for seg in segments:
            if seg.n_frames > 0:
                column_chunks.append(seg)
                total_frames += seg.n_frames
            all_events.extend(seg.events)

        if total_frames < 10:
            raise InsufficientPIDDataError(
                message=f"Too few frames in log ({total_frames})",
                hint="The flight recording may have been too short.",
            )

        if len(column_chunks) == 1:
            merged_columns = column_chunks[0].columns
            merged_ft = column_chunks[0].frame_types
            available_fields = column_chunks[0].available_fields
        else:
            available_fields = column_chunks[0].available_fields
            merged_columns = {}
            for name in column_chunks[0].field_names:
                merged_columns[name] = np.concatenate(
                    [seg.columns[name] for seg in column_chunks if name in seg.columns]
                )
            merged_ft = np.concatenate([seg.frame_types for seg in column_chunks])

        n_frames = total_frames

        # ── 提取参数 (从头信息) ────────────────────
        params: Dict[str, float] = {}
        for key, value in header.properties.items():
            try:
                params[key] = float(value)
            except (ValueError, TypeError):
                pass

        # Parse old-style compound PID parameters (e.g. rollPID: 71,127,67,90)
        for axis in ("roll", "pitch", "yaw"):
            prop_name = f"{axis}PID"
            if prop_name in header.properties:
                parts = header.properties[prop_name].split(",")
                if len(parts) >= 3:
                    try:
                        params[f"pid.{axis}.p"] = float(parts[0])
                        params[f"pid.{axis}.i"] = float(parts[1])
                        params[f"pid.{axis}.d"] = float(parts[2])
                        if len(parts) >= 4:
                            params[f"pid.{axis}.ff"] = float(parts[3])
                    except ValueError:
                        pass

        # Inject generic keys into params dictionary
        for _generic, _plat in _PARAM_MAP_TO_PLATFORM.items():
            if _plat in params and _generic not in params:
                params[_generic] = params[_plat]

        # Calculate time series
        loop_rate_hz = params.get("looptime", 500)
        if loop_rate_hz > 100:
            sample_rate_hz = 1_000_000.0 / loop_rate_hz
        else:
            sample_rate_hz = loop_rate_hz

        p_interval_str = header.properties.get("P interval", "")
        if '/' in p_interval_str:
            try:
                parts = p_interval_str.split('/')
                p_num = int(parts[0])
                p_denom = int(parts[1])
                if p_denom > 0 and p_num > 0:
                    sample_rate_hz = sample_rate_hz * p_num / p_denom
            except (ValueError, IndexError):
                pass
        elif p_interval_str.strip():
            try:
                p_denom = int(p_interval_str.strip())
                if p_denom > 1:
                    sample_rate_hz = sample_rate_hz / p_denom
            except ValueError:
                pass

        dt_s = 1.0 / sample_rate_hz
        timestamps_s = np.arange(n_frames, dtype=np.float64) * dt_s

        def _col_f64(name: str) -> Optional[np.ndarray]:
            arr = merged_columns.get(name)
            if arr is not None:
                return arr.astype(np.float64)
            return None

        # ── 提取 PID 信号 ──────────────────────────
        pid_data: Dict[str, AxisPIDSignal] = {}

        for axis in ["roll", "pitch", "yaw"]:
            gyro_name = _resolve_field_name(available_fields, _GYRO_FIELD_NAMES[axis])
            sp_name = _resolve_field_name(available_fields, _SETPOINT_FIELD_NAMES[axis])

            if gyro_name is None and sp_name is None:
                continue

            actual = _col_f64(gyro_name) if gyro_name else np.zeros(n_frames)
            desired = _col_f64(sp_name) if sp_name else np.zeros(n_frames)

            actual = _sanitize_signal(actual, max_abs=2000.0)
            desired = _sanitize_signal(desired, max_abs=2000.0)

            p_name = _resolve_field_name(available_fields, _PID_P_FIELDS[axis])
            i_name = _resolve_field_name(available_fields, _PID_I_FIELDS[axis])
            d_name = _resolve_field_name(available_fields, _PID_D_FIELDS[axis])
            f_name = _resolve_field_name(available_fields, _PID_F_FIELDS[axis])

            p_term = _sanitize_signal(_col_f64(p_name), max_abs=2000.0) if p_name else None
            i_term = _sanitize_signal(_col_f64(i_name), max_abs=2000.0) if i_name else None
            d_term = _sanitize_signal(_col_f64(d_name), max_abs=2000.0) if d_name else None
            ff_term = _sanitize_signal(_col_f64(f_name), max_abs=2000.0) if f_name else None

            pid_data[axis] = AxisPIDSignal(
                timestamp_s=timestamps_s.copy(),
                desired=desired,
                actual=actual,
                p_term=p_term,
                i_term=i_term,
                d_term=d_term,
                ff_term=ff_term,
            )

        # ── 提取 gyro IMU 数据 ─────────────────────
        gyro_x_name = _resolve_field_name(available_fields, _GYRO_FIELD_NAMES["roll"])
        gyro_y_name = _resolve_field_name(available_fields, _GYRO_FIELD_NAMES["pitch"])
        gyro_z_name = _resolve_field_name(available_fields, _GYRO_FIELD_NAMES["yaw"])

        gyro = None
        if gyro_x_name and gyro_y_name and gyro_z_name:
            gx = _sanitize_signal(_col_f64(gyro_x_name), max_abs=2000.0)
            gy = _sanitize_signal(_col_f64(gyro_y_name), max_abs=2000.0)
            gz = _sanitize_signal(_col_f64(gyro_z_name), max_abs=2000.0)
            gyro = np.column_stack([gx, gy, gz])
            del gx, gy, gz

        # ── 提取加速度并进行降噪清洗 ─────────────────────────────
        acc_x_name = _resolve_field_name(available_fields, _ACCEL_FIELDS["x"])
        acc_y_name = _resolve_field_name(available_fields, _ACCEL_FIELDS["y"])
        acc_z_name = _resolve_field_name(available_fields, _ACCEL_FIELDS["z"])

        accel = None
        if acc_x_name and acc_y_name and acc_z_name:
            ax = _col_f64(acc_x_name)
            ay = _col_f64(acc_y_name)
            az = _col_f64(acc_z_name)
            acc_1g_raw = params.get("acc_1G", 2048)
            try:
                acc_1g_raw = int(acc_1g_raw)
            except (ValueError, TypeError):
                acc_1g_raw = 2048
            
            # 清洗加速度计中由于 delta 编码解码损坏导致的极端跳变值 (限制在 16G 范围内)
            max_acc_abs = 16.0 * acc_1g_raw
            ax = _sanitize_signal(ax, max_abs=max_acc_abs)
            ay = _sanitize_signal(ay, max_abs=max_acc_abs)
            az = _sanitize_signal(az, max_abs=max_acc_abs)

            accel = np.column_stack([ax, ay, az]) * (9.80665 / acc_1g_raw)
            del ax, ay, az

        # ── 提取电机/舵机输出 ────────────────────────────
        motor_output = None
        motor_names = [m for m in _MOTOR_FIELDS if m in available_fields]
        if motor_names:
            motor_cols = [_col_f64(mn) for mn in motor_names]
            motor_output = np.column_stack(motor_cols)
            del motor_cols

        i_frame_count = int(np.sum(merged_ft == FRAME_TYPE_I))
        p_frame_count = int(np.sum(merged_ft == FRAME_TYPE_P))
        del merged_columns, merged_ft

        # ── 提取飞行模式 ──────────────────────────
        mode_changes: List[ModeChange] = []
        for event in all_events:
            if event.event_type == EVENT_FLIGHT_MODE:
                flags = event.data.get("flags", 0)
                raw_mode = get_primary_mode(flags)
                unified = _MODE_MAP.get(raw_mode, raw_mode.lower())
                mode_changes.append(ModeChange(
                    timestamp_s=0.0,
                    mode_name=unified,
                    raw_mode=raw_mode,
                ))

        # ── 组装 FlightData ─────────────────────────
        duration_s = float(timestamps_s[-1]) if len(timestamps_s) > 1 else 0.0

        flight_data = FlightData(
            platform="inav",
            firmware_version=header.firmware_revision,
            board_name=header.board_info or "",
            log_file=str(path),
            sample_rate_hz=sample_rate_hz,
            duration_s=duration_s,
            pid=pid_data,
            gyro=gyro,
            accel=accel,
            imu_timestamp_s=timestamps_s,
            motor_output=motor_output,
            motor_timestamp_s=timestamps_s if motor_output is not None else None,
            mode_changes=mode_changes,
            params=params,
            extras={
                "craft_name": header.craft_name,
                "firmware_type": header.firmware_type,
                "data_version": header.data_version,
                "frame_count": n_frames,
                "i_frame_count": i_frame_count,
                "p_frame_count": p_frame_count,
                "event_count": len(all_events),
            },
        )

        # ── 自动检测机型 (Fixed-Wing vs Multirotor) ──────────
        is_fixed_wing = False
        for f in available_fields:
            if f.startswith("servo") or f.startswith("fw"):
                is_fixed_wing = True
                break
        
        if is_fixed_wing:
            flight_data.frame_type = "fixed_wing"
        else:
            flight_data.frame_type = "quad"  # Multirotor default

        return flight_data

    def map_param_to_platform(self, generic_name: str) -> str:
        return _PARAM_MAP_TO_PLATFORM.get(generic_name, generic_name)

    def map_param_to_generic(self, platform_name: str) -> str:
        return _PARAM_MAP_TO_GENERIC.get(platform_name, platform_name)

    def capabilities(self) -> Set[str]:
        return {"pid", "fft", "filter", "hardware", "quality"}

    def param_table(self):
        from smarttune.platform.params import ParamTable
        return ParamTable.from_knowledge("inav")
