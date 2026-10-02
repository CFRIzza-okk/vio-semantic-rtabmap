"""载具抽象层：把"导航决策"和"怎么让载具动起来"彻底分开。

设计动机
--------
毕业设计要先把视觉和导航做完，无人机部分最后接。如果导航逻辑里到处
写 MAVROS 的话题名和消息格式，那就没法在飞控到位前验证任何东西，
而且换平台（比如从 PX4 换到 ArduPilot）要重写一半代码。

所以引入这一层抽象：

    navigator_node  ──调用──▶  VehicleInterface  ──┬─▶ MockVehicle
     (纯决策逻辑)                (统一接口)          └─▶ MavrosVehicle

导航层只认三个动作：发位置指令、读当前位姿、问能不能接收指令。
至于指令是变成 MAVROS 的 setpoint 话题、还是被虚拟积分器消化掉，
导航层完全不知道，也不该知道。

坐标约定
--------
所有位姿统一用 **map 帧下的 4x4 齐次变换矩阵** 表示。
用矩阵而不是 (x,y,z,yaw) 四元组，是因为这样位姿可以直接做矩阵乘法
组合，不需要反复在欧拉角和四元数之间转换，减少出错面。
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class VehicleState:
    """载具当前状态快照。"""

    # map 帧下的 4x4 位姿矩阵；还没收到里程计时为 None
    pose: Optional[np.ndarray] = None

    # 是否已经收到过里程计（判断"位姿是否可用"）
    has_pose: bool = False

    # 后端是否就绪（例如 MAVROS 已连接、已解锁、已切到 OFFBOARD）
    ready: bool = False

    # 后端自己上报的模式名，仅用于日志/调试
    mode: str = "unknown"


class VehicleInterface(ABC):
    """载具后端必须实现的接口。

    实现者需保证：
      - send_position_setpoint() 可以被周期性高速调用（典型 20-50 Hz），
        内部不应有阻塞操作
      - get_state() 是幂等的、非阻塞的，随时可以调用
    """

    #: 后端名字，用于日志
    name: str = "vehicle"

    @abstractmethod
    def send_position_setpoint(
        self, x: float, y: float, z: float, yaw: float
    ) -> bool:
        """发送一个 map 帧下的位置+偏航指令。

        返回 True 表示指令已交给后端；False 表示后端当前不接受指令
        （例如飞控未解锁、MAVROS 断连）。调用方据此决定是否报错，
        但不应因为一次失败就崩溃——瞬时失败很常见。
        """
        raise NotImplementedError

    @abstractmethod
    def get_state(self) -> VehicleState:
        """返回当前状态快照。"""
        raise NotImplementedError

    # ------------------------------------------------------------------
    # 以下为可选动作，默认实现是"什么都不做"，后端按需覆盖
    # ------------------------------------------------------------------
    def hold(self) -> None:
        """在原地悬停（保持当前位置）。

        默认实现什么都不做：对于位置指令型后端，只要继续发当前位置
        作为 setpoint 就自然实现了悬停。
        """
        return None

    def land(self) -> None:
        """原地降落。默认不支持。"""
        return None

    def return_home(self) -> None:
        """返回起点。默认不支持。"""
        return None

    def supports(self, action: str) -> bool:
        """查询后端是否支持某个动作。"""
        return action in ("setpoint", "hold")


def pose_to_matrix(x: float, y: float, z: float, yaw: float) -> np.ndarray:
    """由 (x, y, z, yaw) 构造 4x4 位姿矩阵（只绕 z 轴旋转）。"""
    c, s = np.cos(yaw), np.sin(yaw)
    T = np.eye(4)
    T[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    T[:3, 3] = [x, y, z]
    return T


def matrix_to_pose(T: np.ndarray) -> tuple:
    """由 4x4 位姿矩阵反解 (x, y, z, yaw)。"""
    x, y, z = float(T[0, 3]), float(T[1, 3]), float(T[2, 3])
    yaw = float(np.arctan2(T[1, 0], T[0, 0]))
    return x, y, z, yaw
