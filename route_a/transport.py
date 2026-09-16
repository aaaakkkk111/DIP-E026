"""Replaceable byte-stream transports and a bandwidth-faithful MuJoCo link."""

from __future__ import annotations

import heapq
import random
import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass

from .firmware_profile import EnvironmentConfig
from .plant import MuJoCoPlant, OracleSample
from .protocol import BAUD
from .virtual_stm32 import VirtualSTM32


class Transport(ABC):
    @abstractmethod
    def write(self, data: bytes) -> None: ...
    @abstractmethod
    def read(self, maximum: int = 4096) -> bytes: ...
    @abstractmethod
    def advance(self, seconds: float) -> list[OracleSample]: ...
    @abstractmethod
    def close(self) -> None: ...


@dataclass
class ImpairmentConfig:
    latency_ms: float = 0.0
    byte_drop_probability: float = 0.0
    byte_corruption_probability: float = 0.0
    fragment_max_bytes: int = 0
    baud: int = BAUD

    def validate(self) -> None:
        if self.latency_ms < 0 or self.fragment_max_bytes < 0 or self.baud <= 0: raise ValueError("invalid transport timing")
        if not 0 <= self.byte_drop_probability <= 1 or not 0 <= self.byte_corruption_probability <= 1:
            raise ValueError("probabilities must be in [0,1]")


class MuJoCoTransport(Transport):
    """Virtual UART. Serialization naturally creates fragmentation/sticky frames."""

    def __init__(self, environment: EnvironmentConfig | None = None,
                 impairment: ImpairmentConfig | None = None) -> None:
        self.environment = environment or EnvironmentConfig(); self.impairment = impairment or ImpairmentConfig()
        self.impairment.validate(); self.stm32 = VirtualSTM32(); self.plant = MuJoCoPlant(self.environment)
        self.time_s = 0.0; self._seq = 0; self._pc_to_mcu: list[tuple[float,int,int]] = []
        self._mcu_to_pc: list[tuple[float,int,int]] = []; self._pc_rx = bytearray()
        self._monitor_rx = bytearray()
        self._next_pc_tx = 0.0; self._next_mcu_tx = 0.0; self.rng = random.Random(self.environment.seed)
        self.lock = threading.RLock(); self.closed = False
        self.stats = {"pc_tx_bytes":0,"pc_rx_bytes":0,"dropped_bytes":0,"corrupted_bytes":0,"frames_from_mcu":0}

    def _schedule(self, queue: list[tuple[float,int,int]], data: bytes, *, outgoing_pc: bool) -> None:
        next_time = self._next_pc_tx if outgoing_pc else self._next_mcu_tx
        next_time = max(next_time, self.time_s)+self.impairment.latency_ms/1000.0
        byte_period = 10.0/self.impairment.baud
        for value in data:
            next_time += byte_period
            if self.rng.random() < self.impairment.byte_drop_probability:
                self.stats["dropped_bytes"] += 1
                if not outgoing_pc: self.stm32.report_tx_drop()
                continue
            if self.rng.random() < self.impairment.byte_corruption_probability:
                value ^= 1 << self.rng.randrange(8); self.stats["corrupted_bytes"] += 1
            self._seq += 1; heapq.heappush(queue, (next_time, self._seq, value))
        if outgoing_pc: self._next_pc_tx = next_time
        else: self._next_mcu_tx = next_time

    def write(self, data: bytes) -> None:
        with self.lock:
            if self.closed: raise RuntimeError("transport closed")
            self.stats["pc_tx_bytes"] += len(data); self._schedule(self._pc_to_mcu, bytes(data), outgoing_pc=True)

    def _deliver(self) -> None:
        while self._pc_to_mcu and self._pc_to_mcu[0][0] <= self.time_s+1e-12:
            _,_,value = heapq.heappop(self._pc_to_mcu)
            for reply in self.stm32.receive(bytes((value,))):
                self.stats["frames_from_mcu"] += 1; self._schedule(self._mcu_to_pc, reply, outgoing_pc=False)
        while self._mcu_to_pc and self._mcu_to_pc[0][0] <= self.time_s+1e-12:
            _,_,value = heapq.heappop(self._mcu_to_pc); self._pc_rx.append(value); self._monitor_rx.append(value)
            if len(self._monitor_rx) > 65536: del self._monitor_rx[:-32768]
            self.stats["pc_rx_bytes"] += 1

    def advance(self, seconds: float) -> list[OracleSample]:
        if seconds < 0: raise ValueError("seconds must be non-negative")
        oracles: list[OracleSample] = []
        with self.lock:
            steps = round(seconds/self.plant.PHYSICS_TIMESTEP_S)
            for _ in range(steps):
                self._deliver(); oracle, frames = self.plant.step(self.stm32); oracles.append(oracle)
                self.time_s = float(self.plant.data.time)
                for frame in frames:
                    self.stats["frames_from_mcu"] += 1; self._schedule(self._mcu_to_pc, frame, outgoing_pc=False)
                self._deliver()
        return oracles

    def read(self, maximum: int = 4096) -> bytes:
        with self.lock:
            if maximum <= 0: return b""
            if self.impairment.fragment_max_bytes:
                maximum = min(maximum, self.rng.randint(1, self.impairment.fragment_max_bytes))
            chunk = bytes(self._pc_rx[:maximum]); del self._pc_rx[:maximum]; return chunk

    def read_monitor(self, maximum: int = 8192) -> bytes:
        """Non-consuming UI tap; it never steals bytes from the protocol client."""
        with self.lock:
            chunk=bytes(self._monitor_rx[:maximum]); del self._monitor_rx[:maximum]; return chunk

    def reset_scenario(self, environment: EnvironmentConfig | None = None, *, seed: int | None = None,
                       initial_pitch_deg: float = 3.0) -> None:
        with self.lock:
            if environment is not None:
                self.environment = environment; self.plant.configure(environment)
            selected_seed = self.environment.seed if seed is None else seed
            self.rng = random.Random(selected_seed); self.plant.reset(selected_seed, initial_pitch_deg)
            self.stm32.reset_runtime(); self.time_s = 0.0
            self._pc_to_mcu.clear(); self._mcu_to_pc.clear(); self._pc_rx.clear(); self._monitor_rx.clear()
            self._next_pc_tx = self._next_mcu_tx = 0.0

    def apply_push(self, direction: str, force_n: float | None = None) -> None:
        with self.lock: self.plant.apply_push(direction, force_n)

    def emergency_stop(self) -> None:
        with self.lock: self.stm32.stop_flag = True

    def close(self) -> None:
        with self.lock: self.closed = True


class SerialTransport(Transport):
    """Hardware adapter with the same interface; never used by unit tests."""

    def __init__(self, port: str, baud: int = BAUD) -> None:
        try: import serial
        except ImportError as exc: raise RuntimeError("install pyserial for SerialTransport") from exc
        self.serial = serial.Serial(port, baudrate=baud, bytesize=8, parity="N", stopbits=1, timeout=0)

    def write(self, data: bytes) -> None: self.serial.write(data)
    def read(self, maximum: int = 4096) -> bytes: return self.serial.read(maximum)
    def advance(self, seconds: float) -> list[OracleSample]: return []
    def close(self) -> None: self.serial.close()
