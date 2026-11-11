"""Run with `uv run mjpython -m server`, or `uv run python -m server --headless`."""
from __future__ import annotations

import argparse
import asyncio
from contextlib import nullcontext
import json
import signal
import time

from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

from .simulation import Simulation


class TeleopServer:
    def __init__(self, simulation: Simulation, viewer=None):
        self.simulation = simulation
        self.viewer = viewer
        self.operator = None
        self.stop = asyncio.Event()

    def physics_lock(self):
        return self.viewer.lock() if self.viewer is not None else nullcontext()

    async def receive_controls(self, connection):
        async for payload in connection:
            try:
                if not isinstance(payload, str):
                    raise ValueError("send JSON text messages")
                message = json.loads(payload)
                with self.physics_lock():
                    self.simulation.apply_control(message)
            except (ValueError, TypeError, OverflowError) as error:
                await connection.send(json.dumps({"type": "error", "message": str(error)}, ensure_ascii=False))

    async def send_states(self, connection):
        while True:
            # Snapshot and events are copied on the same event-loop thread as physics.
            with self.physics_lock():
                events = self.simulation.pop_events()
                state = self.simulation.state()
            for event in events:
                await asyncio.wait_for(connection.send(json.dumps(event)), timeout=1.0)
            await asyncio.wait_for(connection.send(json.dumps(state, ensure_ascii=False)), timeout=1.0)
            await asyncio.sleep(0.05)

    async def handle_connection(self, connection):
        if self.operator is not None:
            await connection.send(json.dumps({"type": "error", "message": "已有手机连接，请先断开当前操作者"}, ensure_ascii=False))
            await connection.close(code=1008, reason="Another operator is already connected")
            return
        self.operator = connection
        with self.physics_lock():
            self.simulation.pop_events()
            state = self.simulation.state()
        print(f"Phone connected: {connection.remote_address}", flush=True)
        sender = receiver = None
        try:
            # The first state carries the existing gripper target for a safe reconnect.
            await connection.send(json.dumps(state, ensure_ascii=False))
            sender = asyncio.create_task(self.send_states(connection))
            receiver = asyncio.create_task(self.receive_controls(connection))
            finished, pending = await asyncio.wait([sender, receiver], return_when=asyncio.FIRST_COMPLETED)
            for task in finished:
                task.result()
        except (ConnectionClosed, asyncio.TimeoutError):
            pass
        finally:
            for task in (sender, receiver):
                if task is not None:
                    task.cancel()
            await asyncio.gather(*(task for task in (sender, receiver) if task is not None), return_exceptions=True)
            with self.physics_lock():
                self.simulation.disconnect()
            self.operator = None
            await connection.close()
            print("Phone disconnected; holding arm and gripper target.", flush=True)

    async def advance_physics(self):
        interval = 0.01
        deadline = time.monotonic()
        last_render = deadline
        last_tracking = self.simulation.tracking_active
        last_grasp = self.simulation.grasped
        while not self.stop.is_set():
            now = time.monotonic()
            if self.viewer is not None and not self.viewer.is_running():
                self.stop.set()
                return
            with self.physics_lock():
                self.simulation.step(interval, now=now)
                tracking = self.simulation.tracking_active
                grasped = self.simulation.grasped
            if tracking != last_tracking:
                print(f"Tracking {'ACTIVE' if tracking else 'HOLD'}: {self.simulation.message}", flush=True)
                last_tracking = tracking
            if grasped != last_grasp:
                print(f"Grasp {'ACQUIRED' if grasped else 'RELEASED'}: force={self.simulation.grasp_force:.2f} N", flush=True)
                last_grasp = grasped
            if self.viewer is not None and now - last_render >= 1 / 60:
                self.viewer.sync()
                last_render = now
            deadline += interval
            # Never fast-forward a long suspension with stale control input.
            if deadline < now - interval:
                deadline = now + interval
            await asyncio.sleep(max(0.0, deadline - time.monotonic()))

    async def run(self, host: str, port: int):
        loop = asyncio.get_running_loop()
        for signum in (signal.SIGINT, signal.SIGTERM):
            try:
                loop.add_signal_handler(signum, self.stop.set)
            except NotImplementedError:
                pass
        async with serve(self.handle_connection, host, port, max_size=4096, max_queue=2, ping_interval=10, ping_timeout=5):
            print(f"Franka teleoperation ready on ws://{host}:{port}", flush=True)
            print("Hold the phone control button to move. Release or 0.5 s without data holds the arm.", flush=True)
            physics = asyncio.create_task(self.advance_physics())
            stopping = asyncio.create_task(self.stop.wait())
            try:
                completed, _ = await asyncio.wait([physics, stopping], return_when=asyncio.FIRST_COMPLETED)
                for task in completed:
                    task.result()
            finally:
                self.stop.set()
                physics.cancel()
                stopping.cancel()
                await asyncio.gather(physics, stopping, return_exceptions=True)
                if self.operator is not None:
                    await self.operator.close(code=1001, reason="Simulator stopped")


def main():
    parser = argparse.ArgumentParser(description="iPhone ARKit → Franka Panda MuJoCo teleoperation")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8765, type=int)
    parser.add_argument("--headless", action="store_true", help="Run physics and networking without a viewer")
    arguments = parser.parse_args()
    simulation = Simulation()
    if arguments.headless:
        asyncio.run(TeleopServer(simulation).run(arguments.host, arguments.port))
        return
    import mujoco.viewer
    with mujoco.viewer.launch_passive(simulation.model, simulation.data) as viewer:
        with viewer.lock():
            viewer.cam.lookat[:] = [0.4, 0.0, 0.35]
            viewer.cam.distance = 1.65
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25
        asyncio.run(TeleopServer(simulation, viewer).run(arguments.host, arguments.port))


if __name__ == "__main__":
    main()
