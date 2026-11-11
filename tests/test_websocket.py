"""Exercise the public WebSocket entry point in a separate headless process."""
import asyncio
import json
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
import unittest

from websockets.asyncio.client import connect
from websockets.exceptions import ConnectionClosed

ROOT = Path(__file__).resolve().parents[1]


def frame(seq, *, gripper=0.6, engaged=True):
    return json.dumps({
        "type": "control", "seq": seq, "position": [0, 0, 0],
        "quaternion": [0, 0, 0, 1], "gripper": gripper, "engaged": engaged,
    })


async def receive_until(websocket, predicate, timeout=3.0):
    async with asyncio.timeout(timeout):
        while True:
            payload = json.loads(await websocket.recv())
            if predicate(payload):
                return payload


class WebSocketIntegrationTests(unittest.IsolatedAsyncioTestCase):
    @classmethod
    def setUpClass(cls):
        with socket.socket() as listener:
            listener.bind(("127.0.0.1", 0))
            cls.port = listener.getsockname()[1]
        cls.url = f"ws://127.0.0.1:{cls.port}"
        cls.output = tempfile.TemporaryFile(mode="w+")
        cls.process = subprocess.Popen(
            [sys.executable, "-m", "server", "--headless", "--host", "127.0.0.1", "--port", str(cls.port)],
            cwd=ROOT, stdout=cls.output, stderr=subprocess.STDOUT,
        )
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            if cls.process.poll() is not None:
                cls.output.seek(0)
                raise RuntimeError("Server exited during startup:\n" + cls.output.read())
            try:
                with socket.create_connection(("127.0.0.1", cls.port), timeout=0.2):
                    return
            except OSError:
                time.sleep(0.05)
        cls.process.terminate()
        cls.process.wait(timeout=5)
        raise RuntimeError("Server did not listen within 15 seconds")

    @classmethod
    def tearDownClass(cls):
        cls.process.terminate()
        try:
            cls.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            cls.process.kill()
            cls.process.wait(timeout=5)
        cls.output.close()

    async def test_control_status_timeout_and_disconnect(self):
        async with connect(self.url) as websocket:
            initial = await receive_until(websocket, lambda p: p.get("type") == "state")
            self.assertEqual(len(initial["ee_position"]), 3)
            self.assertIn("gripper_command", initial)
            await websocket.send(frame(0, gripper=0.35))
            await receive_until(websocket, lambda p: p.get("type") == "state" and p.get("tracking_active"))
            stopped = await receive_until(
                websocket, lambda p: p.get("type") == "state" and not p.get("tracking_active")
            )
            self.assertAlmostEqual(stopped["gripper_command"], 0.35)
        await asyncio.sleep(0.05)
        async with connect(self.url) as websocket:
            restored = await receive_until(websocket, lambda p: p.get("type") == "state")
            self.assertAlmostEqual(restored["gripper_command"], 0.35)
            self.assertFalse(restored["tracking_active"])
            # A new connection owns its own sequence space.
            await websocket.send(frame(0, gripper=0.35))
            await receive_until(websocket, lambda p: p.get("type") == "state" and p.get("tracking_active"))

    async def test_second_operator_cannot_override_first(self):
        async with connect(self.url) as owner:
            await receive_until(owner, lambda p: p.get("type") == "state")
            await owner.send(frame(0, gripper=0.42))
            await receive_until(owner, lambda p: p.get("type") == "state" and p.get("tracking_active"))
            async with connect(self.url) as other:
                with self.assertRaises(ConnectionClosed):
                    async with asyncio.timeout(2):
                        while True:
                            await other.recv()
            await owner.send(frame(1, gripper=0.42))
            state = await receive_until(owner, lambda p: p.get("type") == "state" and p.get("tracking_active"))
            self.assertAlmostEqual(state["gripper_command"], 0.42)

    async def test_malformed_json_returns_error_and_connection_recovers(self):
        async with connect(self.url) as websocket:
            await receive_until(websocket, lambda p: p.get("type") == "state")
            await websocket.send("{not valid json")
            error = await receive_until(websocket, lambda p: p.get("type") == "error")
            self.assertTrue(error.get("message"))
            await websocket.send(frame(0))
            await receive_until(websocket, lambda p: p.get("type") == "state" and p.get("tracking_active"))


if __name__ == "__main__":
    unittest.main()
