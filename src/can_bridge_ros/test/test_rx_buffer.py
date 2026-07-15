import threading
import unittest

from can_bridge_ros.rx_buffer import LatestFrameBuffer


class LatestFrameBufferTest(unittest.TestCase):
    def test_returns_fifo_batches(self) -> None:
        buffer = LatestFrameBuffer[int](4)
        buffer.put(1)
        buffer.put(2)
        buffer.put(3)

        self.assertEqual(buffer.get_batch(2), (1, 2))
        self.assertEqual(buffer.get_batch(2), (3,))

    def test_discards_oldest_item_when_full(self) -> None:
        buffer = LatestFrameBuffer[str](2)
        buffer.put("oldest")
        buffer.put("middle")

        buffer.put("latest")
        self.assertEqual(buffer.get_batch(2), ("middle", "latest"))

    def test_close_wakes_waiter_and_allows_drain(self) -> None:
        buffer = LatestFrameBuffer[int](2)
        result = []
        waiter = threading.Thread(
            target=lambda: result.extend(buffer.get_batch(1)))
        waiter.start()
        buffer.close()
        waiter.join(timeout=1.0)

        self.assertFalse(waiter.is_alive())
        self.assertEqual(result, [])
        self.assertTrue(buffer.closed_and_empty)
        with self.assertRaisesRegex(RuntimeError, "closed"):
            buffer.put(1)

    def test_rejects_invalid_sizes(self) -> None:
        with self.assertRaisesRegex(ValueError, "capacity"):
            LatestFrameBuffer(0)
        buffer = LatestFrameBuffer[int](1)
        with self.assertRaisesRegex(ValueError, "max_items"):
            buffer.get_batch(0)


if __name__ == "__main__":
    unittest.main()
