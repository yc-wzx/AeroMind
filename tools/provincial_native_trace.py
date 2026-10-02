"""Receive-only evidence with bounded asynchronous disk writes and fail-closed health.

Receive timestamps are captured before conversion/enqueue, never by the writer.
ROS message conversion remains in the callback; serialization and I/O do not.
"""
import json
import queue
import threading
import time
from collections import defaultdict
from contextlib import contextmanager
from rosidl_runtime_py.convert import message_to_ordereddict

HEALTH_SCHEMA = 1


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class NativeTrace:
    def __init__(self, path, queue_capacity=8192, close_timeout_s=10.0):
        self.stream = path.open('x', encoding='utf-8')
        self.timing_path = path.with_suffix('.timing.jsonl')
        self.health_path = path.with_suffix('.health.json')
        self.timing_stream = self.timing_path.open('x', encoding='utf-8')
        self.counts = defaultdict(int)
        self.written = defaultdict(int)
        self.operations = {}
        self.sim_s = None
        self.closed = False
        self.errors = []
        self.pending = queue.Queue(maxsize=queue_capacity)
        self.queue_high_water = 0
        self.timing_count = 0
        self.written_timing_count = 0
        self.max_queue_delay_ns = 0
        self.max_write_ns = 0
        self.close_timeout_s = close_timeout_s
        self.worker = threading.Thread(target=self._write_loop, daemon=True,
                                       name='native-evidence-writer')
        self.worker.start()

    def _enqueue(self, kind, row):
        if self.closed or self.errors:
            raise RuntimeError('native trace unavailable: ' + repr(self.errors))
        try:
            self.pending.put_nowait((kind, row, time.monotonic_ns()))
        except queue.Full:
            self.errors.append('bounded evidence queue overflow; evidence incomplete')
            raise RuntimeError(self.errors[-1])
        self.queue_high_water = max(self.queue_high_water, self.pending.qsize())

    def _write_loop(self):
        try:
            while True:
                item = self.pending.get()
                if item is None:
                    break
                kind, row, queued = item
                started = time.monotonic_ns()
                self.max_queue_delay_ns = max(self.max_queue_delay_ns, started-queued)
                stream = self.stream if kind == 'message' else self.timing_stream
                stream.write(json.dumps(row, ensure_ascii=False, allow_nan=True) + '\n')
                if kind == 'message':
                    self.written[row['topic']] += 1
                else:
                    self.written_timing_count += 1
                self.max_write_ns = max(self.max_write_ns, time.monotonic_ns()-started)
        except Exception as error:
            self.errors.append('writer failure: ' + repr(error))
        finally:
            for stream in (self.stream, self.timing_stream):
                try:
                    stream.flush()
                    stream.close()
                except Exception as error:
                    self.errors.append('flush/close failure: ' + repr(error))

    @contextmanager
    def observe(self, label):
        start = time.monotonic_ns()
        try:
            yield
        finally:
            end = time.monotonic_ns()
            stats = self.operations.setdefault(label, {'count': 0, 'total_ns': 0,
                                                       'max_ns': 0})
            stats['count'] += 1
            stats['total_ns'] += end-start
            stats['max_ns'] = max(stats['max_ns'], end-start)
            # Every measured operation is preserved; this is diagnostic data,
            # not fabricated ROS messages or replacement receive timestamps.
            self.timing_count += 1
            self._enqueue('timing', {'operation': label, 'start_monotonic_ns': start,
                                    'end_monotonic_ns': end, 'duration_ns': end-start,
                                    'receive_sim_s': self.sim_s})

    def record(self, topic, msg):
        received_monotonic_ns = time.monotonic_ns()
        received_wall_ns = time.time_ns()
        if topic == '/clock':
            self.sim_s = seconds(msg.clock)
        self.counts[topic] += 1
        header = getattr(msg, 'header', None)
        message_stamp = (seconds(header.stamp) if header is not None else
                         seconds(msg.clock) if topic == '/clock' else None)
        row = {
            'topic': topic, 'receive_sequence': self.counts[topic],
            'receive_monotonic_ns': received_monotonic_ns,
            'receive_wall_ns': received_wall_ns, 'receive_sim_s': self.sim_s,
            'message_stamp_s': message_stamp,
            'message_stamp_basis': ('header' if header is not None else
                                    'clock' if topic == '/clock' else
                                    'none_receive_time_only'),
            'data': message_to_ordereddict(msg),
        }
        self._enqueue('message', row)

    def close(self):
        if self.closed:
            return self.health
        self.closed = True
        deadline = time.monotonic() + self.close_timeout_s
        while self.worker.is_alive():
            try:
                self.pending.put(None, timeout=min(.05, max(0, deadline-time.monotonic())))
                break
            except queue.Full:
                if time.monotonic() >= deadline:
                    break
        self.worker.join(timeout=max(0, deadline-time.monotonic()))
        if self.worker.is_alive():
            self.errors.append('writer close timeout; files not confirmed closed')
        complete = (not self.worker.is_alive() and not self.errors and
                    dict(self.counts) == dict(self.written) and
                    self.timing_count == self.written_timing_count)
        self.health = {
            'schema': HEALTH_SCHEMA, 'pass': complete, 'errors': self.errors,
            'received_counts': dict(self.counts), 'written_counts': dict(self.written),
            'timing_count': self.timing_count,
            'written_timing_count': self.written_timing_count,
            'queue_capacity': self.pending.maxsize, 'queue_high_water': self.queue_high_water,
            'max_queue_delay_ns': self.max_queue_delay_ns, 'max_write_ns': self.max_write_ns,
            'worker_joined': not self.worker.is_alive(), 'operations': self.operations,
            'receive_time_basis': 'callback entry before conversion/enqueue',
            'publisher_loss_proven_absent': False,
        }
        with self.health_path.open('x', encoding='utf-8') as stream:
            json.dump(self.health, stream, indent=2)
        return self.health
