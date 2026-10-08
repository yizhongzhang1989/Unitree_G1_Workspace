"""Durable request identities, execution claims and immutable feedback."""

from __future__ import annotations

import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import sqlite3
import threading
import time
import uuid


def score_reward(value):
    text = value.strip() or '3'
    if text == '/null':
        return None
    if text not in ('1', '2', '3', '4', '5'):
        raise ValueError('Score must be 1..5, Enter (=3), or /null')
    return (int(text) - 3) / 2.0


class Journal:
    PENDING_FILTER = "json_extract(data, '$.state') NOT IN ('confirmed', 'expired', 'discarded')"
    RESPONSE_FIELDS = ('policy_version', 'batch_id', 'execution_chunk_size')

    def __init__(self, directory):
        self.directory = Path(directory).expanduser()
        self.directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        self._file_lock = (self.directory / 'session.lock').open('a')
        try:
            fcntl.flock(self._file_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self._file_lock.close()
            raise RuntimeError('This RL directory is already in use') from None
        self._lock = threading.RLock()
        path = self.directory / 'session.sqlite3'
        self._db = sqlite3.connect(path, check_same_thread=False)
        os.chmod(path, 0o600)
        self._db.execute('PRAGMA synchronous=FULL')
        self._db.execute('PRAGMA auto_vacuum=FULL')
        self._db.execute('''CREATE TABLE IF NOT EXISTS requests (
            request_id TEXT PRIMARY KEY, chunk_id TEXT UNIQUE, data TEXT NOT NULL,
            query BLOB NOT NULL, head BLOB NOT NULL, left_image BLOB NOT NULL,
            right_image BLOB NOT NULL)''')
        self._db.execute('CREATE TABLE IF NOT EXISTS metadata (key TEXT PRIMARY KEY, value TEXT)')
        self._db.execute(
            f'CREATE INDEX IF NOT EXISTS pending_requests ON requests(request_id) '
            f'WHERE {self.PENDING_FILTER}')
        self._db.commit()
        if not self._db.execute("SELECT 1 FROM metadata WHERE key='compact_storage'").fetchone():
            with self._db:
                for request_id, data in self._db.execute('SELECT request_id, data FROM requests'):
                    record = json.loads(data)
                    if 'response' in record:
                        record['response_digest'] = self._response_digest(record['response'])
                        record['response'] = self._response_metadata(record['response'])
                    if 'feedback_response' in record:
                        record['feedback_response'] = {'duplicate': bool(
                            record['feedback_response'].get('duplicate', False))}
                    self._write(record)
                self._db.execute("INSERT INTO metadata VALUES ('compact_storage', '1')")
            self._db.execute('VACUUM')
        for record in self.pending():
            if record['state'] == 'executing':
                self.update(record['request_id'], state='uncertain',
                            reason='Restart during execution; operator verification required')
            elif record['state'] == 'received':
                self.update(record['request_id'], state='skipped',
                            reason='Restart before execution; cached action will not be replayed')

    def _read(self, request_id):
        row = self._db.execute('SELECT data FROM requests WHERE request_id=?',
                               (request_id,)).fetchone()
        if row is None:
            raise ValueError('Unknown request ID')
        return json.loads(row[0])

    def get(self, request_id):
        with self._lock:
            return self._read(request_id)

    def bind_server(self, url):
        with self._lock:
            row = self._db.execute("SELECT value FROM metadata WHERE key='server_url'").fetchone()
            if row and row[0] != url:
                raise ValueError('This experiment directory belongs to a different server URL')
            with self._db:
                self._db.execute("INSERT OR IGNORE INTO metadata VALUES ('server_url', ?)", (url,))

    def pending(self):
        with self._lock:
            return [json.loads(row[0]) for row in self._db.execute(
                f'SELECT data FROM requests INDEXED BY pending_requests '
                f'WHERE {self.PENDING_FILTER} ORDER BY rowid')]

    def prepare(self, payload, images, observation_time):
        if len(images) != 3:
            raise ValueError('Three encoded image parts are required')
        with self._lock:
            if self.pending():
                raise RuntimeError('Resolve the pending chunk before requesting another')
            request_id = str(uuid.uuid4())
            payload = dict(payload, request_id=request_id, return_dict=True)
            query = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode('utf-8')
            record = dict(request_id=request_id, id=None, state='prepared',
                          observation_time=observation_time, created_at=time.time(),
                          executed_steps=0, feedback=None, feedback_confirmed=False)
            with self._db:
                self._db.execute('INSERT INTO requests VALUES (?, NULL, ?, ?, ?, ?, ?)',
                                 (request_id, json.dumps(record, allow_nan=False), query, *images))
            return request_id

    def wire(self, request_id):
        with self._lock:
            if self._read(request_id)['state'] != 'prepared':
                raise ValueError('Request result is already resolved')
            row = self._db.execute(
                'SELECT query, head, left_image, right_image FROM requests WHERE request_id=?',
                (request_id,)).fetchone()
            return row[0], tuple(row[1:])

    @staticmethod
    def _response_digest(response):
        return hashlib.sha256(json.dumps(
            response, sort_keys=True, allow_nan=False).encode('utf-8')).hexdigest()

    @classmethod
    def _response_metadata(cls, response):
        return {key: response[key] for key in cls.RESPONSE_FIELDS if key in response}

    def _write(self, record):
        clear = (", query=X'', head=X'', left_image=X'', right_image=X''"
                 if record['state'] != 'prepared' else '')
        self._db.execute(f'UPDATE requests SET chunk_id=?, data=?{clear} WHERE request_id=?',
                         (record['id'], json.dumps(record, allow_nan=False), record['request_id']))

    def update(self, request_id, **changes):
        with self._lock:
            record = self._read(request_id)
            record.update(changes)
            with self._db:
                self._write(record)
            return record

    def receive(self, request_id, response):
        with self._lock:
            record = self._read(request_id)
            digest = self._response_digest(response)
            if record['state'] != 'prepared':
                if record.get('response_digest') != digest:
                    raise ValueError('A request returned conflicting responses')
                return record
            chunk_id = response.get('id')
            if not isinstance(chunk_id, str) or not chunk_id:
                raise ValueError('RL response requires a nonempty string id')
            return self.update(request_id, id=chunk_id, response=self._response_metadata(response),
                               response_digest=digest, state='received',
                               received_at=time.time())

    def claim(self, request_id, steps):
        with self._lock:
            if self._read(request_id)['state'] != 'received':
                raise RuntimeError('Chunk already claimed or no longer executable')
            return self.update(request_id, state='executing', planned_steps=steps,
                               execution_started_at=time.time())

    def finish(self, request_id, steps, completed, reason=''):
        with self._lock:
            record = self._read(request_id)
            if record['state'] != 'executing':
                raise RuntimeError('Chunk is not executing')
            if type(steps) is not int or not 0 <= steps <= record['planned_steps']:
                raise ValueError('Invalid model-step count')
            return self.update(request_id, state='completed' if completed else 'interrupted',
                               executed_steps=steps, reason=reason, execution_ended_at=time.time())

    def resolve(self, request_id, steps):
        with self._lock:
            record = self._read(request_id)
            if (record.get('operator_verified') and record['executed_steps'] == steps
                    and type(steps) is int):
                return record
            if record['state'] != 'uncertain':
                raise ValueError('Only uncertain executions require resolution')
            if type(steps) is not int or not 0 <= steps <= record['planned_steps']:
                raise ValueError('Verify and enter the actual model-step count')
            return self.update(request_id, state='interrupted', executed_steps=steps,
                               operator_verified=True)

    def queue_feedback(self, request_id, reward):
        if reward is not None and (isinstance(reward, bool) or not isinstance(reward, (int, float))
                                   or not math.isfinite(reward) or reward not in (-1, -.5, 0, .5, 1)):
            raise ValueError('Invalid reward')
        with self._lock:
            record = self._read(request_id)
            feedback = dict(id=record['id'], reward=reward,
                            executed_steps=record['executed_steps'])
            if record['feedback'] is not None:
                if feedback != record['feedback']:
                    raise ValueError('Feedback is immutable after submission')
                return record
            if record['state'] not in ('completed', 'interrupted', 'skipped'):
                raise ValueError('Execution result must be resolved before scoring')
            if reward is not None and (record['state'] != 'completed' or not record['executed_steps']):
                raise ValueError('Use /null for skipped or interrupted chunks')
            return self.update(request_id, feedback=feedback, state='feedback_pending')

    def confirm(self, request_id, response):
        with self._lock:
            if self._read(request_id)['state'] != 'feedback_pending':
                raise ValueError('No submitted feedback to confirm')
            return self.update(request_id, state='confirmed', feedback_confirmed=True,
                               feedback_response={'duplicate': bool(response.get('duplicate', False))},
                               confirmed_at=time.time())

    def close(self):
        with self._lock:
            self._db.close()
            self._file_lock.close()
