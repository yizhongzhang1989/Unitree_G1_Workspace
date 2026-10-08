"""CogACT RL transport; retries use the journal's original multipart parts."""

import math
import time

import requests

from g1_vla_bridge.backends.cogact_unitree import IMAGE_PARTS, build_payload, encode_images, parse_action


class RetryRequest(RuntimeError):
    pass


class ApiError(RuntimeError):
    def __init__(self, status, code):
        self.status = status
        self.code = code
        super().__init__(f'RL HTTP {status}: {code}')


class Transport:
    def __init__(self, backend, journal):
        self.backend = backend
        self.journal = journal
        self.journal.bind_server(backend.url)
        self.http = requests.Session()
        self.http.proxies.update(backend._session.proxies)

    def _request(self, method, endpoint, **kwargs):
        try:
            response = self.http.request(method, self.backend._endpoint(endpoint),
                                         timeout=self.backend.timeout, allow_redirects=False, **kwargs)
        except (requests.Timeout, requests.ConnectionError):
            raise RetryRequest('RL connection interrupted; original request retained') from None
        try:
            body = response.json()
        except ValueError:
            if response.status_code >= 500:
                raise RetryRequest('RL server unavailable') from None
            raise ApiError(response.status_code, 'invalid_json') from None
        if not isinstance(body, dict):
            raise ApiError(response.status_code, 'invalid_body')
        if response.status_code != 200:
            code = body.get('code', body.get('error', 'unknown'))
            if isinstance(code, dict):
                code = code.get('code', 'unknown')
            if code not in ('pending', 'conflict', 'updating', 'stale_observation',
                            'pending_limit', 'expired'):
                code = 'unknown'
            if response.status_code == 409 and code == 'pending':
                raise RetryRequest('Original inference is pending')
            raise ApiError(response.status_code, code)
        return body

    def status(self):
        body = self._request('GET', 'adaptation/status')
        if body.get('phase') not in ('collecting', 'updating', 'error'):
            raise ValueError('Unrecognized adaptation phase; check the server API')
        return body

    def prepare(self, observation):
        images = encode_images(observation, self.backend.spec)
        payload = build_payload(observation, self.backend._frame, self.backend.history_length)
        acquired = observation.acquired_monotonic
        if acquired is None or not math.isfinite(acquired) or acquired > time.monotonic():
            raise ValueError('RL requires a valid observation acquisition time')
        observed_at = time.time() - (time.monotonic() - acquired)
        return self.journal.prepare(payload, images, observed_at)

    def infer(self, request_id):
        query, images = self.journal.wire(request_id)
        files = [(part, (filename, image, 'image/jpeg'))
                 for (part, filename), image in zip(IMAGE_PARTS, images)]
        files.append(('json', ('query.json', query, 'application/json')))
        body = self._request('POST', 'inference', files=files)
        self.journal.receive(request_id, body)
        for key in ('policy_version', 'batch_id', 'execution_chunk_size'):
            if key not in body:
                raise ValueError(f'RL response requires {key}')
        chunk = self.backend._to_chunk(parse_action(body))
        count = body['execution_chunk_size']
        if type(count) is not int or not 1 <= count <= chunk.horizon:
            raise ValueError('Invalid execution_chunk_size; check the server API')
        return chunk, body

    def feedback(self, request_id):
        record = self.journal.get(request_id)
        if record['state'] != 'feedback_pending':
            raise ValueError('Feedback must be journaled before transmission')
        body = self._request('POST', 'feedback', json=record['feedback'])
        if body.get('id', record['id']) != record['id']:
            raise ValueError('Feedback acknowledgement ID mismatch')
        if body.get('success') is False or body.get('error'):
            raise ValueError('Feedback was not acknowledged')
        self.journal.confirm(request_id, body)

    def close(self):
        self.http.close()
