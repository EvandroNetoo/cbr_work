"""Última ação visual recebida, mantida até outra ação relevante chegar."""

import threading


class LiveOperations:
    _IGNORED = {None, '', 'prepare_manipulator'}

    def __init__(self):
        self._lock = threading.RLock()
        self._operation = ''
        # Tokens servem apenas para ignorar feedback tardio, sem restaurar rostos.
        self._pending = {}
        self._active = set()

    def current(self):
        with self._lock:
            return self._operation

    def _select(self, operation):
        if operation in self._IGNORED or operation == self._operation:
            return False
        self._operation = operation
        return True

    def pending(self, operation):
        token = object()
        with self._lock:
            self._pending[token] = operation
        return token

    def dispatched(self, token):
        with self._lock:
            if token not in self._pending:
                return False
            operation = self._pending.pop(token)
            self._active.add(token)
            return self._select(operation)

    def feedback(self, token, operation):
        with self._lock:
            if token in self._pending:
                self._pending.pop(token)
                self._active.add(token)
            if token not in self._active:
                return False
            return self._select(operation)

    def finished(self, token):
        with self._lock:
            self._pending.pop(token, None)
            self._active.discard(token)
        return False  # Terminar uma ação nunca restaura uma carinha anterior.

    def reset(self):
        with self._lock:
            changed = self._operation != ''
            self._operation = ''
            self._pending.clear()
            self._active.clear()
            return changed
