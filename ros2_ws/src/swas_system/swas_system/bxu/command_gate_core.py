import math

class GateCore:
    """Simulation-only arbitration logic. No ROS/time dependency; caller supplies monotonic time."""
    def __init__(self):
        self.mode = 'STOP'
        self.reason = 'Initial stop'
        self.samples = {}
        self.vectors = {}
        self.clock_value = None
        self.clock_changed_at = None

    def observe(self, key, value, now):
        self.samples[key] = (value, now)
        if key == 'clock' and value != self.clock_value:
            self.clock_value = value
            self.clock_changed_at = now

    def get(self, key, now, age=1.2):
        value, at = self.samples.get(key, (None, -math.inf))
        return value if now - at <= age else None

    def clock_ok(self, now):
        return (self.get('clock', now, 0.6) is not None and
                self.clock_changed_at is not None and now-self.clock_changed_at <= 0.6)

    def stop(self, reason='Stop requested'):
        self.mode = 'STOP'
        self.reason = reason
        self.vectors.clear()

    def select(self, mode, now):
        if mode == 'STOP':
            self.stop()
            return True, 'Command gate stopped. This does not cancel lift actuation.'
        if mode not in ('AUTO', 'MANUAL', 'OPERATION'):
            return False, 'Unsupported mode'
        if not self.clock_ok(now):
            return False, 'An advancing simulation clock is required'
        if mode == 'OPERATION':
            operation = self.get('operation', now, 1.0)
            if not isinstance(operation, dict) or operation.get('state') != 'ACTIVE':
                return False, 'Marker-approach operation must be ACTIVE and recent'
            if operation.get('motion_authorized') is not False:
                return False, 'Marker-approach operation reports unexpected motion authorization'
            lift = self.get('lift', now, 2.0)
            if lift not in ('IDLE', 'LIFTED', 'LOWERED'):
                return False, 'Lift state unavailable or in motion; operation authority refused'
            self.mode, self.reason = mode, 'Marker-approach operation authorized'
            self.vectors.clear()
            return True, 'Marker-approach operation authorized. Fresh commands are required.'
        if self.get('mission', now) != 'WAITING':
            return False, 'Supervisor must report a recent WAITING state'
        if self.get('front', now) is not False or self.get('dock', now) is not False:
            return False, 'Both controller-enable reports must be recent and false'
        lift = self.get('lift', now, 2.0)
        if lift not in ('IDLE', 'LIFTED', 'LOWERED'):
            return False, 'Lift state unavailable or in motion; manual/auto authority refused'
        self.mode, self.reason = mode, 'Explicit authority selection'
        self.vectors.clear()
        return True, mode + ' selected. Fresh commands are required.'

    def command(self, source, values, now):
        if len(values) != 3 or not all(math.isfinite(v) for v in values):
            self.stop('Invalid velocity input')
            return
        self.vectors[source] = (tuple(values), now)

    def output(self, now):
        if self.mode == 'STOP':
            return (0.0, 0.0, 0.0)
        if not self.clock_ok(now):
            self.stop('Simulation clock freshness lost')
            return (0.0, 0.0, 0.0)
        if self.mode == 'OPERATION':
            operation = self.get('operation', now, 1.0)
            if not isinstance(operation, dict) or operation.get('state') != 'ACTIVE':
                self.stop('Marker-approach operation authority lost')
                return (0.0, 0.0, 0.0)
            source = 'auto'
        elif self.mode == 'MANUAL':
            if (self.get('mission', now) != 'WAITING' or self.get('front', now) is not False or
                    self.get('dock', now) is not False or self.get('lift', now, 2.0) not in ('IDLE', 'LIFTED', 'LOWERED')):
                self.stop('Manual authority interlock lost')
                return (0.0, 0.0, 0.0)
            source = 'manual'
        else:
            if self.get('mission', now) is None:
                self.stop('Supervisor freshness lost')
                return (0.0, 0.0, 0.0)
            source = 'auto'
        vector, at = self.vectors.get(source, ((0.0, 0.0, 0.0), -math.inf))
        if now-at > 0.35:
            # Never fall through from Manual to Auto on timeout.
            return (0.0, 0.0, 0.0)
        # OPERATION uses the conservative manual limits; it is an explicitly
        # authorized, bounded simulation operation rather than a supervisor demo.
        limits = (0.20, 0.20, 0.15)
        return tuple(max(-limit, min(value, limit)) for value, limit in zip(vector, limits))
