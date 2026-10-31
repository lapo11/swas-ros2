"""Bounded publisher-identity resolution for the vision-only manager.
No ROS, process signalling, authentication or motion authority in this module.
"""
import math

UNKNOWN = (None, '', '_NODE_NAME_UNKNOWN_', '_NODE_NAMESPACE_UNKNOWN_')

class PublisherIdentityWindow:
    def __init__(self, timeout_s=5.0):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('Positive finite timeout required')
        self.timeout_s = timeout_s
        self.first_unresolved_at = None

    def evaluate(self, publishers, expected_name, expected_namespace, now):
        topics = {}
        unknown = []
        missing = []
        conflicts = []
        if not math.isfinite(now):
            return {'state':'CONFLICT','reason':'Invalid monotonic evaluation time','topics':{}}
        for topic, endpoints in publishers.items():
            count = len(endpoints)
            if count > 1:
                topics[topic] = {'count':count,'identity':'CONFLICT'}
                conflicts.append(topic+': multiple publishers')
                continue
            if count == 0:
                topics[topic] = {'count':0,'identity':'MISSING'}
                missing.append(topic)
                continue
            endpoint = endpoints[0]
            name = getattr(endpoint, 'node_name', None)
            namespace = getattr(endpoint, 'node_namespace', None)
            unknown_name = name in UNKNOWN
            unknown_namespace = namespace in UNKNOWN
            # A known wrong field is a conflict even when the other field is unresolved.
            if ((not unknown_name and name != expected_name) or
                    (not unknown_namespace and namespace != expected_namespace)):
                topics[topic] = {'count':1,'identity':'CONFLICT','node_name':name,'node_namespace':namespace}
                conflicts.append(topic+': resolved publisher identity differs')
            elif unknown_name or unknown_namespace:
                topics[topic] = {'count':1,'identity':'UNRESOLVED','node_name':name,'node_namespace':namespace}
                unknown.append(topic)
            else:
                topics[topic] = {'count':1,'identity':'MATCHED','node_name':name,'node_namespace':namespace}
        if conflicts:
            return {'state':'CONFLICT','reason':'; '.join(conflicts),'topics':topics}
        if unknown and self.first_unresolved_at is None:
            self.first_unresolved_at = now
        if self.first_unresolved_at is not None and (unknown or missing):
            elapsed = now-self.first_unresolved_at
            if elapsed < 0:
                return {'state':'IDENTITY_TIMEOUT','reason':'Monotonic evaluation time moved backwards','topics':topics}
            if elapsed >= self.timeout_s:
                return {'state':'IDENTITY_TIMEOUT','reason':'Publisher identity did not fully resolve within '+str(self.timeout_s)+' seconds','topics':topics,'elapsed_s':elapsed}
            return {'state':'PENDING','reason':'Waiting for publisher identity resolution; no readiness granted','topics':topics,'elapsed_s':elapsed,'timeout_s':self.timeout_s}
        if unknown:
            return {'state':'PENDING','reason':'Waiting for publisher identity resolution','topics':topics}
        if missing or not publishers:
            return {'state':'MISSING','reason':'Waiting for both marker output publishers','topics':topics}
        self.first_unresolved_at = None
        return {'state':'MATCHED','reason':'Exactly one expected publisher per marker output topic','topics':topics}
