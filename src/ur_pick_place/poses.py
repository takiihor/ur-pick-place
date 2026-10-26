"""Named RTDE snapshots stored atomically as portable JSON."""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import tempfile
import threading
import uuid

from .config import vector


def pose_name(value):
    if not isinstance(value, str) or not value.strip() or len(value.strip()) > 80:
        raise ValueError('Pose name must contain 1–80 characters')
    return value.strip()


class PoseStore:
    def __init__(self, path):
        self.path = Path(path)
        self._lock = threading.RLock()
        self._poses = []
        if self.path.exists():
            raw = json.loads(self.path.read_text())
            if not isinstance(raw, dict) or raw.get('version') != 1 or not isinstance(raw.get('poses'), list):
                raise ValueError('Unsupported pose store format')
            ids, names = set(), set()
            for pose in raw['poses']:
                if not isinstance(pose, dict):
                    raise ValueError('Invalid stored pose')
                name = pose_name(pose.get('name'))
                ident = pose.get('id')
                if not isinstance(ident, str) or not ident or ident in ids or name.casefold() in names:
                    raise ValueError('Duplicate or invalid stored pose ID/name')
                vector(pose.get('tcp_pose_m_rad'), 6, name + ' TCP')
                vector(pose.get('joints_rad'), 6, name + ' joints')
                ids.add(ident)
                names.add(name.casefold())
            self._poses = raw['poses']

    def list(self):
        with self._lock:
            return json.loads(json.dumps(self._poses))

    def get(self, ident):
        with self._lock:
            for pose in self._poses:
                if pose['id'] == ident:
                    return dict(pose, tcp_pose_m_rad=list(pose['tcp_pose_m_rad']),
                                joints_rad=list(pose['joints_rad']))
        raise ValueError('Saved pose not found')

    def by_name(self, name):
        with self._lock:
            for pose in self._poses:
                if pose['name'] == name:
                    return self.get(pose['id'])
        raise ValueError(f'Saved pose not found: {name}')

    def _unique(self, name, excluding=None):
        if any(p['id'] != excluding and p['name'].casefold() == name.casefold() for p in self._poses):
            raise ValueError('A pose with that name already exists')

    def save(self, name, tcp_pose, joints, *, robot_ip, demo=False, ident=None):
        name = pose_name(name)
        pose = dict(id=ident if ident is not None else uuid.uuid4().hex, name=name,
                    tcp_pose_m_rad=vector(tcp_pose, 6, 'TCP pose'),
                    joints_rad=vector(joints, 6, 'Joints'),
                    recorded_at=datetime.now(timezone.utc).isoformat(),
                    robot_ip=robot_ip, demo=bool(demo))
        with self._lock:
            if ident is not None:
                existing = self.get(ident)
                if bool(existing.get('demo')) != bool(demo):
                    raise ValueError('Demo and live poses cannot be mixed')
            self._unique(name, excluding=ident)
            updated = ([pose if p['id'] == ident else p for p in self._poses]
                       if ident is not None else [*self._poses, pose])
            self._write(updated)
            self._poses = updated
        return dict(pose)

    def rename(self, ident, name):
        name = pose_name(name)
        with self._lock:
            self.get(ident)
            self._unique(name, excluding=ident)
            updated = [dict(p, name=name) if p['id'] == ident else p for p in self._poses]
            self._write(updated)
            self._poses = updated
        return self.get(ident)

    def _write(self, updated):
        atomic_json(self.path, dict(version=1, poses=updated))


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(value, indent=2, allow_nan=False) + '\n'
    descriptor, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'w') as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
