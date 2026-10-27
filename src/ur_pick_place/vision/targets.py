"""Pick/place target definitions: standalone ArUco markers and ChArUco boards.

Target frames follow charuco-generator: X right, Y down, Z into the printed surface.
A standalone marker's origin is its centre; a board's origin is its outer top-left
pattern corner.
"""
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from ..config import number
from .errors import VisionError


def canonical_hash(value):
    """Identical to charuco_generator.board.canonical_hash (configuration checksums)."""
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


@dataclass(frozen=True)
class ArucoTarget:
    marker_id: int
    dictionary: str
    marker_length_m: float
    border_bits: int = 1
    source: str = 'config'
    kind: str = field(default='aruco', init=False)

    @property
    def centre(self):
        return np.zeros(3)

    @property
    def label(self):
        return f'ArUco {self.dictionary} id {self.marker_id} ({self.marker_length_m * 1000:g} mm)'

    def describe(self):
        return {'kind': 'aruco', 'marker_id': self.marker_id, 'dictionary': self.dictionary,
                'marker_length_mm': self.marker_length_m * 1000, 'border_bits': self.border_bits,
                'frame': 'marker centre; X right, Y down, Z into marker'}


@dataclass(frozen=True)
class CharucoTarget:
    board_name: str
    board: object = field(compare=False, repr=False)
    record: dict = field(compare=False, repr=False)
    source: str = ''
    kind: str = field(default='charuco', init=False)

    @property
    def border_bits(self):
        return int(self.record['geometry']['border_bits'])

    @property
    def dictionary(self):
        return self.record['dictionary']

    @property
    def centre(self):
        width, height = self.record['pattern_dimensions_m']
        return np.array([width / 2, height / 2, 0.0])

    @property
    def label(self):
        return f'ChArUco board {self.board_name!r} ({self.dictionary}, IDs {self.record["marker_ids"]})'

    def describe(self):
        return {'kind': 'charuco', 'board': self.board_name, 'identifier': self.record.get('identifier'),
                'dictionary': self.dictionary, 'marker_ids': list(self.record['marker_ids']),
                'square_length_mm': self.record['square_length_mm'],
                'marker_length_mm': self.record['marker_length_mm'],
                'chessboard_squares': list(self.record['chessboard_squares']),
                'legacy_pattern': self.record['geometry']['legacy_pattern'],
                'border_bits': self.border_bits,
                'frame': 'outer top-left pattern corner; X right, Y down, Z into board'}


def _read_json(path, what):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except (OSError, ValueError) as exc:
        raise VisionError(f'Cannot read {what} {path}: {exc}') from exc


def load_aruco_manifest(path):
    """Read a charuco-generator ``aruco_manifest.json`` and verify its configuration checksum."""
    manifest = _read_json(path, 'ArUco manifest')
    if not isinstance(manifest, dict) or manifest.get('kind') != 'aruco' or manifest.get('schema_version') != 1:
        raise VisionError(f'{path} is not a schema-1 charuco-generator ArUco manifest')
    configuration = manifest.get('configuration')
    if not isinstance(configuration, dict) or canonical_hash(configuration) != manifest.get('configuration_sha256'):
        raise VisionError(f'ArUco manifest {path} configuration checksum mismatch; regenerate it')
    return manifest


def aruco_target(raw, base_dir):
    allowed = {'kind', 'marker_id', 'aruco_manifest', 'dictionary', 'marker_length_mm', 'border_bits'}
    unknown = set(raw) - allowed
    if unknown:
        raise ValueError(f'Unknown ArUco target settings: {sorted(unknown)}')
    marker_id = raw.get('marker_id')
    if type(marker_id) is not int or marker_id < 0:
        raise ValueError('ArUco target marker_id must be a non-negative integer')
    explicit = {key: raw[key] for key in ('dictionary', 'marker_length_mm', 'border_bits') if key in raw}
    source = 'config'
    if raw.get('aruco_manifest'):
        path = Path(base_dir) / raw['aruco_manifest']
        manifest = load_aruco_manifest(path)
        if marker_id not in manifest.get('marker_ids', []):
            raise ValueError(f'Marker {marker_id} is not in ArUco manifest {path}')
        configuration = manifest['configuration']
        saved = {'dictionary': configuration.get('dictionary'),
                 'marker_length_mm': configuration.get('marker_length_mm'),
                 'border_bits': configuration.get('border_bits', 1)}
        for key, value in explicit.items():
            if value != saved[key]:
                raise ValueError(f'ArUco target {key}={value!r} contradicts manifest value {saved[key]!r}')
        explicit, source = saved, str(path)
    if not isinstance(explicit.get('dictionary'), str) or not explicit['dictionary'].startswith('DICT_'):
        raise ValueError('ArUco target requires dictionary (e.g. DICT_5X5_100) or aruco_manifest')
    length = number(explicit.get('marker_length_mm'), 'marker_length_mm', positive=True, high=1000)
    border = explicit.get('border_bits', 1)
    if type(border) is not int or not 1 <= border <= 8:
        raise ValueError('border_bits must be an integer in 1..8')
    return ArucoTarget(marker_id=marker_id, dictionary=explicit['dictionary'],
                       marker_length_m=length / 1000.0, border_bits=border, source=source)


def charuco_target(raw, base_dir):
    allowed = {'kind', 'board', 'board_manifest'}
    unknown = set(raw) - allowed
    if unknown:
        raise ValueError(f'Unknown ChArUco target settings: {sorted(unknown)}')
    name, manifest_path = raw.get('board'), raw.get('board_manifest')
    if not isinstance(name, str) or not name or not isinstance(manifest_path, str) or not manifest_path:
        raise ValueError('ChArUco target requires board and board_manifest')
    path = Path(base_dir) / manifest_path
    try:
        from charuco_generator.board import reconstruct_board
    except ImportError as exc:
        raise VisionError('ChArUco targets need charuco-generator: '
                          'pip install -e ../charuco-generator --no-deps') from exc
    try:
        from charuco_generator.validation import load_manifest
        manifest = load_manifest(path)
        board = reconstruct_board(manifest, name)
    except VisionError:
        raise
    except Exception as exc:  # ConfigError, KeyError from malformed manifests
        raise VisionError(f'Cannot reconstruct board {name!r} from {path}: {exc}') from exc
    record = next(b for b in manifest['boards'] if b['name'] == name)
    return CharucoTarget(board_name=name, board=board, record=record, source=str(path))


def target_from_config(raw, base_dir='.'):
    if not isinstance(raw, dict):
        raise ValueError('target must be a JSON object')
    kind = raw.get('kind')
    if kind == 'aruco':
        return aruco_target(raw, base_dir)
    if kind == 'charuco':
        return charuco_target(raw, base_dir)
    raise ValueError('target.kind must be aruco or charuco')


def same_target(description, target):
    """Compare a stored target description with a configured target."""
    if not isinstance(description, dict):
        return False
    current = target.describe()
    keys = (('kind', 'marker_id', 'dictionary') if target.kind == 'aruco'
            else ('kind', 'board', 'dictionary', 'marker_ids'))
    if any(description.get(k) != current.get(k) for k in keys):
        return False
    if target.kind == 'aruco':
        return (description.get('border_bits') == current['border_bits']
                and math.isclose(description.get('marker_length_mm', -1), current['marker_length_mm'], abs_tol=1e-6))
    return all(description.get(k) == current[k] for k in ('square_length_mm', 'marker_length_mm',
               'chessboard_squares', 'legacy_pattern', 'border_bits'))
