"""Índice regional em disco, preservando as coordenadas inteiras do LAS.

A prévia é somente visual. Uma consulta regional devolve todos os pontos
originais no raio solicitado ou recusa a carga se exceder o limite de memória.
"""
from concurrent.futures import CancelledError
from contextlib import closing
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import tempfile
import zlib

import laspy
import numpy as np


@dataclass
class CloudIndex:
    path: Path
    metadata: dict
    preview: np.ndarray

    def region(self, center, radius, max_points=2_000_000):
        center = np.asarray(center, dtype=float)
        if center.shape != (3,) or not np.isfinite(center).all() or not np.isfinite(radius) or radius <= 0:
            raise ValueError('Centro e raio da região inválidos.')
        size = self.metadata['tile_size']
        lower, upper = np.floor((center-radius)/size), np.floor((center+radius)/size)
        with closing(sqlite3.connect(self.path)) as db:
            tiles = db.execute('SELECT id, count FROM tiles WHERE tx BETWEEN ? AND ? AND ty BETWEEN ? AND ? AND tz BETWEEN ? AND ?',
                               (int(lower[0]), int(upper[0]), int(lower[1]), int(upper[1]), int(lower[2]), int(upper[2]))).fetchall()
            if sum(count for _, count in tiles) > max_points:
                raise ValueError('Região muito densa. Aumente o limite de carga regional ou reduza o raio para carregar todos os pontos sem amostrar.')
            scale = np.asarray(self.metadata['scales'])
            shift = np.asarray(self.metadata['offsets']) - self.metadata['origin']
            partes = []
            for tile, _ in tiles:
                for (data,) in db.execute('SELECT data FROM parts WHERE tile = ?', (tile,)):
                    raw = np.frombuffer(zlib.decompress(data), dtype='<i4').reshape(-1, 3)
                    points = raw * scale + shift
                    points = points[np.sum((points-center)**2, axis=1) <= radius**2]
                    if len(points):
                        partes.append(points)
        return np.concatenate(partes) if partes else np.empty((0, 3))


def build_index(source, cache_dir, tile_size=50., preview_limit=300_000,
                chunk_size=500_000, notify=None, cancelled=None):
    source = Path(source).resolve()
    stat = source.stat()
    if tile_size <= 0 or not np.isfinite(tile_size) or preview_limit < 1 or chunk_size < 1:
        raise ValueError('Parâmetros de indexação inválidos.')
    signature = f'1|{source}|{stat.st_size}|{stat.st_mtime_ns}|{tile_size}|{preview_limit}'
    digest = hashlib.sha256(signature.encode()).hexdigest()[:24]
    cache_dir = Path(cache_dir)
    cache_dir.mkdir(parents=True, exist_ok=True)
    target = cache_dir/f'{digest}.sqlite'
    if target.is_file():
        with closing(sqlite3.connect(target)) as db:
            metadata = json.loads(db.execute('SELECT value FROM metadata').fetchone()[0])
            preview = np.frombuffer(zlib.decompress(db.execute('SELECT data FROM preview').fetchone()[0]), dtype='<f8').reshape(-1, 3).copy()
        return CloudIndex(target, metadata, preview)
    notify = notify or (lambda text: None)
    with tempfile.NamedTemporaryFile(dir=cache_dir, suffix='.building', delete=False) as temp:
        temporary = Path(temp.name)
    try:
        with laspy.open(source) as reader, closing(sqlite3.connect(temporary)) as db, db:
            header = reader.header
            if header.point_count == 0:
                raise ValueError('A nuvem não contém pontos.')
            origin = header.mins.copy()
            scale, offset = header.scales.copy(), header.offsets.copy()
            if np.any(scale <= 0) or not np.isfinite([origin, scale, offset]).all():
                raise ValueError('Metadados de coordenadas inválidos.')
            dimensions = np.floor((header.maxs-origin)/tile_size).astype(np.int64)+1
            stride = max(1, math.ceil(header.point_count/preview_limit))
            db.executescript('''
                CREATE TABLE parts (tile INTEGER, data BLOB);
                CREATE TABLE tiles (id INTEGER PRIMARY KEY, tx INTEGER, ty INTEGER, tz INTEGER, count INTEGER);
                CREATE TABLE metadata (value TEXT);
                CREATE TABLE preview (data BLOB);
            ''')
            counts, preview_parts, processed = {}, [], 0
            for chunk in reader.chunk_iterator(chunk_size):
                if cancelled is not None and cancelled.is_set():
                    raise CancelledError('Indexação cancelada.')
                raw = np.column_stack((chunk.X, chunk.Y, chunk.Z)).astype('<i4', copy=False)
                points = raw*scale+(offset-origin)
                cells = np.clip(np.floor(points/tile_size).astype(np.int64), 0, dimensions-1)
                codes = cells[:, 0]+dimensions[0]*(cells[:, 1]+dimensions[1]*cells[:, 2])
                order = np.argsort(codes, kind='stable')
                sorted_codes = codes[order]
                boundaries = np.r_[0, np.flatnonzero(np.diff(sorted_codes))+1, len(raw)]
                for first, last in zip(boundaries[:-1], boundaries[1:]):
                    code = int(sorted_codes[first])
                    positions = order[first:last]
                    db.execute('INSERT INTO parts VALUES (?, ?)',
                               (code, zlib.compress(raw[positions].tobytes(), level=1)))
                    if code not in counts:
                        counts[code] = [*map(int, cells[positions[0]]), 0]
                    counts[code][3] += len(positions)
                start = (-processed) % stride
                preview_parts.append(points[start::stride])
                processed += len(raw)
                notify(f'Indexando nuvem original: {processed:,} / {header.point_count:,} pontos')
            preview = np.concatenate(preview_parts)
            metadata = dict(source=str(source), count=int(header.point_count),
                            origin=origin.tolist(), scales=scale.tolist(), offsets=offset.tolist(),
                            max_relative=(header.maxs-origin).tolist(), tile_size=float(tile_size),
                            source_size=stat.st_size, source_mtime_ns=stat.st_mtime_ns)
            db.executemany('INSERT INTO tiles VALUES (?, ?, ?, ?, ?)', [(key, *values) for key, values in counts.items()])
            db.execute('CREATE INDEX parts_tile ON parts(tile)')
            db.execute('INSERT INTO metadata VALUES (?)', (json.dumps(metadata),))
            db.execute('INSERT INTO preview VALUES (?)', (zlib.compress(preview.astype('<f8').tobytes(), level=1),))
        # O cache só fica disponível após a transação e o fechamento do SQLite.
        temporary.replace(target)
        return CloudIndex(target, metadata, preview)
    finally:
        if temporary.exists():
            temporary.unlink()
