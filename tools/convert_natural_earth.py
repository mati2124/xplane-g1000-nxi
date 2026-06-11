#!/usr/bin/env python3
"""Builds the bundled land-data asset from Natural Earth (public domain).

Downloads the 10m-scale GeoJSON layers (rivers, lakes, roads, country
borders, populated places) from the Natural Earth vector repository and packs
the geometry the map actually draws into a compact little-endian binary:

    u32  magic "AVLD"
    u32  version (1)
    u32  line count
    u32  city count
    line records:  u8 class, u16 point count, point count * (f32 lat, f32 lon)
    city records:  f32 lat, f32 lon, u8 rank, u8 name length, name bytes

Line classes match avionics::LandClass: 0 river, 1 lake (closed ring,
filled), 2 road, 3 border (nation), 4 coast, 5 state/province border,
6 railroad. City rank is 0-10 (bigger = more prominent),
derived from Natural Earth's scalerank, and drives range declutter.

Geometry is decimated to ~MIN_POINT_SPACING_DEG so the asset stays small at
map scales; tiny lakes and minor roads are dropped entirely.

Usage:
    python3 tools/convert_natural_earth.py [output.bin]

Output defaults to shell-standalone/assets/land_data.bin. Downloads are
cached in tools/.ne_cache/ so re-runs are offline.
"""

import json
import math
import os
import struct
import sys
import urllib.request

BASE_URL = ("https://raw.githubusercontent.com/nvkelso/"
            "natural-earth-vector/master/geojson/")
CACHE_DIR = os.path.join(os.path.dirname(__file__), ".ne_cache")
DEFAULT_OUTPUT = os.path.join(os.path.dirname(__file__), "..",
                              "shell-standalone", "assets", "land_data.bin")

MAGIC = 0x444C5641  # "AVLD" little-endian
VERSION = 1

CLASS_RIVER = 0
CLASS_LAKE = 1
CLASS_ROAD = 2
CLASS_BORDER = 3
CLASS_COAST = 4
CLASS_STATE = 5
CLASS_RAIL = 6

# Decimation: drop intermediate vertices closer than this to the last kept
# one (~0.6 NM); plenty below the ~1 px resolution at the map's ranges.
MIN_POINT_SPACING_DEG = 0.01
# Lakes with a bounding box smaller than this are invisible at map scales.
MIN_LAKE_EXTENT_DEG = 0.05
MAX_POINTS_PER_LINE = 65535


def fetch(name):
    os.makedirs(CACHE_DIR, exist_ok=True)
    cached = os.path.join(CACHE_DIR, name)
    if not os.path.exists(cached):
        url = BASE_URL + name
        print("downloading", url)
        with urllib.request.urlopen(url) as response, open(cached, "wb") as f:
            f.write(response.read())
    with open(cached, "r", encoding="utf-8") as f:
        return json.load(f)


def decimate(coords):
    """GeoJSON [lon, lat] list -> [(lat, lon)] with min spacing applied."""
    out = []
    for lon, lat in (c[:2] for c in coords):
        if out:
            dlat = lat - out[-1][0]
            dlon = lon - out[-1][1]
            if math.hypot(dlat, dlon) < MIN_POINT_SPACING_DEG:
                continue
        out.append((lat, lon))
    # Always keep the true endpoint so lines do not visibly shrink.
    if coords and len(out) >= 1:
        last = (coords[-1][1], coords[-1][0])
        if out[-1] != last:
            out.append(last)
    return out


def each_linestring(geometry):
    gtype = geometry.get("type")
    if gtype == "LineString":
        yield geometry["coordinates"]
    elif gtype == "MultiLineString":
        yield from geometry["coordinates"]
    elif gtype == "Polygon":
        yield geometry["coordinates"][0]  # exterior ring only
    elif gtype == "MultiPolygon":
        for poly in geometry["coordinates"]:
            yield poly[0]


def add_lines(lines, data, line_class, keep=lambda props: True):
    kept = 0
    for feature in data["features"]:
        props = feature.get("properties") or {}
        geometry = feature.get("geometry") or {}
        if not keep(props):
            continue
        for coords in each_linestring(geometry):
            pts = decimate(coords)
            if len(pts) < 2:
                continue
            if line_class == CLASS_LAKE:
                lats = [p[0] for p in pts]
                lons = [p[1] for p in pts]
                if (max(lats) - min(lats) < MIN_LAKE_EXTENT_DEG and
                        max(lons) - min(lons) < MIN_LAKE_EXTENT_DEG):
                    continue
            for start in range(0, len(pts), MAX_POINTS_PER_LINE):
                chunk = pts[start:start + MAX_POINTS_PER_LINE]
                if len(chunk) >= 2:
                    lines.append((line_class, chunk))
                    kept += 1
    return kept


def main():
    output = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUTPUT

    lines = []

    rivers = fetch("ne_10m_rivers_lake_centerlines.geojson")
    n = add_lines(lines, rivers, CLASS_RIVER)
    print(f"rivers: {n} lines")

    lakes = fetch("ne_10m_lakes.geojson")
    n = add_lines(lines, lakes, CLASS_LAKE)
    print(f"lakes: {n} rings")

    # Major roads only: the G1000 land data shows highways, not every street.
    roads = fetch("ne_10m_roads.geojson")
    n = add_lines(
        lines, roads, CLASS_ROAD,
        keep=lambda p: (p.get("type") in
                        ("Major Highway", "Beltway", "Freeway") or
                        (p.get("scalerank") or 99) <= 5))
    print(f"roads: {n} lines")

    borders = fetch("ne_10m_admin_0_boundary_lines_land.geojson")
    n = add_lines(lines, borders, CLASS_BORDER)
    print(f"borders: {n} lines")

    coast = fetch("ne_10m_coastline.geojson")
    n = add_lines(lines, coast, CLASS_COAST)
    print(f"coastlines: {n} lines")

    states = fetch("ne_10m_admin_1_states_provinces_lines.geojson")
    n = add_lines(lines, states, CLASS_STATE)
    print(f"state/province borders: {n} lines")

    railroads = fetch("ne_10m_railroads.geojson")
    n = add_lines(lines, railroads, CLASS_RAIL)
    print(f"railroads: {n} lines")

    places = fetch("ne_10m_populated_places_simple.geojson")
    cities = []
    for feature in places["features"]:
        props = feature.get("properties") or {}
        geometry = feature.get("geometry") or {}
        if geometry.get("type") != "Point":
            continue
        scalerank = props.get("scalerank")
        if scalerank is None or scalerank > 7:
            continue
        name = (props.get("name") or "").strip().upper()
        if not name:
            continue
        lon, lat = geometry["coordinates"][:2]
        rank = max(0, min(10, 10 - scalerank))
        cities.append((lat, lon, rank, name.encode("utf-8")[:255]))
    print(f"cities: {len(cities)}")

    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, "wb") as f:
        f.write(struct.pack("<IIII", MAGIC, VERSION, len(lines), len(cities)))
        for line_class, pts in lines:
            f.write(struct.pack("<BH", line_class, len(pts)))
            for lat, lon in pts:
                f.write(struct.pack("<ff", lat, lon))
        for lat, lon, rank, name in cities:
            f.write(struct.pack("<ffBB", lat, lon, rank, len(name)))
            f.write(name)

    size_mb = os.path.getsize(output) / (1024 * 1024)
    print(f"wrote {output} ({size_mb:.1f} MB, "
          f"{len(lines)} lines, {len(cities)} cities)")


if __name__ == "__main__":
    main()
