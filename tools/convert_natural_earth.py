#!/usr/bin/env python3
"""Builds the bundled land-data asset from Natural Earth (public domain).

Downloads Natural Earth GeoJSON (rivers, lakes, roads, borders, labels) and
GSHHG high-resolution shorelines (landmass + coast) from the GMT GitHub
release, then packs the geometry the map draws into a compact little-endian
binary:

    u32  magic "AVLD"
    u32  version (2)
    u32  line count
    u32  label count
    line records:  u8 class, u16 point count, point count * (f32 lat, f32 lon)
    label records (v2): f32 lat, f32 lon, u8 rank, u8 kind, u8 name length,
                        name bytes

Line classes match avionics::LandClass: 0 river, 1 lake (closed ring,
filled), 2 road, 3 border (nation), 4 coast, 5 state/province border,
6 railroad, 7 landmass (continent fill). City rank is 0-10 (bigger = more prominent),
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
import zipfile

try:
    import shapefile  # pyshp — needed for GSHHG high-res shorelines
except ImportError:
    shapefile = None

BASE_URL = ("https://raw.githubusercontent.com/nvkelso/"
            "natural-earth-vector/master/geojson/")
GSHHG_ZIP_URL = ("https://github.com/GenericMappingTools/gshhg-gmt/"
                 "releases/download/2.3.7/gshhg-shp-2.3.7.zip")
GSHHG_ZIP_NAME = "gshhg-shp-2.3.7.zip"
GSHHG_COAST_REL = os.path.join("GSHHS_shp", "h", "GSHHS_h_L1.shp")
GSHHG_LAND_REL = os.path.join("GSHHS_shp", "l", "GSHHS_l_L1.shp")
CACHE_DIR = os.path.join(os.path.dirname(__file__), ".ne_cache")
DEFAULT_OUTPUT = os.path.join(os.path.dirname(__file__), "..",
                              "shell-standalone", "assets", "land_data.bin")

MAGIC = 0x444C5641  # "AVLD" little-endian
VERSION = 2

CLASS_RIVER = 0
CLASS_LAKE = 1
CLASS_ROAD = 2
CLASS_BORDER = 3
CLASS_COAST = 4
CLASS_STATE = 5
CLASS_RAIL = 6
CLASS_LAND = 7

KIND_CITY = 0
KIND_HYDRO = 1
KIND_REGION = 2

# Decimation: drop intermediate vertices closer than this to the last kept
# one (~0.6 NM). Fine for rivers/roads; landmass and coast use tighter spacing.
MIN_POINT_SPACING_DEG = 0.01
# Land/water edge: GSHHG high (~200 m) keeps mangrove inlets and islands
# visible at 10–15 NM; Natural Earth 10 m is too coarse for close range.
MIN_GSHHG_SPACING_DEG = 0.00012
# Drop GSHHG slivers smaller than this bbox (deg); keeps the asset bounded.
MIN_GSHHG_EXTENT_DEG = 0.0008
# Lakes with a bounding box smaller than this are invisible at map scales.
MIN_LAKE_EXTENT_DEG = 0.05
MAX_POINTS_PER_LINE = 65535
LAND_DECIMATE_SPACING_DEG = 0.005
# Oversized high-res rings are split into closed lon bands so fills stay safe.
DETAIL_LON_BAND_DEG = 10.0
DETAIL_BAND_OVERLAP_DEG = 0.15
# Close-range land fill uses coarser spacing than coast strokes (~700 m vs ~200 m)
# so 10–15 NM charts stay smooth without the vertex load of full GSHHG high.
DETAIL_LAND_SPACING_DEG = 0.0004
DETAIL_BAND_SPACING_DEG = 0.0015


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


def decimate(coords, min_spacing=MIN_POINT_SPACING_DEG):
    """GeoJSON [lon, lat] list -> [(lat, lon)] with min spacing applied."""
    out = []
    for lon, lat in (c[:2] for c in coords):
        if out:
            dlat = lat - out[-1][0]
            dlon = lon - out[-1][1]
            if math.hypot(dlat, dlon) < min_spacing:
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


def append_line_chunks(lines, line_class, pts):
    """Pack a vertex list into one or more asset records."""
    if len(pts) < 2:
        return 0
    kept = 0
    for start in range(0, len(pts), MAX_POINTS_PER_LINE):
        chunk = pts[start:start + MAX_POINTS_PER_LINE]
        if len(chunk) >= 2:
            lines.append((line_class, chunk))
            kept += 1
    return kept


def decimate_latlon(pts, min_spacing=MIN_POINT_SPACING_DEG):
    """[(lat, lon), ...] with minimum spacing."""
    out = []
    for lat, lon in pts:
        if out:
            dlat = lat - out[-1][0]
            dlon = lon - out[-1][1]
            if math.hypot(dlat, dlon) < min_spacing:
                continue
        out.append((lat, lon))
    if pts and len(out) >= 1 and out[-1] != pts[-1]:
        out.append(pts[-1])
    return out


def clip_ring_against_edge(pts, edge, vertical, keep_greater):
    """Sutherland-Hodgman clip of a closed ring against one edge."""
    if not pts:
        return []
    out = []
    prev = pts[-1]
    prev_inside = (
        (prev[0] >= edge if vertical else prev[1] >= edge)
        if keep_greater else
        (prev[0] <= edge if vertical else prev[1] <= edge))
    for curr in pts:
        curr_inside = (
            (curr[0] >= edge if vertical else curr[1] >= edge)
            if keep_greater else
            (curr[0] <= edge if vertical else curr[1] <= edge))
        if curr_inside:
            if not prev_inside:
                if vertical:
                    denom = curr[0] - prev[0]
                    if abs(denom) > 1e-9:
                        t = (edge - prev[0]) / denom
                        out.append((edge, prev[1] + t * (curr[1] - prev[1])))
                else:
                    denom = curr[1] - prev[1]
                    if abs(denom) > 1e-9:
                        t = (edge - prev[1]) / denom
                        out.append((prev[0] + t * (curr[0] - prev[0]), edge))
            out.append(curr)
        elif prev_inside:
            if vertical:
                denom = curr[0] - prev[0]
                if abs(denom) > 1e-9:
                    t = (edge - prev[0]) / denom
                    out.append((edge, prev[1] + t * (curr[1] - prev[1])))
            else:
                denom = curr[1] - prev[1]
                if abs(denom) > 1e-9:
                    t = (edge - prev[1]) / denom
                    out.append((prev[0] + t * (curr[0] - prev[0]), edge))
        prev = curr
        prev_inside = curr_inside
    return out


def clip_polygon_to_box(pts, lat_min, lat_max, lon_min, lon_max):
    """Clip a closed (lat, lon) ring to a geographic box."""
    stage = pts
    stage = clip_ring_against_edge(stage, lat_min, True, True)
    if not stage:
        return []
    stage = clip_ring_against_edge(stage, lat_max, True, False)
    if not stage:
        return []
    stage = clip_ring_against_edge(stage, lon_min, False, True)
    if not stage:
        return []
    stage = clip_ring_against_edge(stage, lon_max, False, False)
    return stage


def emit_landmass_rings(lines, ring):
    """Crude GSHHG ring -> fill-safe landmass record(s)."""
    pts = decimate(ring, LAND_DECIMATE_SPACING_DEG)
    if len(pts) < 3:
        return 0
    return append_line_chunks(lines, CLASS_LAND, pts)


def decimate_latlon(pts, min_spacing):
    """Decimate a [(lat, lon), ...] ring."""
    coords = [(p[1], p[0]) for p in pts]
    return decimate(coords, min_spacing)


def emit_lon_band_fills(lines, pts):
    """Split an oversized closed ring into closed lon-band landmass polygons."""
    lons = [p[1] for p in pts]
    lon_min, lon_max = min(lons), max(lons)
    if lon_max - lon_min < DETAIL_LON_BAND_DEG * 1.5:
        return 0
    kept = 0
    band_start = lon_min
    while band_start < lon_max:
        band_end = band_start + DETAIL_LON_BAND_DEG
        clipped = clip_polygon_to_box(
            pts, -90.0, 90.0,
            band_start - DETAIL_BAND_OVERLAP_DEG,
            band_end + DETAIL_BAND_OVERLAP_DEG)
        if len(clipped) >= 3:
            if len(clipped) > MAX_POINTS_PER_LINE:
                clipped = decimate_latlon(clipped, DETAIL_BAND_SPACING_DEG)
            if 3 <= len(clipped) <= MAX_POINTS_PER_LINE:
                lines.append((CLASS_LAND, clipped))
                kept += 1
        band_start = band_end
    return kept


def emit_detail_landmass(lines, ring):
    """High-res GSHHG ring -> one or more closed fill records."""
    pts = decimate(ring, DETAIL_LAND_SPACING_DEG)
    if len(pts) < 3:
        return 0
    if len(pts) <= MAX_POINTS_PER_LINE:
        lines.append((CLASS_LAND, pts))
        return 1
    return emit_lon_band_fills(lines, pts)


def gshhg_path(rel):
    path = os.path.join(CACHE_DIR, rel)
    if os.path.exists(path):
        return path
    if shapefile is None:
        raise RuntimeError(
            "pyshp is required for GSHHG shorelines: "
            "python3 -m pip install --user pyshp")
    os.makedirs(CACHE_DIR, exist_ok=True)
    cached_zip = os.path.join(CACHE_DIR, GSHHG_ZIP_NAME)
    if not os.path.exists(cached_zip):
        print("downloading", GSHHG_ZIP_URL)
        with urllib.request.urlopen(GSHHG_ZIP_URL) as response, open(
                cached_zip, "wb") as f:
            f.write(response.read())
    print("extracting", GSHHG_ZIP_NAME)
    with zipfile.ZipFile(cached_zip) as zf:
        zf.extractall(CACHE_DIR)
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    return path


def gshhg_l1_path():
    return gshhg_path(GSHHG_COAST_REL)


def iter_shape_rings(shape):
    """Yield exterior rings from a pyshp polygon (lon, lat) vertices."""
    if not shape.points:
        return
    parts = list(shape.parts) + [len(shape.points)]
    for i, start in enumerate(shape.parts):
        ring = shape.points[start:parts[i + 1]]
        if len(ring) >= 2:
            yield ring


def add_gshhg_landmass(lines):
    """Continental landmass fill from GSHHG crude shorelines."""
    reader = shapefile.Reader(gshhg_path(GSHHG_LAND_REL))
    kept = 0
    for shape in reader.shapes():
        for ring in iter_shape_rings(shape):
            lons = [p[0] for p in ring]
            lats = [p[1] for p in ring]
            if (max(lats) - min(lats) < MIN_GSHHG_EXTENT_DEG and
                    max(lons) - min(lons) < MIN_GSHHG_EXTENT_DEG):
                continue
            kept += emit_landmass_rings(lines, ring)
    return kept


def add_gshhg_high_res(lines):
    """High-res coast strokes and close/mid-range land fill from one GSHHG pass."""
    reader = shapefile.Reader(gshhg_path(GSHHG_COAST_REL))
    coast = 0
    detail = 0
    for shape in reader.shapes():
        for ring in iter_shape_rings(shape):
            lons = [p[0] for p in ring]
            lats = [p[1] for p in ring]
            if (max(lats) - min(lats) < MIN_GSHHG_EXTENT_DEG and
                    max(lons) - min(lons) < MIN_GSHHG_EXTENT_DEG):
                continue
            pts = decimate(ring, MIN_GSHHG_SPACING_DEG)
            if len(pts) < 3:
                continue
            coast += append_line_chunks(lines, CLASS_COAST, pts)
            detail += emit_detail_landmass(lines, ring)
    return detail, coast


def add_gshhg_shorelines(lines):
    """Landmass fills (low-res silhouettes + high-res detail) + coast strokes."""
    land_rings = add_gshhg_landmass(lines)
    detail_rings, coast_lines = add_gshhg_high_res(lines)
    return land_rings + detail_rings, coast_lines


def add_lines(lines, data, line_class, keep=lambda props: True,
              min_spacing=MIN_POINT_SPACING_DEG):
    kept = 0
    for feature in data["features"]:
        props = feature.get("properties") or {}
        geometry = feature.get("geometry") or {}
        if not keep(props):
            continue
        for coords in each_linestring(geometry):
            pts = decimate(coords, min_spacing)
            if len(pts) < 2:
                continue
            if line_class == CLASS_LAKE:
                lats = [p[0] for p in pts]
                lons = [p[1] for p in pts]
                if (max(lats) - min(lats) < MIN_LAKE_EXTENT_DEG and
                        max(lons) - min(lons) < MIN_LAKE_EXTENT_DEG):
                    continue
            kept += append_line_chunks(lines, line_class, pts)
    return kept


def polygon_centroid(coords):
    """Rough label anchor from a GeoJSON ring's vertex average."""
    lats = [c[1] for c in coords]
    lons = [c[0] for c in coords]
    return sum(lats) / len(lats), sum(lons) / len(lons)


def feature_centroid(geometry):
    gtype = geometry.get("type")
    if gtype == "Point":
        lon, lat = geometry["coordinates"][:2]
        return lat, lon
    if gtype == "Polygon":
        return polygon_centroid(geometry["coordinates"][0])
    if gtype == "MultiPolygon":
        best = None
        best_area = 0.0
        for poly in geometry["coordinates"]:
            ring = poly[0]
            lats = [c[1] for c in ring]
            lons = [c[0] for c in ring]
            area = (max(lats) - min(lats)) * (max(lons) - min(lons))
            if area > best_area:
                best_area = area
                best = ring
        if best is not None:
            return polygon_centroid(best)
    return None


def scalerank_to_rank(scalerank, default=5):
    if scalerank is None:
        return default
    return max(0, min(10, 10 - int(scalerank)))


def feature_area(geometry):
    """Rough bbox area (deg²) for label-priority sorting."""
    gtype = geometry.get("type")

    def ring_area(ring):
        lats = [c[1] for c in ring]
        lons = [c[0] for c in ring]
        return (max(lats) - min(lats)) * (max(lons) - min(lons))

    if gtype == "Polygon":
        return ring_area(geometry["coordinates"][0])
    if gtype == "MultiPolygon":
        return max(ring_area(poly[0]) for poly in geometry["coordinates"])
    return 0.0


def country_rank_from_area(area_sq_deg):
    """Map land area to 0-10 prominence for range declutter.

    Natural Earth gives every country scalerank 0, so rank must come from size.
    At 1000 NM the real NXi keeps only the largest nations (USA, Canada,
    Mexico, Cuba, …) — not every Central American or Caribbean state.
    """
    if area_sq_deg >= 500.0:
        return 10
    if area_sq_deg >= 30.0:
        return 9
    if area_sq_deg >= 12.0:
        return 8
    if area_sq_deg >= 5.0:
        return 7
    if area_sq_deg >= 2.0:
        return 6
    if area_sq_deg >= 0.8:
        return 5
    return 4


def hydro_rank_from_area(area_sq_deg):
    """Water-body label prominence from bbox area (deg²).

    At 1000 NM the NXi labels only the largest lakes (Great Lakes tier) and
    major gulfs/bays — not every reservoir or small inland lake.
    """
    if area_sq_deg >= 15.0:
        return 10
    if area_sq_deg >= 7.0:
        return 9
    if area_sq_deg >= 4.0:
        return 8
    if area_sq_deg >= 2.0:
        return 6
    if area_sq_deg >= 0.8:
        return 5
    return 4


def add_geo_label(labels, name, lat, lon, rank, kind):
    name = name.strip().upper()
    if not name:
        return
    key = (kind, name)
    existing = labels.get(key)
    if existing is None or rank > existing[0]:
        labels[key] = (rank, lat, lon)


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

    # Nation borders: 10 m geometry keeps small-country outlines (50 m drops
    # many South American segments). Light decimation only.
    borders = fetch("ne_10m_admin_0_boundary_lines_land.geojson")
    n = add_lines(lines, borders, CLASS_BORDER, min_spacing=0.0015)
    print(f"borders: {n} lines")

    coast_land, coast_lines = add_gshhg_shorelines(lines)
    print(f"GSHHG landmass: {coast_land} rings")
    print(f"GSHHG coastlines: {coast_lines} lines")

    states = fetch("ne_10m_admin_1_states_provinces_lines.geojson")
    n = add_lines(lines, states, CLASS_STATE)
    print(f"state/province borders: {n} lines")

    railroads = fetch("ne_10m_railroads.geojson")
    n = add_lines(lines, railroads, CLASS_RAIL)
    print(f"railroads: {n} lines")

    places = fetch("ne_10m_populated_places_simple.geojson")
    labels = {}
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
        rank = scalerank_to_rank(scalerank)
        add_geo_label(labels, name, lat, lon, rank, KIND_CITY)
    print(f"cities: {sum(1 for k in labels if k[0] == KIND_CITY)}")

    # Lakes: hydro labels ranked by water-body size.
    for feature in lakes["features"]:
        props = feature.get("properties") or {}
        geometry = feature.get("geometry") or {}
        name = props.get("label") or props.get("name") or ""
        center = feature_centroid(geometry)
        if center is None:
            continue
        area = feature_area(geometry)
        if area < 0.4:
            continue
        lat, lon = center
        rank = hydro_rank_from_area(area)
        add_geo_label(labels, name, lat, lon, rank, KIND_HYDRO)
    print(f"hydro (lakes): {sum(1 for k in labels if k[0] == KIND_HYDRO)}")

    marine = fetch("ne_10m_geography_marine_polys.geojson")
    for feature in marine["features"]:
        props = feature.get("properties") or {}
        geometry = feature.get("geometry") or {}
        scalerank = props.get("scalerank")
        if scalerank is None or scalerank > 2:
            continue
        name = props.get("label") or props.get("name") or ""
        center = feature_centroid(geometry)
        if center is None:
            continue
        area = feature_area(geometry)
        if area < 2.0:
            continue
        lat, lon = center
        rank = hydro_rank_from_area(area)
        add_geo_label(labels, name, lat, lon, rank, KIND_HYDRO)
    print(f"hydro (total): {sum(1 for k in labels if k[0] == KIND_HYDRO)}")

    # Country names (UNITED STATES OF AMERICA, CANADA, MEXICO, etc.).
    countries = fetch("ne_10m_admin_0_countries.geojson")
    for feature in countries["features"]:
        props = feature.get("properties") or {}
        geometry = feature.get("geometry") or {}
        name = (props.get("NAME") or props.get("ADMIN") or "").strip()
        if not name or len(name) > 48:
            continue
        center = feature_centroid(geometry)
        if center is None:
            continue
        lat, lon = center
        area = feature_area(geometry)
        rank = country_rank_from_area(area)
        add_geo_label(labels, name, lat, lon, rank, KIND_REGION)
    print(f"countries: {sum(1 for k in labels if k[0] == KIND_REGION)}")

    # Named geographic regions (islands, seas, bays) from point layer.
    reg_pts = fetch("ne_10m_geography_regions_points.geojson")
    for feature in reg_pts["features"]:
        props = feature.get("properties") or {}
        geometry = feature.get("geometry") or {}
        if geometry.get("type") != "Point":
            continue
        scalerank = props.get("scalerank")
        if scalerank is None or scalerank > 3:
            continue
        name = props.get("label") or props.get("name") or ""
        lon, lat = geometry["coordinates"][:2]
        rank = scalerank_to_rank(scalerank)
        add_geo_label(labels, name, lat, lon, rank, KIND_HYDRO)
    print(f"hydro (regions pts): {sum(1 for k in labels if k[0] == KIND_HYDRO)}")

    # US states / provinces: large region labels centered on each polygon.
    states_polys = fetch("ne_10m_admin_1_states_provinces_scale_rank.geojson")
    for feature in states_polys["features"]:
        props = feature.get("properties") or {}
        geometry = feature.get("geometry") or {}
        scalerank = props.get("scalerank")
        if scalerank is None or scalerank > 3:
            continue
        name = props.get("label") or props.get("name") or ""
        if not name or len(name) > 32:
            continue
        center = feature_centroid(geometry)
        if center is None:
            continue
        lat, lon = center
        rank = min(scalerank_to_rank(scalerank, default=8), 5)
        add_geo_label(labels, name, lat, lon, rank, KIND_REGION)
    print(f"regions: {sum(1 for k in labels if k[0] == KIND_REGION)}")

    label_records = []
    for (kind, _name), (rank, lat, lon) in labels.items():
        label_records.append((lat, lon, rank, kind, _name.encode("utf-8")[:255]))
    print(f"labels total: {len(label_records)}")

    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, "wb") as f:
        f.write(struct.pack("<IIII", MAGIC, VERSION, len(lines), len(label_records)))
        for line_class, pts in lines:
            f.write(struct.pack("<BH", line_class, len(pts)))
            for lat, lon in pts:
                f.write(struct.pack("<ff", lat, lon))
        for lat, lon, rank, kind, name in label_records:
            f.write(struct.pack("<ffBBB", lat, lon, rank, kind, len(name)))
            f.write(name)

    size_mb = os.path.getsize(output) / (1024 * 1024)
    print(f"wrote {output} ({size_mb:.1f} MB, "
          f"{len(lines)} lines, {len(label_records)} labels)")


if __name__ == "__main__":
    main()
