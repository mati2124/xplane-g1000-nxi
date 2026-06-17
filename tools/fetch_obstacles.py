#!/usr/bin/env python3
"""Download the FAA Daily Digital Obstacle File (DDOF) CSV for the map overlay.

The G1000 NXi obstacle layer reads the FAA's public DDOF CSV distribution
(LATDEC/LONDEC decimal coordinates, AGL/AMSL heights in feet). US coverage only;
the file is large (~90 MB) and updated daily, so it is not committed to git.

Usage:
    python3 tools/fetch_obstacles.py [output.csv]

Output defaults to shell-standalone/assets/obstacles.csv. Downloads are cached
in tools/.dof_cache/ so re-runs are offline unless the cache is cleared.

Source: https://www.faa.gov/air_traffic/flight_info/aeronav/digital_products/dailydof/
"""

import os
import sys
import urllib.request
import zipfile

DEFAULT_URL = "https://aeronav.faa.gov/Obst_Data/DAILY_DOF_CSV.ZIP"
CACHE_DIR = os.path.join(os.path.dirname(__file__), ".dof_cache")
DEFAULT_OUTPUT = os.path.join(
    os.path.dirname(__file__), "..", "shell-standalone", "assets", "obstacles.csv"
)
ZIP_MEMBER = "DOF.CSV"


def fetch(url=DEFAULT_URL):
    os.makedirs(CACHE_DIR, exist_ok=True)
    cached_zip = os.path.join(CACHE_DIR, "DAILY_DOF_CSV.ZIP")
    if not os.path.exists(cached_zip):
        print("downloading", url)
        with urllib.request.urlopen(url) as response, open(cached_zip, "wb") as out:
            out.write(response.read())
    else:
        print("using cached", cached_zip)
    return cached_zip


def extract_csv(zip_path, output_path):
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    with zipfile.ZipFile(zip_path) as zf:
        names = zf.namelist()
        member = ZIP_MEMBER if ZIP_MEMBER in names else names[0]
        print("extracting", member, "->", output_path)
        with zf.open(member) as src, open(output_path, "wb") as dst:
            while True:
                chunk = src.read(1024 * 1024)
                if not chunk:
                    break
                dst.write(chunk)


def main():
    output = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_OUTPUT
    zip_path = fetch()
    extract_csv(zip_path, output)
    size_mb = os.path.getsize(output) / (1024 * 1024)
    print(f"wrote {output} ({size_mb:.1f} MB)")


if __name__ == "__main__":
    main()
