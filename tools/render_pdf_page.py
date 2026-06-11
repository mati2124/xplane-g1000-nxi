#!/usr/bin/env python3
"""Render page(s) of a PDF to PNG so figures/screenshots can be viewed.

The IDE's PDF reader converts to text and drops figures, so use this to see
the actual G1000 NXi screenshots in the Pilot's Guide.

Usage:
    python3 tools/render_pdf_page.py PAGE [END_PAGE] [--pdf PATH] [--dpi N] [--out DIR]

Pages are 1-based. Examples:
    python3 tools/render_pdf_page.py 42
    python3 tools/render_pdf_page.py 42 45 --dpi 200
Outputs: <out>/<pdf-stem>_p<NN>.png  (default out: /tmp/pdf_pages)
"""
import argparse
import sys
from pathlib import Path

import pypdfium2 as pdfium

DEFAULT_PDF = Path(__file__).resolve().parents[1] / "docs/reference/G1000_NXi_Pilots_Guide_Cessna_NavIII_190-02177-00.pdf"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("page", type=int, help="first page to render (1-based)")
    ap.add_argument("end_page", type=int, nargs="?", help="last page to render (inclusive)")
    ap.add_argument("--pdf", type=Path, default=DEFAULT_PDF, help="PDF path")
    ap.add_argument("--dpi", type=int, default=150, help="render resolution (default 150)")
    ap.add_argument("--out", type=Path, default=Path("/tmp/pdf_pages"), help="output directory")
    args = ap.parse_args()

    if not args.pdf.exists():
        print(f"PDF not found: {args.pdf}", file=sys.stderr)
        return 1

    args.out.mkdir(parents=True, exist_ok=True)
    pdf = pdfium.PdfDocument(str(args.pdf))
    n = len(pdf)
    first = args.page
    last = args.end_page or args.page
    if first < 1 or last > n or first > last:
        print(f"Invalid page range {first}-{last}; PDF has {n} pages.", file=sys.stderr)
        return 1

    scale = args.dpi / 72.0
    stem = args.pdf.stem
    for p in range(first, last + 1):
        page = pdf[p - 1]
        image = page.render(scale=scale).to_pil()
        out_path = args.out / f"{stem}_p{p:03d}.png"
        image.save(out_path)
        print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
