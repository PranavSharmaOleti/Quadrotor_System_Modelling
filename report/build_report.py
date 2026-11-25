#!/usr/bin/env python3
"""Compile report/final_report.md into PDF and DOCX.

PDF : python-markdown -> HTML -> WeasyPrint (no LaTeX toolchain required).
DOCX: pandoc (bundled via pypandoc-binary).

Run from anywhere: python3 report/build_report.py
"""

import os
from pathlib import Path

import markdown
import pypandoc
from weasyprint import HTML

HERE = Path(__file__).resolve().parent
MD = HERE / "final_report.md"
PDF = HERE / "final_report.pdf"
DOCX = HERE / "final_report.docx"

CSS = """
@page { size: A4; margin: 18mm 16mm 18mm 16mm; }
body { font-family: "DejaVu Sans", "Liberation Sans", sans-serif;
       font-size: 9.5pt; line-height: 1.35; color: #1a1a1a; }
h1 { font-size: 20pt; color: #123; page-break-after: avoid; }
h2 { font-size: 14pt; color: #123; margin-top: 18pt; page-break-after: avoid;
     border-bottom: 1px solid #bbb; padding-bottom: 3pt; }
h3 { font-size: 11.5pt; color: #234; margin-top: 12pt; page-break-after: avoid; }
p { margin: 5pt 0; text-align: justify; }
table { border-collapse: collapse; width: 100%; font-size: 7.2pt;
        margin: 8pt 0; page-break-inside: auto; }
th, td { border: 0.5pt solid #999; padding: 2.5pt 3pt; text-align: left; }
th { background: #eef2f7; font-weight: bold; }
tr { page-break-inside: avoid; }
img { max-width: 100%; max-height: 8.5cm; display: block;
      margin: 6pt auto 2pt auto; }
em { color: #444; font-size: 8pt; }
code, pre { font-family: "DejaVu Sans Mono", monospace; font-size: 7.5pt; }
pre { background: #f5f6f8; border: 0.5pt solid #ddd; padding: 5pt;
      page-break-inside: avoid; white-space: pre-wrap; }
blockquote { border-left: 2pt solid #789; margin-left: 6pt; padding-left: 6pt; }
"""


def build_pdf() -> Path:
    text = MD.read_text(encoding="utf-8")
    html_body = markdown.markdown(
        text, extensions=["tables", "fenced_code", "sane_lists"]
    )
    html = f"<!DOCTYPE html><html><head><meta charset='utf-8'><style>{CSS}</style></head><body>{html_body}</body></html>"
    HTML(string=html, base_url=str(HERE)).write_pdf(str(PDF))
    return PDF


def build_docx() -> Path:
    # Pandoc resolves relative image paths from the working directory.
    previous = Path.cwd()
    try:
        os.chdir(HERE)
        pypandoc.convert_file(
            MD.name, "docx", outputfile=DOCX.name, format="md",
            extra_args=["--toc", "--toc-depth=2"],
        )
    finally:
        os.chdir(previous)
    return DOCX


def main() -> None:
    pdf = build_pdf()
    docx = build_docx()
    print(f"PDF : {pdf} ({pdf.stat().st_size} bytes)")
    print(f"DOCX: {docx} ({docx.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
