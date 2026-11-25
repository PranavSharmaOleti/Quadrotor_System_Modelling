#!/usr/bin/env python3
"""Compile report/final_report.md into publication-quality PDF and DOCX.

PDF : pandoc -> tectonic (XeTeX-compatible TeX engine), with geometry=0.85in.
DOCX: pandoc directly from Markdown.

Requires:
  - pypandoc-binary (bundled pandoc) or pandoc on PATH
  - tectonic on PATH (https://github.com/tectonic-typesetting/tectonic)

Run from anywhere: python3 report/build_report.py
"""

import os
import shutil
import subprocess
from pathlib import Path

import pypandoc

HERE = Path(__file__).resolve().parent
MD = HERE / "final_report.md"
PREAMBLE = HERE / "preamble.tex"
PDF = HERE / "final_report.pdf"
DOCX = HERE / "final_report.docx"

PANDOC = shutil.which("pandoc") or pypandoc.get_pandoc_path()
TECTONIC = shutil.which("tectonic") or str(Path.home() / ".local/bin/tectonic")


def main() -> None:
    previous = Path.cwd()
    os.chdir(HERE)
    try:
        env = dict(os.environ)
        env["PATH"] = str(Path(TECTONIC).parent) + os.pathsep + env.get("PATH", "")
        subprocess.run(
            [PANDOC, MD.name, "-o", PDF.name,
             "--pdf-engine=tectonic",
             "-V", "geometry:margin=0.85in",
             "-V", "fontsize=10pt",
             "-V", "documentclass=article",
             "--toc", "--toc-depth=2",
             "-H", PREAMBLE.name],
            check=True, env=env,
        )
        pypandoc.convert_file(
            MD.name, "docx", outputfile=DOCX.name, format="md",
            extra_args=["--toc", "--toc-depth=2", "--resource-path=."],
        )
        print(f"PDF : {PDF} ({PDF.stat().st_size} bytes)")
        print(f"DOCX: {DOCX} ({DOCX.stat().st_size} bytes)")
    finally:
        os.chdir(previous)


if __name__ == "__main__":
    main()
