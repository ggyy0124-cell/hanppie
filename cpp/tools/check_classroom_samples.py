"""Compile the actual embedded classroom programs without running robot code."""

from __future__ import annotations

import argparse
import json
import subprocess
import tempfile
from pathlib import Path

__all__: list[str] = []

PAGE = Path(__file__).resolve().parent.parent / "examples" / "classroom.html"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--header", type=Path, required=True)
    parser.add_argument("--compiler", default="g++")
    args = parser.parse_args()
    header = Path(str(args.header)).resolve()
    page = PAGE.read_text(encoding="utf-8")
    samples: list[dict[str, str]] = json.loads(
        page.split("const SAMPLES = ", 1)[1].split(";\n// END GENERATED SAMPLES", 1)[0]
    )
    with tempfile.TemporaryDirectory(prefix="hanppie-samples-") as directory:
        work = Path(directory)
        for index, sample in enumerate(samples):
            source = work / f"sample-{index}.cpp"
            source.write_text(sample["code"], encoding="utf-8")
            result = subprocess.run(
                [
                    str(args.compiler),
                    "-O2",
                    "-std=c++11",
                    "-DONLINE_JUDGE",
                    "-I" + str(header.parent),
                    str(source),
                    "-o",
                    str(work / f"sample-{index}.exe"),
                ],
                capture_output=True,
                text=True,
                errors="replace",
                check=False,
            )
            if result.returncode:
                raise RuntimeError(f"{sample['name']}\n{result.stderr}")
            print(f"{index + 1}/{len(samples)}: compiled", flush=True)


if __name__ == "__main__":
    main()
