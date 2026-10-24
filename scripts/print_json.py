#!/usr/bin/env python3
"""Print a JSON file, or one field of it.

    scripts/print_json.py results/<run>/metadata.json             # whole file
    scripts/print_json.py results/<run>/metadata.json summary     # one field
    scripts/print_json.py results/<run>/metadata.json llm.model   # nested field
    scripts/print_json.py results/<run>/metadata.json summary -m  # render Markdown

Text fields are printed as-is, so a Markdown summary keeps its line breaks;
anything else is printed as indented JSON. With --markdown, a text field is
rendered as formatted Markdown (needs the `rich` package).
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Any


def get_field(data: Any, path: str) -> Any:
    """Follow a dotted path such as "llm.model" or "items.0.name"."""
    for key in path.split("."):
        if isinstance(data, list) and key.isdigit():
            data = data[int(key)]
        elif isinstance(data, dict) and key in data:
            data = data[key]
        else:
            raise KeyError(path)
    return data


def main() -> None:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("file", type=Path, help="JSON file to read")
    ap.add_argument("field", nargs="?", help="dotted path of the field to print (default: all)")
    ap.add_argument("-m", "--markdown", action="store_true", help="render a text field as Markdown")
    args = ap.parse_args()

    try:
        data = json.loads(args.file.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        sys.exit(f"error: {args.file}: {exc}")

    if args.field:
        try:
            data = get_field(data, args.field)
        except (KeyError, IndexError):
            sys.exit(f"error: {args.file} has no field '{args.field}'")

    if not isinstance(data, str):
        print(json.dumps(data, indent=2, ensure_ascii=False))
    elif args.markdown:
        print_markdown(data)
    else:
        print(data)


def print_markdown(text: str) -> None:
    try:
        from rich.console import Console
        from rich.markdown import Markdown
    except ImportError:
        sys.exit("error: --markdown needs the rich package; run with `uv run`")
    Console().print(Markdown(text))


if __name__ == "__main__":
    main()
