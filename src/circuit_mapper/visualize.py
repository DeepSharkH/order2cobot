"""Render a mapper result as one self-contained HTML page of the breadboard."""

import argparse
import json
from pathlib import Path

HERE = Path(__file__).parent


def read_json(path):
    return json.loads(Path(path).read_text())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("circuit", type=Path)
    parser.add_argument("mapping", type=Path, help="mapper.py --output file")
    parser.add_argument("--inventory", type=Path, default=HERE / "example_components.json")
    parser.add_argument("--board", type=Path, default=HERE / "board_connections.json")
    parser.add_argument("--out", type=Path, required=True, help="directory for index.html")
    args = parser.parse_args()
    circuit, mapping, board = read_json(args.circuit), read_json(args.mapping), read_json(args.board)
    if mapping["circuit_id"] != circuit["circuit_id"] or mapping["board_id"] != board["board_id"]:
        parser.error("mapping was made for a different circuit or board")
    used = {c["part_id"] for c in circuit["components"]} | {j["part_id"] for j in mapping["jumpers"]}
    parts = [p for p in read_json(args.inventory)["components"] if p["part_id"] in used]
    data = json.dumps({"circuit": circuit, "mapping": mapping, "board": board, "parts": parts})
    # "</" inside the inline JSON would end the <script> element early.
    html = (HERE / "visualize.html").read_text().replace("__DATA__", data.replace("</", "<\\/"), 1)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "index.html").write_text(html)
    (args.out / "circuit.json").write_text(args.circuit.read_text())
    (args.out / "mapping.json").write_text(args.mapping.read_text())
    print(args.out / "index.html")


if __name__ == "__main__":
    main()
