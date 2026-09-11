#!/usr/bin/python3
"""Dimensioned orthographic CAD illustration from the exported triangle meshes."""

import json

import numpy as np
from build_akshu_candidate import DTYPE, TARGET
from PIL import Image, ImageDraw, ImageFont


def main():
    report = json.loads((TARGET / "packaging.json").read_text())
    page = Image.new("RGB", (1500, 1420), "#f4f6f8")
    pen = ImageDraw.Draw(page)
    font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
    font = ImageFont.truetype(font_path, 17)
    small = ImageFont.truetype(font_path, 14)
    title = ImageFont.truetype(font_path, 30)
    pen.text((45, 25), "ICARUS / CUSTOM AKSHU V1", fill="#102638", font=title)
    pen.text(
        (45, 68),
        "Dimensioned packaging study • millimetres • orthographic views • NOT fabrication drawings",
        fill="#405469",
        font=font,
    )
    geometry = []
    for part in report["parts"]:
        path = TARGET / "meshes" / (part["name"] + ".stl")
        triangles = (
            np.frombuffer(path.read_bytes(), dtype=DTYPE, offset=84)["v"].astype(float)
            * 1000
        )
        color = tuple(int(float(v) * 255) for v in part["color"].split()[:3])
        geometry.append((triangles, color))

    def dimension(a, b, text, vertical=False):
        pen.line([a, b], fill="#16677e", width=2)
        for x, y in (a, b):
            pen.line([(x - 5, y - 5), (x + 5, y + 5)], fill="#16677e", width=2)
        x, y = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        pen.text(
            (x + 8 if vertical else x, y - 20),
            text,
            font=small,
            fill="#16677e",
            anchor="la" if vertical else "ma",
            stroke_width=2,
            stroke_fill="#f4f6f8",
        )

    def view(label, center, u, v, depth):
        cx, cy = center
        scale = 0.96
        pen.text((cx - 290, cy - 305), label, font=font, fill="#102638")
        triangles = np.concatenate([g[0] for g in geometry])
        colors = np.concatenate(
            [np.repeat([g[1]], len(g[0]), axis=0) for g in geometry]
        )
        order = np.argsort(triangles.mean(axis=1) @ depth)
        projected = np.stack(
            [cx + scale * (triangles @ u), cy - scale * (triangles @ v)], axis=2
        )
        for idx in order:
            pen.polygon([tuple(p) for p in projected[idx]], fill=tuple(colors[idx]))
        if label.startswith("TOP"):
            for x in (-138.5, 138.5):
                for y in (-173.387, 173.387):
                    px, py = cx - scale * y, cy - scale * x
                    r = 127 * scale
                    pen.ellipse(
                        (px - r, py - r, px + r, py + r), outline="#8097a6", width=1
                    )
            dimension(
                (cx - 300.387 * scale, cy + 280),
                (cx + 300.387 * scale, cy + 280),
                "600.8 swept width",
            )
            dimension(
                (cx + 310, cy - 265.5 * scale),
                (cx + 310, cy + 265.5 * scale),
                "531.0",
                True,
            )
            pen.text(
                (cx, cy - 282), "FRONT / +X ↑", font=small, fill="#16677e", anchor="ma"
            )
        else:
            dimension(
                (cx - 250, cy + 190.1 * scale),
                (cx - 250, cy - 163 * scale),
                "353.1 high",
                True,
            )
            pen.line(
                [(cx - 270, cy + 190.1 * scale), (cx + 270, cy + 190.1 * scale)],
                fill="#8097a6",
                width=1,
            )
            pen.text(
                (cx, cy + 210),
                "Ground clearance under battery tray: 45.6 mm",
                font=small,
                fill="#16677e",
                anchor="ma",
            )

    view(
        "TOP / full 254 mm rotor sweep",
        (365, 425),
        np.array([0, -1, 0]),
        np.array([1, 0, 0]),
        np.array([0, 0, 1]),
    )
    view(
        "FRONT / looking toward -X",
        (1110, 425),
        np.array([0, -1, 0]),
        np.array([0, 0, 1]),
        np.array([1, 0, 0]),
    )
    view(
        "SIDE / front is right",
        (365, 1080),
        np.array([1, 0, 0]),
        np.array([0, 0, 1]),
        np.array([0, -1, 0]),
    )
    x, y = 790, 770
    pen.text((x, y), "REBUILT COMPONENT ALLOCATIONS", font=font, fill="#102638")
    rows = [
        "Compute + carrier + cooling: 140 × 100 × 51",
        "Pixhawk / baseboard allowance: 90 × 60 × 32",
        "6S 10 Ah battery allowance: 180 × 75 × 55",
        "MID-360 nominal envelope: 65 × 65 × 60",
        "Forward camera allowance: 30 × 100 × 32",
        "LW20/C nominal envelope: 30 × 20 × 43",
        "GNSS allowance: 45 × 45 × 18",
        "Lower centre plate: 200 × 110 × 5",
        "Compute shelf: 150 × 110 × 5",
        "Motor diagonal: approximately 443.8",
        "Nearest adjacent 254 mm prop clearance: 23.0",
        "Blade envelope → compute shelf: 20.5 vertical",
        "Landing-gear extension: 35",
    ]
    for i, row in enumerate(rows):
        pen.text((x, y + 40 + i * 28), row, font=font, fill="#405469")
    pen.text(
        (45, 1360),
        "Original arms and leg shape retained; source electronics removed. Hardware envelopes ≠ manufacturer CAD.",
        font=font,
        fill="#8d3e24",
    )
    output = TARGET / "packaging_dimensions.png"
    page.save(output)
    print(output)


if __name__ == "__main__":
    main()
