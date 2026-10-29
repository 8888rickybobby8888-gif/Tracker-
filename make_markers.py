"""
make_markers.py - generates printable ArUco markers for tracker.py.

    python make_markers.py

Writes:
  markers/marker_<id>.png   individual markers (white border included)
  markers_sheet.pdf         A4 sheet at TRUE SIZE - print at 100% / "actual size"
                            (turn OFF "fit to page"), on matte paper.

Marker sizes are read from tracker.py (MARKER_SIZE, FLOOR_SIZE), so change them
there, then re-run this. After printing, measure the black square with a ruler:
it must match the config.
"""
import os

import cv2
import numpy as np

import tracker as tk

DPI = 300
PX_PER_M = DPI / 0.0254
A4 = (int(8.27 * DPI), int(11.69 * DPI))        # (w, h) pixels
GAP = int(0.003 * PX_PER_M)                      # 3 mm between tiles


def make(marker_id, size_m):
    """Marker with a white quiet-zone border of one module on every side."""
    side = int(round(size_m * PX_PER_M / 6)) * 6          # multiple of 6 -> crisp modules
    d = cv2.aruco.getPredefinedDictionary(tk.DICTIONARY)
    m = cv2.aruco.generateImageMarker(d, marker_id, side)
    pad = side // 6
    return cv2.copyMakeBorder(m, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)


def label(img, text):
    out = cv2.copyMakeBorder(img, 0, 40, 0, 0, cv2.BORDER_CONSTANT, value=255)
    cv2.putText(out, text, (8, out.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.9, 0, 2, cv2.LINE_AA)
    return out


def role(mid):
    if mid == tk.FLOOR_ID:
        return f"ID {mid} FLOOR  {tk.FLOOR_SIZE * 100:.0f} cm"
    robot, ang = tk.FACES[mid]
    face = "top, front edge up" if ang is None else {0: "front", 90: "left", 180: "back", 270: "right"}.get(ang, f"{ang} deg")
    return f"ID {mid} {robot} {face}  {tk.MARKER_SIZE * 100:.0f} cm"


def main():
    os.makedirs("markers", exist_ok=True)
    tiles = []
    ids = [tk.FLOOR_ID] + sorted(tk.FACES)
    for mid in ids:
        size = tk.FLOOR_SIZE if mid == tk.FLOOR_ID else tk.MARKER_SIZE
        img = label(make(mid, size), role(mid))
        cv2.imwrite(f"markers/marker_{mid}.png", img)
        tiles.append(img)

    # simple shelf packing onto A4 pages, true size
    pages, page, x, y, row_h = [], np.full((A4[1], A4[0]), 255, np.uint8), GAP, GAP, 0
    for t in tiles:
        h, w = t.shape
        if x + w + GAP > A4[0]:
            x, y, row_h = GAP, y + row_h + GAP, 0
        if y + h + GAP > A4[1]:
            pages.append(page)
            page, x, y, row_h = np.full((A4[1], A4[0]), 255, np.uint8), GAP, GAP, 0
        page[y:y + h, x:x + w] = t
        cv2.rectangle(page, (x - 2, y - 2), (x + w + 1, y + h + 1), 170, 1)   # light cut line
        x += w + GAP
        row_h = max(row_h, h)
    pages.append(page)

    from PIL import Image
    ims = [Image.fromarray(p) for p in pages]
    ims[0].save("markers_sheet.pdf", save_all=True, append_images=ims[1:], resolution=DPI)
    print(f"wrote markers/ ({len(tiles)} PNGs) and markers_sheet.pdf ({len(pages)} page(s)) at true size")


if __name__ == "__main__":
    main()
