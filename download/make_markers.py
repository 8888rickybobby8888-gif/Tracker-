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


def single_pdf(mid, size_m, path=None):
    """One marker alone on an A4 page at TRUE SIZE (print at 100% / actual size), with a 10 cm check bar under it. Returns the file name."""
    from PIL import Image
    img = label(make(mid, size_m), role(mid))
    page = np.full((A4[1], A4[0]), 255, np.uint8)
    h, w = img.shape
    y0, x0 = int(0.03 * A4[1]), (A4[0] - w) // 2
    page[y0:y0 + h, x0:x0 + w] = img
    cv2.rectangle(page, (x0 - 2, y0 - 2), (x0 + w + 1, y0 + h + 1), 170, 1)          # light cut line
    bar = int(round(0.10 * PX_PER_M))                                                   # 10 cm check bar: measure it after printing
    by = y0 + h + int(0.02 * A4[1])
    bx = (A4[0] - bar) // 2
    cv2.line(page, (bx, by), (bx + bar, by), 0, 3)
    cv2.line(page, (bx, by - 12), (bx, by + 12), 0, 3)
    cv2.line(page, (bx + bar, by - 12), (bx + bar, by + 12), 0, 3)
    cv2.putText(page, "This bar must measure 10.0 cm. If not, print at 100% / actual size (not 'fit to page').", (bx - 260, by + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, 0, 2, cv2.LINE_AA)
    path = path or f"markers/marker_{mid}.pdf"
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    Image.fromarray(page).save(path, resolution=DPI)
    return path


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
    import argparse
    ap = argparse.ArgumentParser(description="Print-ready ArUco markers at true size")
    ap.add_argument("--size", type=float, default=None, metavar="METRES", help="black-square size of each FOLLOWER marker, e.g. 0.05 for 5 cm (default: MARKER_SIZE in tracker.py)")
    ap.add_argument("--floor-size", type=float, default=None, metavar="METRES", help="size of the floor marker (default: FLOOR_SIZE in tracker.py)")
    ap.add_argument("--markers", choices=["tower3", "tower", "post", "top"], default=None, help="tower3 = flat ID 0 on top + IDs 1 left, 2 back, 3 right (default), tower = flat ID 0 + faces IDs 1-4, post = faces IDs 0-3, top = one flat marker")
    ap.add_argument("--only", type=int, default=None, metavar="ID", help="write just this marker as markers/marker_<ID>.pdf (one A4 page, true size) and stop")
    a = ap.parse_args()
    tk.set_layout(a.markers or tk.MARKER_LAYOUT)
    if a.size:
        tk.MARKER_SIZE = a.size
    if a.floor_size:
        tk.FLOOR_SIZE = a.floor_size
    print(f"Layout {tk.MARKER_LAYOUT}: follower markers {sorted(tk.FACES)} at {tk.MARKER_SIZE * 100:.1f} cm, floor marker {tk.FLOOR_ID} at {tk.FLOOR_SIZE * 100:.0f} cm. "
          f"Remember to set MARKER_SIZE (tracker.py or the Settings page) to the size you print.")
    if a.only is not None:
        size = tk.FLOOR_SIZE if a.only == tk.FLOOR_ID else tk.MARKER_SIZE
        print("wrote", single_pdf(a.only, size), f"({size * 100:.1f} cm marker, print at 100%)")
    else:
        main()
