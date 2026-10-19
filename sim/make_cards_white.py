"""White Apple/Unitree-style intro and end cards (1920x1080).

Edit the TEXT and COLORS near the top, rerun, PNGs land in docs/cards/.
"""
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / 'docs/cards'
REPORTS = ROOT / 'sim/reports'
W, H = 1920, 1080

BG = (255, 255, 255)
INK = (29, 29, 31)          # near-black
GRAY = (110, 110, 115)      # secondary
LIGHT = (134, 134, 139)     # footer
RULE = (210, 210, 215)      # hairlines
GREEN = (46, 158, 91)
ORANGE = (217, 122, 36)
GOLD = (232, 184, 0)

F_LIGHT = 'C:/Windows/Fonts/segoeuil.ttf'
F_SEMI = 'C:/Windows/Fonts/segoeuisl.ttf'
F_REG = 'C:/Windows/Fonts/segoeui.ttf'
F_BOLD = 'C:/Windows/Fonts/segoeuib.ttf'

X = 140
TEXT = {
    'eyebrow': 'WALLFIN-SIM \u00b7 PHYSICS SIMULATION',
    'eyebrow_end': 'WALLFIN-SIM \u00b7 RESULTS',
    'descriptor': 'Force-controlled wall sanding',
    'meta': 'SO-101 arm \u00b7 3 passes \u00b7 610 mm wall \u00b7 4 mm seam \u00b7 2.5 mm screw head',
    'modes': {'controlled': 'CONTROLLED \u2014 force loop \u00b7 5 N target',
              'baseline': 'BASELINE \u2014 rigid position hold'},
    'footer': 'wallfin-sim \u00b7 simulation in MuJoCo',
    'head1': 'Same arm. Same wall.',
    'head2': 'One control loop difference.',
    'rows': [
        ('Contact duty', '97.8 %', '61.6 %'),
        ('Strip coverage', '100 %', '71.9 %'),
        ('Steady force error', '\u00b11.1 N', '\u00b12.8 N'),
    ],
}


def font(path, size):
    try:
        return ImageFont.truetype(path, size)
    except OSError:
        return ImageFont.load_default()


def tracked(dr, xy, text, fnt, fill, tr=3.6):
    x, y = xy
    for ch in text:
        dr.text((x, y), ch, font=fnt, fill=fill)
        x += dr.textlength(ch, font=fnt) + tr


def pill(dr, xy, text, fnt, fg, dot):
    x, y = xy
    pad_x, pad_y = 26, 15
    bb = dr.textbbox((0, 0), text, font=fnt)
    h = bb[3] - bb[1]
    box = [x, y, x + 2 * pad_x + 22 + (bb[2] - bb[0]), y + 2 * pad_y + h]
    dr.rounded_rectangle(box, radius=(box[3] - box[1]) // 2, outline=RULE, width=2)
    cy = (box[1] + box[3]) // 2
    dr.ellipse([x + pad_x, cy - 7, x + pad_x + 14, cy + 7], fill=dot)
    dr.text((x + pad_x + 36, y + pad_y - bb[1]), text, font=fnt, fill=fg)


def intro(mode):
    img = Image.new('RGB', (W, H), BG)
    dr = ImageDraw.Draw(img)
    tracked(dr, (X, 112), TEXT['eyebrow'], font(F_REG, 21), LIGHT)
    dr.text((X - 6, 232), 'WallFin', fill=INK, font=font(F_LIGHT, 178))
    dr.rectangle([X, 466, X + 64, 471], fill=GOLD)
    dr.text((X, 534), TEXT['descriptor'], fill=INK, font=font(F_SEMI, 56))
    dr.text((X, 640), TEXT['meta'], fill=GRAY, font=font(F_REG, 27))
    dot = GREEN if mode == 'controlled' else ORANGE
    pill(dr, (X, 786), TEXT['modes'][mode], font(F_REG, 27), INK, dot)
    dr.text((X, 970), TEXT['footer'], fill=LIGHT, font=font(F_REG, 19))
    out = OUT / f'intro_{mode}.png'
    img.save(out)
    img.save(REPORTS / f'card_intro_{mode}.png')
    return out


def end(mode):
    img = Image.new('RGB', (W, H), BG)
    dr = ImageDraw.Draw(img)
    tracked(dr, (X, 112), TEXT['eyebrow_end'], font(F_REG, 21), LIGHT)
    dr.text((X, 226), TEXT['head1'], fill=INK, font=font(F_LIGHT, 66))
    dr.text((X, 316), TEXT['head2'], fill=GRAY, font=font(F_LIGHT, 66))

    f_hdr, f_lbl, f_val = font(F_REG, 20), font(F_REG, 30), font(F_REG, 42)
    c1, c2 = 1240, 1780
    dr.line([X, 596, c2, 596], fill=RULE, width=2)
    dr.text((c1, 545), 'CONTROLLED \u00b7 5 N', font=f_hdr, fill=GRAY, anchor='ra')
    dr.text((c2, 545), 'FIXED DEPTH', font=f_hdr, fill=GRAY, anchor='ra')

    ys = (628, 740, 852)
    for i, (lbl, v1, v2) in enumerate(TEXT['rows']):
        y = ys[i]
        dr.text((X, y + 6), lbl, font=f_lbl, fill=GRAY)
        dr.text((c1, y), v1, font=f_val, fill=INK, anchor='ra')
        dr.text((c2, y), v2, font=f_val, fill=INK, anchor='ra')
        dr.line([X, y + 78, c2, y + 78], fill=(232, 232, 237), width=2)

    dr.text((X, 970), TEXT['footer'], fill=LIGHT, font=font(F_REG, 19))
    out = OUT / f'end_{mode}.png'
    img.save(out)
    img.save(REPORTS / f'card_end_{mode}.png')
    return out


if __name__ == '__main__':
    OUT.mkdir(parents=True, exist_ok=True)
    for mode in ('controlled', 'baseline'):
        print(intro(mode))
        print(end(mode))
