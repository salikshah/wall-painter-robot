"""Title and end cards for the demo videos (1920x1080, PIL)."""
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
REPORTS = ROOT / 'sim/reports'
W, H = 1920, 1080
S = H / 720.0
BG = (16, 20, 28)
SLATE = (60, 68, 82)
YELLOW = (237, 194, 41)
GREEN = (150, 220, 160)
ORANGE = (240, 170, 120)
TEXT = (232, 236, 242)
DIM = (160, 172, 188)


def px(v):
    return int(round(v * S))


def font(name, size):
    try:
        return ImageFont.truetype(name, size)
    except OSError:
        return ImageFont.load_default()


def chip(dr, xy, text, fnt, fg, border):
    x, y = xy
    pad = px(14)
    bb = dr.textbbox((0, 0), text, font=fnt)
    w, h = bb[2] - bb[0], bb[3] - bb[1]
    box = [x, y, x + w + 2 * pad, y + h + 2 * pad]
    dr.rounded_rectangle(box, radius=px(10), outline=border, width=px(3),
                         fill=(255, 255, 255, 10))
    dr.text((x + pad, y + pad - px(3)), text, fill=fg, font=fnt)
    return box[2]


def lines(dr, x, y, rows, f_big, f_med):
    for row in rows:
        r = row[0]
        text = row[1]
        if r == 'big':
            dr.text((x, y), text, fill=TEXT, font=f_big)
            y += f_big.size + px(14)
        elif r == 'sub':
            dr.text((x, y), text, fill=DIM, font=f_med)
            y += f_med.size + px(12)
        elif r == 'yellow':
            dr.text((x, y), text, fill=YELLOW, font=f_big)
            y += f_big.size + px(14)
        elif r == 'gap':
            y += px(26)
    return y


def make_card(kind, mode):
    img = Image.new('RGB', (W, H), BG)
    dr = ImageDraw.Draw(img)
    dr.rectangle([0, 0, W - 1, px(6)], fill=YELLOW)
    f_title = font('arialbd.ttf', px(96))
    f_big = font('arialbd.ttf', px(34))
    f_med = font('arial.ttf', px(27))
    f_small = font('arial.ttf', px(22))
    if kind == 'intro':
        dr.text((px(84), px(150)), 'WallFin', fill=YELLOW, font=f_title)
        y = lines(dr, px(88), px(290), [
            ('sub', 'force-controlled wall sanding \u00b7 SO-101 arm \u00b7 physics sim'),
            ('gap', ''),
            ('sub', '3 passes \u00b7 610 mm wall \u00b7 4 mm seam + screw head proud 2.5 mm'),
        ], f_big, f_med)
        tag = ('CONTROLLED \u2014 force loop, 5 N target' if mode == 'controlled'
               else 'BASELINE \u2014 rigid position hold')
        col = GREEN if mode == 'controlled' else ORANGE
        chip(dr, (px(88), px(470)), tag, f_med, col, col)
    else:
        dr.text((px(84), px(130)), 'WallFin', fill=YELLOW, font=f_title)
        dr.text((px(88), px(280)), 'controlled', fill=GREEN, font=f_big)
        dr.text((px(88), px(335)), '97.8 % contact duty   100 % coverage   \u00b11.1 N error',
                fill=TEXT, font=f_med)
        dr.text((px(88), px(410)), 'baseline', fill=ORANGE, font=f_big)
        dr.text((px(88), px(465)), '61.6 % contact duty   71.9 % coverage   \u00b12.8 N error',
                fill=TEXT, font=f_med)
        dr.text((px(88), px(570)), 'same arm, same wall, one control loop difference',
                fill=DIM, font=f_med)
    dr.text((px(84), px(660)), 'wallfin-sim \u00b7 simulation in MuJoCo',
            fill=SLATE, font=f_small)
    out = REPORTS / f'card_{kind}_{mode}.png'
    img.save(out)
    return out


if __name__ == '__main__':
    for mode in ('controlled', 'baseline'):
        make_card('intro', mode)
        make_card('end', mode)
    print('cards saved')
