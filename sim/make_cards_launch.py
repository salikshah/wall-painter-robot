"""Launch-film cards: intro explainers, mode dividers, CTA (1920x1080)."""
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / 'docs/cards'
REPORTS = ROOT / 'sim/reports'
W, H = 1920, 1080
X = 140

BG = (255, 255, 255)
INK = (29, 29, 31)
GRAY = (110, 110, 115)
LIGHT = (134, 134, 139)
RULE = (210, 210, 215)
GREEN = (46, 158, 91)
ORANGE = (217, 122, 36)
GOLD = (232, 184, 0)

F_LIGHT = 'C:/Windows/Fonts/segoeuil.ttf'
F_SEMI = 'C:/Windows/Fonts/segoeuisl.ttf'
F_REG = 'C:/Windows/Fonts/segoeui.ttf'
F_BOLD = 'C:/Windows/Fonts/segoeuib.ttf'

FOOTER = 'wallfin \u00b7 open simulation \u00b7 MuJoCo'


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


def wrap(dr, text, fnt, max_w):
    lines, line = [], ''
    for word in text.split():
        trial = (line + ' ' + word).strip()
        if dr.textlength(trial, font=fnt) <= max_w:
            line = trial
        else:
            if line:
                lines.append(line)
            line = word
    if line:
        lines.append(line)
    return lines


def para(dr, x, y, text, fnt, fill, max_w, lh):
    for i, line in enumerate(wrap(dr, text, fnt, max_w)):
        dr.text((x, y + i * lh), line, font=fnt, fill=fill)


def eyebrow(dr, text):
    tracked(dr, (X, 112), text, font(F_REG, 21), LIGHT)


def footer(dr, text=FOOTER):
    dr.text((X, 970), text, fill=LIGHT, font=font(F_REG, 19))


def save(img, name):
    OUT.mkdir(parents=True, exist_ok=True)
    out = OUT / f'{name}.png'
    img.save(out)
    img.save(REPORTS / f'card_{name}.png')
    return out


def title():
    img = Image.new('RGB', (W, H), BG)
    dr = ImageDraw.Draw(img)
    eyebrow(dr, 'WALLFIN \u00b7 OPEN ROBOTICS STUDY')
    dr.text((X - 6, 232), 'WallFin', fill=INK, font=font(F_LIGHT, 178))
    dr.rectangle([X, 466, X + 64, 471], fill=GOLD)
    dr.text((X, 534), 'A robot that sands drywall with a careful hand.',
            fill=INK, font=font(F_SEMI, 54))
    footer(dr, 'Simulation built with MuJoCo \u00b7 SO-101 arm')
    return save(img, 'launch_01_title')


def what():
    img = Image.new('RGB', (W, H), BG)
    dr = ImageDraw.Draw(img)
    eyebrow(dr, 'WHAT IT IS')
    dr.text((X, 232), 'A small arm, a steady push.', fill=INK,
            font=font(F_LIGHT, 66))
    body = ('WallFin rides a rail along a wall and presses a sanding pad with '
            'a steady five-newton push \u2014 about the weight of a water '
            'bottle resting in your palm.')
    para(dr, X, 360, body, font(F_REG, 33), GRAY, 1010, 52)
    dr.line([1330, 352, 1330, 780], fill=RULE, width=2)
    dr.text((1400, 366), '5 N', fill=INK, font=font(F_LIGHT, 104))
    dr.text((1400, 500), 'steady contact force', fill=GRAY, font=font(F_REG, 27))
    dr.text((1400, 626), '50 Hz', fill=INK, font=font(F_LIGHT, 104))
    dr.text((1400, 760), 'force-control loop', fill=GRAY, font=font(F_REG, 27))
    footer(dr)
    return save(img, 'launch_02_what')


def how():
    img = Image.new('RGB', (W, H), BG)
    dr = ImageDraw.Draw(img)
    eyebrow(dr, 'HOW IT WORKS')
    dr.text((X, 232), 'Force, not position.', fill=INK,
            font=font(F_LIGHT, 66))
    body = ('The arm reads motor load as contact force and adjusts its depth '
            'as it moves. A fixed-depth robot trusts one position instead '
            '\u2014 and loses touch when the wall changes.')
    para(dr, X, 360, body, font(F_REG, 33), GRAY, 1010, 52)
    dr.line([1330, 352, 1330, 780], fill=RULE, width=2)
    f_lbl, f_txt = font(F_REG, 21), font(F_REG, 29)
    dr.ellipse([1400, 384, 1414, 398], fill=GREEN)
    tracked(dr, (1432, 380), 'FORCE LOOP', f_lbl, GRAY)
    dr.text((1400, 424), 'feels the wall', fill=INK, font=f_txt)
    dr.text((1400, 466), 'keeps 5 N', fill=GRAY, font=f_txt)
    dr.ellipse([1400, 596, 1414, 610], fill=ORANGE)
    tracked(dr, (1432, 592), 'FIXED DEPTH', f_lbl, GRAY)
    dr.text((1400, 636), 'trusts a pose', fill=INK, font=f_txt)
    dr.text((1400, 678), 'misses the wall', fill=GRAY, font=f_txt)
    footer(dr)
    return save(img, 'launch_03_how')


def controlled():
    img = Image.new('RGB', (W, H), BG)
    dr = ImageDraw.Draw(img)
    eyebrow(dr, 'THE DEMO \u00b7 1 OF 2')
    dr.text((X, 232), 'Force control on.', fill=INK, font=font(F_LIGHT, 66))
    body = ('Same arm, same wall. The pad feels the surface and holds 5 N '
            'through the seam and over the screw head. The strip along the '
            'bottom shows the force, live.')
    para(dr, X, 360, body, font(F_REG, 33), GRAY, 1180, 52)
    pill(dr, (X, 780), 'CONTROLLED \u00b7 FORCE LOOP \u00b7 5 N TARGET',
         font(F_REG, 27), INK, GREEN)
    footer(dr)
    return save(img, 'launch_04_controlled')


def baseline():
    img = Image.new('RGB', (W, H), BG)
    dr = ImageDraw.Draw(img)
    eyebrow(dr, 'THE DEMO \u00b7 2 OF 2')
    dr.text((X, 232), 'Force control off.', fill=INK,
            font=font(F_LIGHT, 66))
    body = ('Same arm, same wall, same path. The pad now holds a fixed '
            'depth. Watch the seam and the screw head.')
    para(dr, X, 360, body, font(F_REG, 33), GRAY, 1180, 52)
    pill(dr, (X, 780), 'BASELINE \u00b7 RIGID POSITION HOLD',
         font(F_REG, 27), INK, ORANGE)
    footer(dr)
    return save(img, 'launch_05_baseline')


def cta():
    img = Image.new('RGB', (W, H), BG)
    dr = ImageDraw.Draw(img)
    eyebrow(dr, 'WALLFIN \u00b7 OPEN SOURCE')
    dr.text((X, 232), 'Follow the build.', fill=INK, font=font(F_LIGHT, 72))
    dr.rectangle([X, 352, X + 64, 357], fill=GOLD)
    tracked(dr, (X, 420), 'GITHUB', font(F_REG, 20), LIGHT)
    dr.text((X, 452), 'github.com/salikshah/wall-painter-robot',
            fill=INK, font=font(F_SEMI, 46))
    dr.line([X, 566, 1780, 566], fill=RULE, width=2)
    dr.text((X, 620), 'Salik Shah', fill=INK, font=font(F_SEMI, 38))
    dr.text((X, 692), 'X @salik  \u00b7  salik.shah@gmail.com',
            fill=GRAY, font=font(F_REG, 28))
    footer(dr, 'videos \u00b7 simulation \u00b7 hardware guide')
    out = save(img, 'launch_06_cta')
    img.save(OUT / 'cta.png')
    return out


if __name__ == '__main__':
    for fn in (title, what, how, controlled, baseline, cta):
        print(fn())
