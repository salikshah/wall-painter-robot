"""Assemble pitch videos from rendered shot frames.

Grade chain, sliding force strip, macro captions, intro/end cards.
Outputs a 1080p master and a 720p web cut per mode into docs/.
"""
import json
import os
import shutil
import subprocess
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parent.parent
REPORTS = ROOT / 'sim/reports'
BUILD = REPORTS / 'edit_build'
DOCS = ROOT / 'docs'
FPS = 50
W, H = 1920, 1080
STRIP_PX_PER_S = 46
STRIP_H = 118
PLAYHEAD_X = 1880

CAPTIONS = {
    ('controlled', 'macro_touch'): 'first touch \u00b7 5 N target',
    ('controlled', 'macro_seam'): 'crossing the panel seam',
    ('controlled', 'macro_event'): 'clamp bump \u00b7 recovers to 5 N',
    ('baseline', 'macro_touch'): 'first touch \u00b7 rigid position hold',
    ('baseline', 'macro_seam'): 'seam crossing \u00b7 no force feedback',
    ('baseline', 'macro_event'): 'contact lost \u00b7 pad skates past',
}

GRADE = (
    'scale=1920:1080:flags=lanczos,format=yuv420p,'
    "curves=all='0/0 0.4/0.405 0.75/0.775 1/0.99',"
    'colorbalance=rs=-0.005:bs=0.005:rh=0.008:bh=-0.008,'
    'eq=contrast=1.03:saturation=1.03,'
    'split=2[base][bloom],'
    '[bloom]scale=480:270:flags=bilinear,'
    "lutyuv=y='if(gt(val,205),val,0)':u=0:v=0,"
    'gblur=sigma=4,scale=1920:1080:flags=bilinear[bl],'
    '[base][bl]blend=all_mode=screen:all_opacity=0.20,'
    'vignette=angle=PI/7,'
    'unsharp=5:5:0.35'
)


def run(cmd):
    subprocess.run(cmd, check=True)


def ffmpeg():
    return shutil.which('ffmpeg') or 'ffmpeg'


def load_manifest(mode):
    tag = 'v2ctl' if mode == 'controlled' else 'v2base'
    return json.loads((REPORTS / f'shots_{tag}.json').read_text())


def frames_dir(mode):
    tag = 'v2ctl' if mode == 'controlled' else 'v2base'
    return REPORTS / f'frames_{tag}'


def make_strip(mode):
    """Scrolling force chart + static panel, drawn from the canonical npz."""
    tag = 'ctl5' if mode == 'controlled' else 'base5'
    d = np.load(REPORTS / f'{tag}.npz')
    t, f = d['t'], d['force']
    # span the strip to the recorded run, not a hardcoded length, so a change
    # in run duration cannot desync the playhead
    t_end = float(t[-1]) + 0.2
    x0 = 60
    width = x0 + int(t_end * STRIP_PX_PER_S) + 40
    img = Image.new('RGBA', (width, STRIP_H), (0, 0, 0, 0))
    dr = ImageDraw.Draw(img)
    try:
        f_small = ImageFont.truetype('arial.ttf', 13)
    except OSError:
        f_small = ImageFont.load_default()

    def yp(v):
        return int(STRIP_H - 8 - (min(max(v, 0.0), 15.0) / 15.0) * (STRIP_H - 20))

    y5 = yp(5.0)
    for x in range(x0, width - 30, 9):
        dr.line([x, y5, x + 4, y5], fill=(255, 255, 255, 90), width=1)
    dr.text((x0, yp(15.0) - 16), 'N', fill=(200, 208, 218, 160), font=f_small)
    for s in range(5, int(t_end) + 1, 5):
        x = x0 + int(s * STRIP_PX_PER_S)
        dr.line([x, STRIP_H - 8, x, STRIP_H - 3], fill=(255, 255, 255, 70))
        dr.text((x + 3, STRIP_H - 18), f'{s}s', fill=(200, 208, 218, 140),
                font=f_small)
    for i in range(len(t) - 1):
        xa = x0 + int(t[i] * STRIP_PX_PER_S)
        xb = x0 + int(t[i + 1] * STRIP_PX_PER_S)
        fa = float(f[i])
        if fa > 9.0:
            col = (232, 92, 70, 235)
        elif fa < 0.8:
            col = (200, 208, 218, 220)
        else:
            col = (237, 194, 41, 235)
        dr.line([xa, yp(fa), xb, yp(float(f[i + 1]))], fill=col, width=2)
    img.save(BUILD / f'strip_{mode}.png')

    panel = Image.new('RGBA', (W, 150), (0, 0, 0, 0))
    pd = ImageDraw.Draw(panel)
    pd.rounded_rectangle([24, 0, W - 24, 150], radius=14, fill=(14, 18, 26, 200))
    panel.save(BUILD / f'panel_{mode}.png')


def encode_main_segment(mode, f0, f1, out):
    fd = frames_dir(mode) / 'main'
    t0 = f0 / FPS
    xoff = PLAYHEAD_X - (60 + STRIP_PX_PER_S * t0)
    chain = (
        f'[0:v]scale=1920:1080:flags=lanczos,format=rgba[vid];'
        f'[vid][1:v]overlay=x=0:y={H - 178}[v1];'
        f"[v1][2:v]overlay=x='{xoff:.1f}-{STRIP_PX_PER_S}*t':"
        f'y={H - 162}:eval=frame[v2];'
        f'[v2]drawbox=x={PLAYHEAD_X}:y={H - 162}:w=2:h={STRIP_H}:'
        f'color=0xedc229@0.95:t=fill[v3];'
        f'[v3]{GRADE}[vout]'
    )
    cmd = [ffmpeg(), '-y',
           '-framerate', str(FPS), '-start_number', str(f0),
           '-i', str(fd / 'f_%05d.png'),
           '-loop', '1', '-i', str(BUILD / f'panel_{mode}.png'),
           '-loop', '1', '-i', str(BUILD / f'strip_{mode}.png'),
           '-filter_complex', chain, '-map', '[vout]',
           '-frames:v', str(f1 - f0), '-an',
           '-c:v', 'libx264', '-preset', 'medium', '-crf', '17',
           '-pix_fmt', 'yuv420p', str(out)]
    run(cmd)


def encode_caption(mode, shot, count, out):
    fd = frames_dir(mode) / shot
    text = CAPTIONS[(mode, shot)].replace("'", '')
    draw = (f"drawtext=fontfile=caption.ttf:text='{text}':"
            f'fontsize=40:fontcolor=white@0.94:box=1:'
            f'boxcolor=black@0.32:boxborderw=22:x=70:y=h-70-text_h,')
    run([ffmpeg(), '-y', '-framerate', str(FPS), '-i',
         str(fd / 'f_%05d.png'), '-frames:v', str(count), '-vf', draw + GRADE,
         '-an', '-c:v', 'libx264', '-preset', 'medium', '-crf', '17',
         '-pix_fmt', 'yuv420p', str(out)])


def encode_card(kind, mode, seconds, out):
    png = REPORTS / f'card_{kind}_{mode}.png'
    frames = int(round(seconds * FPS))
    run([ffmpeg(), '-y', '-loop', '1', '-framerate', str(FPS), '-i', str(png),
         '-vf', 'scale=1920:1080:flags=lanczos,format=yuv420p',
         '-frames:v', str(frames), '-r', str(FPS), '-an', '-c:v', 'libx264',
         '-preset', 'medium', '-crf', '17', '-pix_fmt', 'yuv420p',
         str(out)])


def assemble(mode):
    man = load_manifest(mode)
    shots = {s['name']: s for s in man['shots']}
    n_main = shots['main']['count']
    segs = []
    cursor = 0

    def fr(x):
        return max(0, min(n_main, int(round(x * FPS))))

    intro = BUILD / f'seg_intro_{mode}.mp4'
    encode_card('intro', mode, 5.4, intro)
    segs.append(intro)
    for name in ('macro_touch', 'macro_seam', 'macro_event'):
        s = shots[name]
        f0 = fr(s['t0'])
        if f0 > cursor:
            out = BUILD / f'seg_{mode}_main_{cursor}_{f0}.mp4'
            encode_main_segment(mode, cursor, f0, out)
            segs.append(out)
        out = BUILD / f'seg_{mode}_{name}.mp4'
        encode_caption(mode, name, s['count'], out)
        segs.append(out)
        cursor = fr(s['t1'])
    if cursor < n_main:
        out = BUILD / f'seg_{mode}_main_{cursor}_{n_main}.mp4'
        encode_main_segment(mode, cursor, n_main, out)
        segs.append(out)
    end = BUILD / f'seg_end_{mode}.mp4'
    encode_card('end', mode, 6.4, end)
    segs.append(end)

    lst = BUILD / f'concat_{mode}.txt'
    lst.write_text(''.join(f"file '{p.as_posix()}'\n" for p in segs),
                   encoding='utf-8')
    master = DOCS / f'{mode}.mp4'
    run([ffmpeg(), '-y', '-f', 'concat', '-safe', '0', '-i', str(lst),
         '-c', 'copy', '-movflags', '+faststart', str(master)])
    web = DOCS / f'{mode}_720p.mp4'
    run([ffmpeg(), '-y', '-i', str(master), '-vf',
         'scale=1280:720:flags=lanczos,unsharp=5:5:0.3,format=yuv420p',
         '-an', '-c:v', 'libx264', '-preset', 'medium', '-crf', '23',
         '-movflags', '+faststart', str(web)])
    print(f'{mode}: {master.stat().st_size / 1e6:.1f} MB  '
          f'{web.stat().st_size / 1e6:.1f} MB')


if __name__ == '__main__':
    BUILD.mkdir(parents=True, exist_ok=True)
    shutil.copy('C:/Windows/Fonts/segoeuib.ttf', BUILD / 'caption.ttf')
    os.chdir(BUILD)
    for mode in ('controlled', 'baseline'):
        make_strip(mode)
        assemble(mode)
    print('edit done')
