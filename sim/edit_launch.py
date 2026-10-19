"""Assemble the combined WallFin film: intro + both demos + CTA."""
import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
REPORTS = ROOT / 'sim/reports'
BUILD = REPORTS / 'edit_build'
DOCS = ROOT / 'docs'
FPS = 50

CARDS = [
    ('launch_01_title', 3.6),
    ('launch_02_what', 5.2),
    ('launch_03_how', 5.4),
    ('launch_04_controlled', 3.2),
    ('launch_05_baseline', 4.0),
    ('launch_06_cta', 7.0),
]

CONTROLLED_BODY = [
    'seg_controlled_main_0_145.mp4',
    'seg_controlled_macro_touch.mp4',
    'seg_controlled_main_320_369.mp4',
    'seg_controlled_macro_seam.mp4',
    'seg_controlled_main_519_1199.mp4',
    'seg_controlled_macro_event.mp4',
    'seg_controlled_main_1349_1950.mp4',
]

BASELINE_BODY = [
    'seg_baseline_main_0_144.mp4',
    'seg_baseline_macro_touch.mp4',
    'seg_baseline_main_319_414.mp4',
    'seg_baseline_macro_seam.mp4',
    'seg_baseline_main_564_1753.mp4',
    'seg_baseline_macro_event.mp4',
    'seg_baseline_main_1903_1950.mp4',
]


def run(cmd):
    subprocess.run(cmd, check=True)


def ffmpeg():
    return shutil.which('ffmpeg') or 'ffmpeg'


def encode_card(name, seconds):
    png = REPORTS / f'card_{name}.png'
    out = BUILD / f'seg_{name}.mp4'
    frames = int(round(seconds * FPS))
    run([ffmpeg(), '-y', '-loop', '1', '-framerate', str(FPS), '-i', str(png),
         '-vf', 'scale=1920:1080:flags=lanczos,format=yuv420p',
         '-frames:v', str(frames), '-r', str(FPS), '-an', '-c:v', 'libx264',
         '-preset', 'medium', '-crf', '17', '-pix_fmt', 'yuv420p', str(out)])
    return out


def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    segs = {}
    for name, seconds in CARDS:
        segs[name] = encode_card(name, seconds)
        print(f'{name}: {seconds:.1f}s')

    order = [segs['launch_01_title'], segs['launch_02_what'],
             segs['launch_03_how'], segs['launch_04_controlled']]
    order += [BUILD / f for f in CONTROLLED_BODY]
    order.append(segs['launch_05_baseline'])
    order += [BUILD / f for f in BASELINE_BODY]
    order.append(segs['launch_06_cta'])

    missing = [p for p in order if not p.exists()]
    if missing:
        raise SystemExit(f'missing segments: {missing}')

    lst = BUILD / 'concat_launch.txt'
    lst.write_text(''.join(f"file '{p.as_posix()}'\n" for p in order),
                   encoding='utf-8')
    master = DOCS / 'wallfin_film.mp4'
    run([ffmpeg(), '-y', '-f', 'concat', '-safe', '0', '-i', str(lst),
         '-c', 'copy', '-movflags', '+faststart', str(master)])
    web = DOCS / 'wallfin_film_720p.mp4'
    run([ffmpeg(), '-y', '-i', str(master), '-vf',
         'scale=1280:720:flags=lanczos,unsharp=5:5:0.3,format=yuv420p',
         '-an', '-c:v', 'libx264', '-preset', 'medium', '-crf', '23',
         '-movflags', '+faststart', str(web)])
    print(f'film: {master.stat().st_size / 1e6:.1f} MB  '
          f'{web.stat().st_size / 1e6:.1f} MB')


if __name__ == '__main__':
    main()
