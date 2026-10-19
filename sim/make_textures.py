"""Procedural studio textures for the demo scene (visual only, self-generated).

Writes assets/wall_tex.png, assets/grit_tex.png, assets/floor_tex.png and a
debug quadrants marker floor_marker.png used to verify plane UV mapping.
"""
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter

ASSETS = Path(__file__).resolve().parent.parent / 'assets'


def periodic_noise(size, rng, cutoff):
    f = rng.standard_normal((size, size))
    F = np.fft.fft2(f)
    fy = np.fft.fftfreq(size)[:, None]
    fx = np.fft.fftfreq(size)[None, :]
    r = np.sqrt(fx ** 2 + fy ** 2)
    F *= np.exp(-(r / cutoff) ** 2)
    n = np.real(np.fft.ifft2(F))
    return n / (np.abs(n).max() + 1e-9)


def make_wall():
    rng = np.random.default_rng(11)
    n = 512
    mottle = periodic_noise(n, rng, 0.014) * 5.0
    tooth = periodic_noise(n, rng, 0.10) * 0.8
    row = np.linspace(0, 2 * np.pi * 5, n)[None, :]
    streaks = np.sin(row) * 1.3 + np.sin(row * 0.37 + 1.1) * 1.0
    base = 234 + mottle + tooth + streaks
    img = Image.fromarray(np.clip(base, 0, 255).astype(np.uint8)).convert('RGB')
    img = img.filter(ImageFilter.GaussianBlur(2.0))
    img.save(ASSETS / 'wall_tex.png')


def make_grit():
    rng = np.random.default_rng(23)
    n = 256
    speck = rng.uniform(-34, 34, (n, n))
    pits = rng.random((n, n)) < 0.030
    base = 198 + speck - pits * 46.0
    img = Image.fromarray(np.clip(base, 0, 255).astype(np.uint8))
    img = img.filter(ImageFilter.GaussianBlur(0.4)).convert('RGB')
    img.save(ASSETS / 'grit_tex.png')


def make_floor(size_m=8.0, px=1024):
    rng = np.random.default_rng(37)
    idx = np.arange(px)
    xs = (idx / (px - 1) - 0.5) * size_m
    ys = (idx / (px - 1) - 0.5) * size_m
    X, Y = np.meshgrid(xs, ys)
    mottle = periodic_noise(px, rng, 0.010) * 3.2 + periodic_noise(px, rng, 0.05) * 1.4
    ao = np.ones_like(X)

    def blob(cx, cy, sx, sy, strength):
        return 1.0 - (1.0 - strength) * np.exp(-(((X - cx) / sx) ** 2 +
                                                ((Y - cy) / sy) ** 2))

    ao *= blob(0.0, 0.0, 0.50, 0.26, 0.90)
    wall_band = np.exp(-((Y + 0.27) / 0.055) ** 2) * np.clip(1 - (np.abs(X) / 0.40) ** 6, 0, 1)
    ao *= 1.0 - 0.16 * wall_band
    rail_line = np.exp(-(Y / 0.10) ** 2) * np.clip(1 - (np.abs(X) / 0.42) ** 6, 0, 1)
    ao *= 1.0 - 0.07 * rail_line
    for cx, cy in [(-0.31, 0.0), (0.23, 0.0), (-0.30, -0.27), (0.30, -0.27)]:
        ao *= blob(cx, cy, 0.085, 0.085, 0.80)
    ao *= blob(0.0, -0.10, 1.5, 1.1, 0.965)
    val = 228 * ao + mottle
    img = Image.fromarray(np.clip(val, 0, 255).astype(np.uint8)).convert('RGB')
    img.save(ASSETS / 'floor_tex.png')


def make_marker():
    n = 512
    img = np.zeros((n, n, 3), dtype=np.uint8)
    img[:, :, :] = (40, 40, 40)
    img[0:n // 2, 0:n // 2] = (200, 60, 60)
    img[0:n // 2, n // 2:] = (60, 200, 60)
    img[n // 2:, 0:n // 2] = (60, 60, 200)
    img[n // 2:, n // 2:] = (200, 200, 60)
    img[n // 2 - 4:n // 2 + 4, :] = (255, 255, 255)
    img[:, n // 2 - 4:n // 2 + 4] = (255, 255, 255)
    Image.fromarray(img).save(ASSETS / 'floor_marker.png')


if __name__ == '__main__':
    ASSETS.mkdir(exist_ok=True)
    make_wall()
    make_grit()
    make_floor()
    make_marker()
    print('textures written to', ASSETS)
