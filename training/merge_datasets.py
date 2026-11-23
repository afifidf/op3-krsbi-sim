#!/usr/bin/env python3
"""Gabungkan beberapa dataset YOLO ke satu folder dengan daftar kelas dataset_gen.

Tiap sumber ditulis DIR[:MAP], MAP = pasangan kelas_lama=kelas_baru dipisah koma.
Kelas yang tidak ada di MAP dibuang. Tanpa MAP, indeks kelas dipakai apa adanya.

  python3 merge_datasets.py --out gabungan \\
      --src ~/datasets/bola_sim \\
      --src ~/datasets/gkeep_sementara:2=0         # dataset tim: GK, Robot, bola -> bola

Gambar di-hardlink (atau disalin kalau beda filesystem). Nama file diberi awalan nama
sumber supaya tidak bentrok. Struktur sumber: images/{train,val} dan labels/{train,val}.
"""
import argparse
import os
import shutil

CLASSES = ['bola', 'tiang_gawang', 'titik_penalti', 'titik_tengah', 'sudut_lapangan', 'robot']
IMAGE_EXT = ('.jpg', '.jpeg', '.png', '.bmp')


def parse_source(text):
    path, _, mapping = text.partition(':')
    remap = None
    if mapping:
        remap = {int(a): int(b) for a, b in (pair.split('=') for pair in mapping.split(','))}
    return os.path.expanduser(path), remap


def link(src, dst):
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--out', required=True)
    parser.add_argument('--src', action='append', required=True, help='DIR[:lama=baru,...]')
    args = parser.parse_args()
    out = os.path.abspath(os.path.expanduser(args.out))
    counts = [0] * len(CLASSES)
    for text in args.src:
        root, remap = parse_source(text)
        prefix = os.path.basename(os.path.normpath(root))
        n_img = 0
        for split in ('train', 'val'):
            img_dir = os.path.join(root, 'images', split)
            if not os.path.isdir(img_dir):
                continue
            os.makedirs(os.path.join(out, 'images', split), exist_ok=True)
            os.makedirs(os.path.join(out, 'labels', split), exist_ok=True)
            for name in sorted(os.listdir(img_dir)):
                stem, ext = os.path.splitext(name)
                if ext.lower() not in IMAGE_EXT:
                    continue
                lines = []
                label = os.path.join(root, 'labels', split, stem + '.txt')
                if os.path.exists(label):
                    for line in open(label):
                        parts = line.split()
                        if len(parts) != 5:
                            continue
                        cls = int(parts[0])
                        if remap is not None:
                            if cls not in remap:
                                continue
                            cls = remap[cls]
                        if not 0 <= cls < len(CLASSES):
                            raise SystemExit(f'{label}: kelas {cls} di luar daftar, beri MAP')
                        counts[cls] += 1
                        lines.append(' '.join([str(cls)] + parts[1:]))
                new = f'{prefix}_{stem}'
                link(os.path.join(img_dir, name), os.path.join(out, 'images', split, new + ext))
                with open(os.path.join(out, 'labels', split, new + '.txt'), 'w') as f:
                    f.write(''.join(line + '\n' for line in lines))
                n_img += 1
        print(f'{root}: {n_img} gambar')
    names = ''.join(f'  {i}: {name}\n' for i, name in enumerate(CLASSES))
    with open(os.path.join(out, 'data.yaml'), 'w') as f:
        f.write(f'path: {out}\ntrain: images/train\nval: images/val\nnames:\n{names}')
    print('Label per kelas: ' + ', '.join(f'{n} {c}' for n, c in zip(CLASSES, counts)))


if __name__ == '__main__':
    main()
