#!/usr/bin/env python3
"""
generate_emoji_sprites.py — Render emoji to ESP32 flash data.

Usage:
    python3 generate_emoji_sprites.py --size 24 --set common --preview
    python3 generate_emoji_sprites.py --size 24 --set full --header-only

Sets:
    minimal  — ~80 emoji (Meshtastic essentials)
    common   — ~500 emoji (covers most chat usage)
    full     — ~1400+ base emoji (all standard, no skin tones/flags)

Default output: emoji_sprites.h + emoji_sprites.bin (binary blob)
With --header-only: single emoji_sprites.h (slower compile for large sets)

For binary blob, add to your CMakeLists.txt:
    target_add_binary_data(${COMPONENT_LIB} "emoji_sprites.bin" BINARY)

Requirements: pip install Pillow
"""
import argparse, struct, sys, os
from pathlib import Path
try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    print("ERROR: pip install Pillow"); sys.exit(1)

EMOJI_MINIMAL = [
    0x1F600,0x1F602,0x1F605,0x1F607,0x1F608,0x1F60D,0x1F60E,0x1F610,
    0x1F614,0x1F61B,0x1F62D,0x1F631,0x1F634,0x1F644,0x1F914,0x1F923,
    0x1F62C,0x1F92F,0x1F973,0x1FAE1,0x1F920,0x1F60F,0x1F62E,0x1F633,
    0x1F641,0x1F642,0x1F643,0x1F648,
    0x1F44D,0x1F44E,0x1F44B,0x1F44F,0x1F919,0x1F91D,0x1F91E,0x1F64F,
    0x1F4AA,0x270C,
    0x2764,0x1F499,0x1F49A,0x1F525,0x2B50,0x2705,0x274C,0x26A0,
    0x1F480,0x1F4AF,
    0x1F332,0x1F308,0x2600,0x1F319,0x26C8,0x2744,0x1F30A,
    0x1F4E1,0x1F50B,0x1F4F6,0x1F6F0,0x1F4CD,0x1F3E0,0x1F697,0x1F681,
    0x2708,0x1F680,0x26F5,0x1F6B2,
    0x1F415,0x1F408,0x1F985,0x1F43B,
    0x1F37A,0x2615,0x1F32E,0x1F355,0x1F354,
    0x1F3B5,0x1F3AE,0x1F511,0x1F514,0x1F4E2,0x1F310,0x1F198,0x267B,
]

def get_common():
    s = set(EMOJI_MINIMAL)
    s.update(range(0x1F600,0x1F650))  # Smileys
    s.update(range(0x1F910,0x1F930))  # More faces
    s.update([0x1F970,0x1F971,0x1F972,0x1F973,0x1F974,0x1F975,0x1F976,
              0x1F978,0x1F979,0x1F97A,0x1FAE0,0x1FAE1,0x1FAE2,0x1FAE3,0x1FAE4,0x1FAE5])
    s.update(range(0x1F440,0x1F463))  # Body parts / gestures
    s.update([0x270A,0x270B,0x270C,0x270D,0x1F590,0x1F595,0x1F596])
    s.update([0x2764,0x1F493,0x1F494,0x1F495,0x1F496,0x1F497,0x1F498,0x1F499,
              0x1F49A,0x1F49B,0x1F49C,0x1F49D,0x1F49E,0x1F49F,0x1F5A4,0x1F90D,0x1F90E])
    s.update(range(0x1F400,0x1F440))  # Animals
    s.update(range(0x1F980,0x1F9A3))  # More animals
    s.update(range(0x1F345,0x1F380))  # Food
    s.update(range(0x1F32D,0x1F345))  # More food/nature
    s.update(range(0x1F680,0x1F6A3))  # Transport
    s.update([0x1F6B2,0x1F6F0,0x1F6F3,0x1F6F4,0x1F6F5,0x1F6F6,0x1F6F7,0x1F6F8,0x1F6F9])
    s.update([0x2600,0x2601,0x2602,0x2603,0x2604,0x2614,0x26A1,0x26C4,0x26C5,0x26C8,0x2728,0x2744])
    s.update(range(0x1F300,0x1F322))  # Weather/celestial
    s.update(range(0x1F4A0,0x1F4B0))  # Symbols
    s.update(range(0x1F4BB,0x1F4CA))  # Tech objects
    s.update(range(0x1F4CD,0x1F500))  # More objects
    s.update(range(0x1F500,0x1F532))  # UI symbols
    s.update(range(0x1F3A0,0x1F3D4))  # Activities
    s.update(range(0x1F3D4,0x1F3F1))  # Buildings
    s.update(range(0x1F5E1,0x1F600))  # Misc objects
    s.update([0x2049,0x2122,0x2139,0x231A,0x231B,0x23E9,0x23EA,0x23F0,0x23F3,
              0x25AA,0x25AB,0x25B6,0x25C0,0x25FB,0x25FC,0x25FD,0x25FE,
              0x2611,0x2614,0x2615,0x2620,0x2622,0x2623,0x2626,0x262A,0x262E,0x262F,
              0x2638,0x2639,0x263A,0x2648,0x2649,0x264A,0x264B,0x264C,0x264D,0x264E,
              0x264F,0x2650,0x2651,0x2652,0x2653,0x2660,0x2663,0x2665,0x2666,0x2668,
              0x267B,0x267E,0x267F,0x2692,0x2693,0x2694,0x2695,0x2696,0x2697,0x2699,
              0x269B,0x269C,0x26A0,0x26AA,0x26AB,0x26BD,0x26BE,0x26CE,0x26D4,0x26EA,
              0x26F0,0x26F1,0x26F2,0x26F3,0x26F5,0x26F7,0x26F8,0x26F9,0x26FA,0x26FD,
              0x2702,0x2705,0x2708,0x2709,0x270F,0x2712,0x2714,0x2716,0x2728,0x2733,
              0x2734,0x2744,0x2747,0x274C,0x274E,0x2753,0x2754,0x2755,0x2757,0x2763,
              0x2764,0x2795,0x2796,0x2797,0x27A1,0x2934,0x2935,0x2B05,0x2B06,0x2B07,
              0x2B1B,0x2B1C,0x2B50,0x2B55,
              0x1F004,0x1F170,0x1F171,0x1F17E,0x1F17F,0x1F18E,
              0x1F191,0x1F192,0x1F193,0x1F194,0x1F195,0x1F196,0x1F197,0x1F198,0x1F199,0x1F19A])
    s.update(range(0x1F6A8,0x1F6C6))  # Signs/warnings
    s.update([0x1F6D0,0x1F6D1,0x1F6D2,0x1F6D5])
    return sorted(s)

def get_full():
    s = set(get_common())
    s.update(range(0x1F900,0x1F9FF))  # Supplemental Symbols and Pictographs
    s.update(range(0x1FA70,0x1FAFF))  # Symbols Extended-A
    s.update(range(0x1F680,0x1F700))  # Transport & Map complete
    return sorted(s)

def find_font(size):
    if sys.platform == "darwin":
        paths = ["/System/Library/Fonts/Apple Color Emoji.ttc"]
    else:
        paths = [
            "/usr/share/fonts/truetype/noto/NotoColorEmoji.ttf",
            "/usr/share/fonts/noto-cjk/NotoColorEmoji.ttf",
            "/usr/share/fonts/google-noto-color-emoji-fonts/NotoColorEmoji.ttf",
            "/usr/share/fonts/truetype/noto-color-emoji/NotoColorEmoji.ttf",
        ]
    for p in paths:
        if os.path.exists(p):
            try:
                f = ImageFont.truetype(p, size)
                print(f"Using font: {p}")
                return f
            except: pass
    print("WARNING: No emoji font found")
    return ImageFont.load_default()

def rgb565(r,g,b): return ((r>>3)<<11)|((g>>2)<<5)|(b>>3)

def render(font, cp, size):
    ch = chr(cp)
    rs = size*4
    img = Image.new("RGBA",(rs,rs),(0,0,0,0))
    d = ImageDraw.Draw(img)
    try: bb = d.textbbox((0,0),ch,font=font)
    except: return None
    tw,th = bb[2]-bb[0], bb[3]-bb[1]
    if tw==0 or th==0: return None
    x,y = (rs-tw)//2-bb[0], (rs-th)//2-bb[1]
    try: d.text((x,y),ch,font=font,embedded_color=True)
    except: return None
    cb = img.getbbox()
    if not cb: return None
    img = img.crop(cb)
    w,h = img.size
    if w==0 or h==0: return None
    sc = min(size/w, size/h)
    nw,nh = max(1,int(w*sc)), max(1,int(h*sc))
    img = img.resize((nw,nh), Image.LANCZOS)
    if sum(px[3] for px in img.getdata()) < 10: return None
    return img

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size",type=int,default=24)
    ap.add_argument("--set",choices=["minimal","common","full"],default="common")
    ap.add_argument("--output",default="emoji_sprites")
    ap.add_argument("--preview",action="store_true")
    ap.add_argument("--header-only",action="store_true")
    args = ap.parse_args()

    cps = {"minimal":sorted(set(EMOJI_MINIMAL)),"common":get_common(),"full":get_full()}[args.set]
    print(f"Rendering {len(cps)} candidates at {args.size}x{args.size} ({args.set})...")
    font = find_font(args.size*4)

    sprites = []
    fail = 0
    for i,cp in enumerate(cps):
        if (i+1)%100==0: print(f"  {i+1}/{len(cps)}...")
        img = render(font, cp, args.size)
        if not img: fail+=1; continue
        w,h = img.size
        pxs = list(img.getdata())
        r5 = [rgb565(r,g,b) for r,g,b,a in pxs]
        al = [a for _,_,_,a in pxs]
        sprites.append((cp,w,h,r5,al))

    sprites.sort(key=lambda x:x[0])
    print(f"Rendered {len(sprites)} emoji ({fail} skipped)")

    if args.header_only:
        gen_header(sprites, args.output+".h")
    else:
        gen_binary(sprites, args.output)

    if args.preview:
        gen_preview(sprites, args.size, args.output+"_preview.png")

    tot = sum(w*h*3 for _,w,h,_,_ in sprites)
    print(f"\n  {len(sprites)} emoji, ~{tot/1024:.1f} KB flash")

def gen_binary(sprites, base):
    bd = bytearray()
    idx = []
    for cp,w,h,r5,al in sprites:
        off = len(bd)
        for v in r5: bd += struct.pack("<H",v)
        for v in al: bd += struct.pack("B",v)
        idx.append((cp,w,h,off,w*h))
    with open(base+".bin","wb") as f: f.write(bd)
    with open(base+".h","w") as f:
        f.write(f"// Auto-generated — {len(sprites)} emoji, data in emoji_sprites.bin\n")
        f.write(f"// Binary size: {len(bd)} bytes ({len(bd)/1024:.1f} KB)\n")
        f.write("#pragma once\n#include <stdint.h>\n#define EMOJI_BINARY_BLOB 1\n\n")
        f.write("extern const uint8_t emoji_bin_start[] asm(\"_binary_emoji_sprites_bin_start\");\n")
        f.write("extern const uint8_t emoji_bin_end[] asm(\"_binary_emoji_sprites_bin_end\");\n\n")
        f.write("typedef struct {\n    uint32_t codepoint;\n    uint8_t w,h;\n")
        f.write("    uint32_t offset;\n    uint16_t pixels;\n} emoji_sprite_t;\n\n")
        f.write(f"static const emoji_sprite_t emoji_sprites[{len(idx)}] = {{\n")
        for cp,w,h,off,px in idx:
            f.write(f"    {{0x{cp:04X},{w},{h},{off},{px}}},\n")
        f.write(f"}};\nstatic const int emoji_sprite_count = {len(idx)};\n\n")
        f.write("static inline const emoji_sprite_t *emoji_find(uint32_t cp) {\n")
        f.write("    int lo=0,hi=emoji_sprite_count-1;\n    while(lo<=hi){\n")
        f.write("        int m=(lo+hi)/2;\n        if(emoji_sprites[m].codepoint==cp) return &emoji_sprites[m];\n")
        f.write("        if(emoji_sprites[m].codepoint<cp) lo=m+1; else hi=m-1;\n    }\n    return 0;\n}\n\n")
        f.write("static inline const uint16_t *emoji_rgb565(const emoji_sprite_t *e) {\n")
        f.write("    return (const uint16_t*)(emoji_bin_start + e->offset);\n}\n")
        f.write("static inline const uint8_t *emoji_alpha(const emoji_sprite_t *e) {\n")
        f.write("    return emoji_bin_start + e->offset + e->pixels*2;\n}\n")
    print(f"Generated {base}.bin ({len(bd)}B) + {base}.h")

def gen_header(sprites, path):
    with open(path,"w") as f:
        f.write(f"// Auto-generated — {len(sprites)} emoji, RGB565+alpha\n")
        f.write("#pragma once\n#include <stdint.h>\n\n")
        for cp,w,h,r5,al in sprites:
            n=f"emoji_{cp:04X}"
            f.write(f"// U+{cp:04X} {w}x{h}\n")
            f.write(f"static const uint16_t {n}_rgb565[{len(r5)}]={{")
            f.write(",".join(f"0x{v:04X}" for v in r5))
            f.write(f"}};\nstatic const uint8_t {n}_alpha[{len(al)}]={{")
            f.write(",".join(f"0x{v:02X}" for v in al))
            f.write("};\n\n")
        f.write("typedef struct {\n    uint32_t codepoint;\n    uint8_t w,h;\n")
        f.write("    const uint16_t *rgb565;\n    const uint8_t *alpha;\n} emoji_sprite_t;\n\n")
        f.write(f"static const emoji_sprite_t emoji_sprites[{len(sprites)}] = {{\n")
        for cp,w,h,_,_ in sprites:
            f.write(f"    {{0x{cp:04X},{w},{h},emoji_{cp:04X}_rgb565,emoji_{cp:04X}_alpha}},\n")
        f.write(f"}};\nstatic const int emoji_sprite_count = {len(sprites)};\n\n")
        f.write("static inline const emoji_sprite_t *emoji_find(uint32_t cp) {\n")
        f.write("    int lo=0,hi=emoji_sprite_count-1;\n    while(lo<=hi){\n")
        f.write("        int m=(lo+hi)/2;\n        if(emoji_sprites[m].codepoint==cp) return &emoji_sprites[m];\n")
        f.write("        if(emoji_sprites[m].codepoint<cp) lo=m+1; else hi=m-1;\n    }\n    return 0;\n}\n")
    tot = sum(w*h*3 for _,w,h,_,_ in sprites)
    print(f"Generated {path}: {len(sprites)} emoji, ~{tot/1024:.1f} KB")

def gen_preview(sprites, size, path):
    cols=20; rows=(len(sprites)+cols-1)//cols; cell=size+4
    img = Image.new("RGBA",(cols*cell,rows*cell),(10,14,20,255))
    for i,(_,w,h,r5,al) in enumerate(sprites):
        sp = Image.new("RGBA",(w,h))
        for y in range(h):
            for x in range(w):
                idx=y*w+x; v=r5[idx]
                sp.putpixel((x,y),(((v>>11)&0x1F)<<3,((v>>5)&0x3F)<<2,(v&0x1F)<<3,al[idx]))
        img.paste(sp,(i%cols*cell+2,i//cols*cell+2),sp)
    img.save(path); print(f"Preview: {path}")

if __name__=="__main__": main()
