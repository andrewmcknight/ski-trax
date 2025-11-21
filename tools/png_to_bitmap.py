"""
- Converts PNG images (with alpha) into RGB565 bitmaps declared in .h files
- Transparent pixels (alpha = 0) are replaced with magenta key 0xF81F
- All opaque pixels use true RGB565 conversion
"""

import os
from PIL import Image

ENABLE_MAGENTA_MASK = False

INPUT_DIR = "images/icons/"
OUTPUT_DIR = "code/main/src/assets/icons/"

# Magenta key for transparency (RGB888 → RGB565)
MAGENTA_RGB565 = 0xF81F  # 11111 000000 11111

os.makedirs(OUTPUT_DIR, exist_ok=True)

def rgb888_to_rgb565(r, g, b):
    """Convert 24-bit RGB to 16-bit RGB565."""
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

if __name__ == "__main__":
    for filename in os.listdir(INPUT_DIR):
        if not filename.lower().endswith(".png"):
            continue

        path = os.path.join(INPUT_DIR, filename)

        # IMPORTANT: Load with alpha preserved
        img = Image.open(path).convert("RGBA")
        w, h = img.size
        pixels = img.load()

        array_name = os.path.splitext(filename)[0]
        out_path = os.path.join(OUTPUT_DIR, array_name + ".h")

        with open(out_path, "w") as f:
            f.write(f"// Converted from {filename}\n")
            f.write(f"// Size: {w} x {h}\n")
            f.write(f"// Transparent pixels converted to magenta key 0xF81F\n\n")
            f.write(f"const uint16_t {array_name}[{w*h}] PROGMEM = {{\n")

            count = 0
            for y in range(h):
                for x in range(w):
                    r, g, b, a = pixels[x, y]

                    # --- Transparency mapping ---
                    if a == 0 and ENABLE_MAGENTA_MASK:
                        rgb565 = MAGENTA_RGB565
                    else:
                        rgb565 = rgb888_to_rgb565(r, g, b)

                    f.write(f"0x{rgb565:04X},")
                    count += 1

                    if count % 12 == 0:
                        f.write("\n")
                f.write("\n")

            f.write("};\n")

        print(f"Converted {filename} -> {out_path}")

    print("\nDone!")
