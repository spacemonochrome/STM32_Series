from PIL import Image, ImageSequence
import os
import struct

gif_path = "image.gif"
base_output = r"C:\Users\hfk47\Desktop\gif_frames_bmp"
os.makedirs(base_output, exist_ok=True)

# Alt klasörler
folders = {
    "32bit": os.path.join(base_output, "bmp32"),
    "24bit": os.path.join(base_output, "bmp24"),
    "16bit": os.path.join(base_output, "bmp16"),
    "8bit":  os.path.join(base_output, "bmp8"),
    "1bit":  os.path.join(base_output, "bmp1"),
}
for f in folders.values():
    os.makedirs(f, exist_ok=True)

def pad_row(data, row_size):
    """BMP satırlarını 4 byte hizala"""
    padding = (4 - (len(data) % 4)) % 4
    return data + b'\x00' * padding

def rgb888_to_rgb565(r, g, b):
    r5 = (r * 31) // 255
    g6 = (g * 63) // 255
    b5 = (b * 31) // 255
    return (r5 << 11) | (g6 << 5) | b5

def create_bmp(width, height, pixel_data, bit_count, palette=None, masks=None):
    # File header
    bfType = b'BM'
    bfReserved1 = 0
    bfReserved2 = 0
    bfOffBits = 14 + 40
    if masks:
        bfOffBits += 12
    if palette:
        bfOffBits += len(palette)

    bfSize = bfOffBits + len(pixel_data)
    file_header = struct.pack('<2sIHHI', bfType, bfSize, bfReserved1, bfReserved2, bfOffBits)

    # Info header
    biSize = 40
    biWidth = width
    biHeight = height
    biPlanes = 1
    biBitCount = bit_count
    biCompression = 0 if not masks else 3  # BI_RGB or BI_BITFIELDS
    biSizeImage = len(pixel_data)
    biXPelsPerMeter = 0
    biYPelsPerMeter = 0
    biClrUsed = 0
    biClrImportant = 0

    info_header = struct.pack('<IIIHHIIIIII', biSize, biWidth, biHeight, biPlanes,
                              biBitCount, biCompression, biSizeImage,
                              biXPelsPerMeter, biYPelsPerMeter, biClrUsed, biClrImportant)

    parts = [file_header, info_header]
    if masks:
        parts.append(struct.pack('<III', *masks))
    if palette:
        parts.append(palette)
    parts.append(pixel_data)
    return b''.join(parts)

gif = Image.open(gif_path)

for i, frame in enumerate(ImageSequence.Iterator(gif)):
    frame_resized = frame.resize((320, 240), Image.LANCZOS)
    frame_rgb = frame_resized.convert("RGB")
    width, height = frame_rgb.size

    # --- 32-bit BMP (XRGB8888) ---
    pixel_data_32 = bytearray()
    for y in reversed(range(height)):
        for x in range(width):
            r, g, b = frame_rgb.getpixel((x, y))
            a = 255  # XRGB8888: alfa hep 255
            pixel_data_32 += struct.pack('<BBBB', b, g, r, a)
    bmp32 = create_bmp(width, height, pixel_data_32, 32)
    with open(os.path.join(folders["32bit"], f"frame_{i:03d}.bmp"), "wb") as f:
        f.write(bmp32)

    # --- 24-bit BMP (RGB888) ---
    pixel_data_24 = bytearray()
    for y in reversed(range(height)):
        row = bytearray()
        for x in range(width):
            r, g, b = frame_rgb.getpixel((x, y))
            row += struct.pack('<BBB', b, g, r)  # BMP = BGR
        pixel_data_24 += pad_row(row, width*3)
    bmp24 = create_bmp(width, height, pixel_data_24, 24)
    with open(os.path.join(folders["24bit"], f"frame_{i:03d}.bmp"), "wb") as f:
        f.write(bmp24)

    # --- 16-bit BMP (RGB565) ---
    pixel_data_16 = bytearray()
    for y in reversed(range(height)):
        row = bytearray()
        for x in range(width):
            r, g, b = frame_rgb.getpixel((x, y))
            rgb565 = rgb888_to_rgb565(r, g, b)
            row += struct.pack('<H', rgb565)
        pixel_data_16 += pad_row(row, width*2)
    masks = (0xF800, 0x07E0, 0x001F)
    bmp16 = create_bmp(width, height, pixel_data_16, 16, masks=masks)
    with open(os.path.join(folders["16bit"], f"frame_{i:03d}.bmp"), "wb") as f:
        f.write(bmp16)

    # --- 8-bit BMP (grayscale) ---
    frame_gray = frame_rgb.convert("L")
    palette = bytearray()
    for v in range(256):
        palette += struct.pack('<BBBB', v, v, v, 0)  # BGRA
    pixel_data_8 = bytearray()
    for y in reversed(range(height)):
        row = bytearray()
        for x in range(width):
            gray = frame_gray.getpixel((x, y))
            row.append(gray)
        pixel_data_8 += pad_row(row, width)
    bmp8 = create_bmp(width, height, pixel_data_8, 8, palette=palette)
    with open(os.path.join(folders["8bit"], f"frame_{i:03d}.bmp"), "wb") as f:
        f.write(bmp8)

    # --- 1-bit BMP (siyah/beyaz) ---
    frame_bw = frame_rgb.convert("1")
    palette_bw = struct.pack('<BBBBBBBB', 0, 0, 0, 0, 255, 255, 255, 0)
    pixel_data_1 = bytearray()
    for y in reversed(range(height)):
        row = bytearray()
        bit_buffer = 0
        bit_count = 0
        for x in range(width):
            bit = 0 if frame_bw.getpixel((x, y)) == 0 else 1
            bit_buffer = (bit_buffer << 1) | bit
            bit_count += 1
            if bit_count == 8:
                row.append(bit_buffer)
                bit_buffer = 0
                bit_count = 0
        if bit_count > 0:
            row.append(bit_buffer << (8-bit_count))
        pixel_data_1 += pad_row(row, (width+7)//8)
    bmp1 = create_bmp(width, height, pixel_data_1, 1, palette=palette_bw)
    with open(os.path.join(folders["1bit"], f"frame_{i:03d}.bmp"), "wb") as f:
        f.write(bmp1)

print(f"{i+1} kare işlendi. Çıkış klasörü: {base_output}")
