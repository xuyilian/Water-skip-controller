"""Chapter 5 Figure 5.3: put timestamps on the hop composite.

The review asked for time stamps. The composite is a lighten blend of nine
frames of C0284_stable2hop_propflyoff.MP4 (25 fps), frames 356 to 388 every
fourth frame, so 160 ms apart, with water contact at frame 374.

The exposure positions were not guessed. The nine source frames were pulled
from the video, the vehicle located in each as the largest bright connected
region, and the composite registered against the video by searching the crop
offset that maximises correlation: the composite is a 1800x1570 crop at native
resolution with its origin at (999, 430). That registration predicts frame
368 at y = 527 and the composite has a blob centroid at y = 527.

Three exposures coincide at each end because the vehicle is still hovering
before release and is near the top of its climb afterwards, so those are
labelled as groups rather than pretending to nine separate positions.

    python fig_composite_annotate.py
"""
import os
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ASSETS = os.path.join(HERE, "..", "..", "paper", "CityUHKThesis-main", "Assets")
SRC = os.path.join(ASSETS, "ch5fig5_composite.jpg")
OUT = os.path.join(ASSETS, "ch5fig5_composite_annotated.jpg")

# Exposure rows measured directly in the composite, not transferred from the
# video: the vehicle carries saturated red/orange wiring while the spray is
# grey, so a saturation profile down the vehicle's column separates them.
# Three groups resolve. The remaining exposures pile up at the two turning
# points, where the vehicle is barely moving between frames, and inside the
# spray; those are bracketed rather than given false individual positions.
MARKS = [
    (880,  307, "−720, −560, −400 ms"),
    (880,  548, "−240 ms"),
    (880,  858, "+240, +400, +560 ms"),
]
BRACKET = (1010, 1180, "−80, +80 ms\n(within the spray)")
LABEL_X = 1210
WATER_Y = 1250

im = Image.open(SRC).convert("RGB")
d = ImageDraw.Draw(im)
try:
    font = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial.ttf", 38)
    small = ImageFont.truetype("/System/Library/Fonts/Supplemental/Arial.ttf", 32)
except OSError:
    font = small = ImageFont.load_default()

WHITE, AMBER = (255, 255, 255), (255, 193, 76)

for x, y, text in MARKS:
    d.line([(x + 95, y), (LABEL_X - 18, y)], fill=WHITE, width=3)
    d.ellipse([x + 86, y - 9, x + 104, y + 9], fill=WHITE)
    bb = d.textbbox((0, 0), text, font=font)
    d.text((LABEL_X, y - (bb[3] - bb[1]) // 2 - 6), text, font=font, fill=WHITE)

# the two exposures either side of contact sit inside the spray
y0, y1, text = BRACKET
bx = LABEL_X - 18
d.line([(bx, y0), (bx, y1)], fill=WHITE, width=3)
d.line([(bx, y0), (bx - 22, y0)], fill=WHITE, width=3)
d.line([(bx, y1), (bx - 22, y1)], fill=WHITE, width=3)
d.multiline_text((LABEL_X, (y0 + y1) // 2 - 44), text, font=font, fill=WHITE, spacing=8)

# contact marker on the water surface
d.line([(260, WATER_Y), (1150, WATER_Y)], fill=AMBER, width=3)
d.text((260, WATER_Y + 14), "water surface,  contact at t = 0", font=small, fill=AMBER)

im.save(OUT, quality=94)
print("wrote", os.path.normpath(OUT))
