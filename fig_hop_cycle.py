"""Chapter 1 figure: the four stages of a single hop.

A schematic, not data. Written to the thesis Assets/ directory so it sits
alongside the figures produced by simu.m and figs_results.m. Style follows
Assets/blade_element_diagram.pdf: black line art, blue foils, light blue water.

    python fig_hop_cycle.py
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Arc, Polygon

BLUE, WATER, GREY = "#4a7fb5", "#cfe0ee", "#8a8a8a"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "..", "paper", "CityUHKThesis-main", "Assets",
                   "ch1fig1_hop_cycle.pdf")

SURFACE = 0.0


def vehicle(ax, y, lift_on, foils_wet=False):
    """Side view: frame bar, two lift rotors above, two legs kinking into foils."""
    hw = 0.62                                   # half-width of the frame
    ax.plot([-hw, hw], [y, y], color="k", lw=2.0, solid_capstyle="round")
    for sx in (-1, 1):
        # lift rotor: stalk + disk
        ax.plot([sx * 0.34, sx * 0.34], [y, y + 0.17], color="k", lw=1.2)
        ax.plot([sx * 0.34 - 0.20, sx * 0.34 + 0.20], [y + 0.17] * 2,
                color="k" if lift_on else GREY, lw=2.4 if lift_on else 1.4,
                solid_capstyle="round")
        if lift_on:                             # blur arcs = turning
            for dy in (0.055, 0.10):
                ax.add_patch(Arc((sx * 0.34, y + 0.17), 0.46, 0.13, theta1=200,
                                 theta2=340, color=GREY, lw=0.7))
        else:                                   # commanded to zero
            ax.plot([sx * 0.34 - 0.07, sx * 0.34 + 0.07],
                    [y + 0.25, y + 0.31], color=GREY, lw=1.0)
            ax.plot([sx * 0.34 - 0.07, sx * 0.34 + 0.07],
                    [y + 0.31, y + 0.25], color=GREY, lw=1.0)
        # leg drops, then kinks outward into the foil at 30 deg
        ax.plot([sx * hw, sx * hw], [y, y - 0.30], color="k", lw=1.6)
        x0, y0 = sx * hw, y - 0.30
        x1, y1 = sx * (hw + 0.42), y0 - 0.42 * 0.5774   # tan(30 deg)
        ax.add_patch(Polygon([[x0, y0], [x1, y1], [x1, y1 + 0.055],
                              [x0, y0 + 0.055]], closed=True,
                             facecolor=BLUE, edgecolor="k", lw=0.8,
                             zorder=5 if foils_wet else 3))


def spin_arrow(ax, y):
    ax.add_patch(Arc((0, y + 0.60), 0.86, 0.30, theta1=195, theta2=400,
                     color="k", lw=1.3))
    ax.add_patch(FancyArrowPatch((0.38, y + 0.68), (0.43, y + 0.60),
                                 arrowstyle="-|>", mutation_scale=9, color="k", lw=1.3))
    ax.text(0.50, y + 0.66, r"$\omega_z$", fontsize=11, va="center")


def vel(ax, x, y0, y1, label):
    ax.add_patch(FancyArrowPatch((x, y0), (x, y1), arrowstyle="-|>",
                                 mutation_scale=13, color="k", lw=1.8))
    ax.text(x + 0.12, (y0 + y1) / 2, label, fontsize=10, va="center", ha="left")


fig, axes = plt.subplots(1, 4, figsize=(7.4, 2.35))
titles = ["(A) spinning hover", "(B) unpowered descent",
          "(C) contact", "(D) ejection and recovery"]

for ax, title in zip(axes, titles):
    ax.set_xlim(-1.60, 1.90)
    ax.set_ylim(-0.62, 1.78)
    ax.axis("off")
    ax.set_aspect("equal")
    ax.fill_between([-1.60, 1.90], -0.62, SURFACE, color=WATER, zorder=0)
    ax.plot([-1.60, 1.90], [SURFACE, SURFACE], color=BLUE, lw=1.1, zorder=1)
    # stage label below the panel, leaving the top clear for the spin arrow
    ax.text(0.5, -0.05, title, fontsize=9, ha="center", va="top",
            transform=ax.transAxes)

# (A) hover: lift rotors on, spin established
vehicle(axes[0], 1.08, lift_on=True)
spin_arrow(axes[0], 1.08)

# (B) lift commanded to zero; the tangential rotors hold the spin
vehicle(axes[1], 0.74, lift_on=False)
spin_arrow(axes[1], 0.74)
vel(axes[1], 1.02, 0.92, 0.32, r"$\dot z<0$")

# (C) foils submerged: impulse up, spin braked
vehicle(axes[2], 0.34, lift_on=False, foils_wet=True)
axes[2].add_patch(FancyArrowPatch((0, 0.08), (0, 0.74), arrowstyle="-|>",
                                  mutation_scale=15, color=BLUE, lw=2.6))
axes[2].text(0.14, 0.86, "impulse", fontsize=9, color=BLUE,
             ha="center", va="bottom")
for sx in (-1, 1):
    for dx, dy in ((0.10, 0.30), (0.26, 0.21), (0.01, 0.34)):
        axes[2].plot([sx * (1.00 + dx * 0.35), sx * (1.00 + dx)],
                     [0.02, dy], color=BLUE, lw=0.9, zorder=2)

# (D) leaves the surface climbing; control restored
vehicle(axes[3], 0.80, lift_on=True)
spin_arrow(axes[3], 0.80)
vel(axes[3], 1.02, 0.34, 0.94, r"$\dot z>0$")

fig.subplots_adjust(left=0.005, right=0.995, top=0.99, bottom=0.16, wspace=0.03)
fig.savefig(OUT, format="pdf", transparent=False)
print("wrote", os.path.normpath(OUT))
