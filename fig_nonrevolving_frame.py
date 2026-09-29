"""Chapter 4 figure: the non-revolving frame.

The point the algebra makes badly: the body frame spins with the vehicle,
the non-revolving frame tracks the tilt but not the spin, so the reduced
attitude eta = [alpha_x, alpha_y] is yaw-independent.

(A) side view: the thrust axis leaning off vertical by eta.
(B) plan view: body axes sweeping round while the non-revolving axes stay put.

    python fig_nonrevolving_frame.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Arc

BLUE, GREY, RED = "#2c5f8a", "#9a9a9a", "#b5462e"
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "..", "..", "paper", "CityUHKThesis-main", "Assets",
                   "ch4fig1_nonrevolving_frame.pdf")


def arrow(ax, x0, y0, x1, y1, color, lw=1.6, ls="-", z=3):
    ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="-|>",
                                 mutation_scale=11, color=color, lw=lw,
                                 linestyle=ls, zorder=z,
                                 shrinkA=0, shrinkB=0))


fig, (axA, axB) = plt.subplots(1, 2, figsize=(7.2, 3.3))

# ---------------------------------------------------------------- (A) side
axA.set_xlim(-1.45, 1.45); axA.set_ylim(-0.55, 1.65)
axA.set_aspect("equal"); axA.axis("off")

# world vertical
arrow(axA, 0, 0, 0, 1.35, GREY, lw=1.3)
axA.text(0.07, 1.36, r"$\hat{z}^{\,w}$", color=GREY, fontsize=10, va="bottom")
arrow(axA, 0, 0, 1.05, 0, GREY, lw=1.3)
axA.text(1.08, -0.02, r"$\hat{x}^{\,w}$", color=GREY, fontsize=10, va="top")

# thrust axis, tilted
tilt = np.deg2rad(20)
zx, zy = 1.28*np.sin(tilt), 1.28*np.cos(tilt)
arrow(axA, 0, 0, zx, zy, BLUE, lw=2.2)
axA.text(zx+0.06, zy+0.02, r"$\hat{z}^{\,b}$", color=BLUE, fontsize=11, va="bottom")

# the airframe: one bar normal to the thrust axis, with a rotor disc at each end
px, py = np.cos(tilt), -np.sin(tilt)      # in-plane normal to z^b
axA.plot([-0.55*px, 0.55*px], [-0.55*py, 0.55*py], color="k", lw=2.4,
         solid_capstyle="round", zorder=4)
for sgn in (-1, 1):
    hx, hy = sgn*0.55*px, sgn*0.55*py
    dx, dy = 0.17*np.cos(tilt), -0.17*np.sin(tilt)
    axA.plot([hx - dx + 0.10*np.sin(tilt), hx + dx + 0.10*np.sin(tilt)],
             [hy - dy + 0.10*np.cos(tilt), hy + dy + 0.10*np.cos(tilt)],
             color="k", lw=2.0, solid_capstyle="round", zorder=4)
    axA.plot([hx, hx + 0.10*np.sin(tilt)], [hy, hy + 0.10*np.cos(tilt)],
             color="k", lw=1.2, zorder=4)

# the tilt angle
axA.add_patch(Arc((0, 0), 1.5, 1.5, theta1=90-np.rad2deg(tilt), theta2=90,
                  color=RED, lw=1.3))
axA.text(0.20, 0.86, r"$\eta$", color=RED, fontsize=12)



# ---------------------------------------------------------------- (B) plan
axB.set_xlim(-1.45, 1.45); axB.set_ylim(-1.35, 1.55)
axB.set_aspect("equal"); axB.axis("off")

# non-revolving axes: fixed
arrow(axB, 0, 0, 1.15, 0, BLUE, lw=2.0)
axB.text(1.18, 0.0, r"$\hat{x}^{\,m}$", color=BLUE, fontsize=11, va="center")
arrow(axB, 0, 0, 0, 1.15, BLUE, lw=2.0)
axB.text(0.04, 1.18, r"$\hat{y}^{\,m}$", color=BLUE, fontsize=11, va="bottom")

# body axes: rotated by psi, plus ghosts showing the sweep
psi = np.deg2rad(38)
for gh in np.deg2rad([-55, -20, 75, 110]):
    axB.plot([0, 0.95*np.cos(psi+gh)], [0, 0.95*np.sin(psi+gh)],
             color=GREY, lw=0.8, ls=(0, (2, 2)), zorder=1)
arrow(axB, 0, 0, 0.95*np.cos(psi), 0.95*np.sin(psi), "k", lw=1.8)
axB.text(0.99*np.cos(psi)+0.03, 0.99*np.sin(psi)+0.03, r"$\hat{x}^{\,b}$",
         fontsize=11, va="bottom")
arrow(axB, 0, 0, 0.95*np.cos(psi+np.pi/2), 0.95*np.sin(psi+np.pi/2), "k", lw=1.8)
axB.text(0.99*np.cos(psi+np.pi/2)-0.05, 0.99*np.sin(psi+np.pi/2)+0.03,
         r"$\hat{y}^{\,b}$", fontsize=11, va="bottom", ha="right")

# yaw angle and the spin
axB.add_patch(Arc((0, 0), 1.05, 1.05, theta1=0, theta2=np.rad2deg(psi),
                  color="k", lw=1.1))
axB.text(0.60, 0.15, r"$\psi$", fontsize=11)
axB.add_patch(Arc((0, 0), 2.30, 2.30, theta1=232, theta2=308, color=GREY, lw=1.2))
axB.add_patch(FancyArrowPatch((0.68, -0.92), (0.80, -0.83), arrowstyle="-|>",
                              mutation_scale=10, color=GREY, lw=1.2))
axB.text(0.0, -1.28, r"$\omega_z$", color=GREY, fontsize=11, ha="center")



fig.subplots_adjust(left=0.01, right=0.99, top=0.99, bottom=0.12, wspace=0.05)
fig.text(0.26, 0.035, r"(A) the thrust axis leans by $\boldsymbol{\eta}=[\alpha_x,\alpha_y]^{\top}$",
         fontsize=9, ha="center")
fig.text(0.75, 0.035, r"(B) $\hat{x}^{\,b},\hat{y}^{\,b}$ sweep with $\psi$; $\hat{x}^{\,m},\hat{y}^{\,m}$ do not",
         fontsize=9, ha="center")
fig.savefig(OUT, format="pdf")
print("wrote", os.path.normpath(OUT))
