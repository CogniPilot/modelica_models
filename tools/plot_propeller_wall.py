"""Draw the native wall polygons in the cached Iowa propeller finite-volume mesh."""
import argparse

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import PolyCollection
from matplotlib.patches import Rectangle
from scipy.io import loadmat


def load_wall(path):
    mesh = loadmat(path)
    faces = mesh["faces"].astype(np.int64)
    starts = faces[:, 0] - 1
    lengths = np.diff(np.r_[starts, len(mesh["corners"])])
    wall = (faces[:, 2] == 0) & (faces[:, 3] == 7)
    sizes = lengths[wall]
    if not len(sizes) or set(sizes) - {3, 4}:
        raise ValueError("Expected a clipped tetrahedral wall on patch 7")
    # Repeat the last vertex of each triangle for a uniform array. A quad's
    # fourth vertex is preserved, so no rendering diagonal is drawn.
    offsets = np.minimum(np.arange(4), sizes[:, None] - 1)
    corners = mesh["corners"][:, 0].astype(np.int64) - 1
    polygons = mesh["points"][corners[starts[wall, None] + offsets]] * 1000
    return polygons[np.argsort(polygons[:, :, 0].mean(axis=1))]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mesh", help="Cached MAT mesh from SampledTetStudy")
    parser.add_argument("output")
    args = parser.parse_args()
    polygons = load_wall(args.mesh)
    yz = polygons[:, :, 1:]
    # Inner fluid-face normals point into the propeller, so reverse them for
    # illumination from the observer's side of the solid.
    normals = -np.cross(polygons[:, 1] - polygons[:, 0], polygons[:, 2] - polygons[:, 0])
    normal_length = np.linalg.norm(normals, axis=1, keepdims=True)
    light = np.array([.82, -.38, .42])
    intensity = np.clip((normals / np.maximum(normal_length, 1e-12)) @ light, 0, 1)
    colors = plt.get_cmap("GnBu")(.45 - .30 * intensity)
    fig, axes = plt.subplots(1, 2, figsize=(10, 5.1), layout="constrained")
    views = [(-56, 56, -56, 56), (-7, 7, 41, 55)]
    names = ["Looking down the shaft", "Blade tip · boxed region"]
    for ax, limits, name, width in zip(axes, views, names, [.065, .26]):
        ymin, ymax, zmin, zmax = limits
        visible = ((yz[:, :, 0].max(axis=1) >= ymin) & (yz[:, :, 0].min(axis=1) <= ymax) &
                   (yz[:, :, 1].max(axis=1) >= zmin) & (yz[:, :, 1].min(axis=1) <= zmax))
        ax.add_collection(PolyCollection(yz[visible], facecolors=colors[visible], edgecolors="#366975",
                                         linewidths=width, antialiaseds=True, rasterized=True))
        ax.set(aspect="equal", xlim=(ymin, ymax), ylim=(zmin, zmax), title=name,
               xlabel="Transverse y (mm)", ylabel="Transverse z (mm)")
    axes[0].add_patch(Rectangle((-7, 41), 14, 14, fill=False, color="#bc5105", lw=1.1))
    fig.suptitle(f"Iowa port propeller · CAD-seeded cut mesh · {len(yz):,} native wall faces")
    fig.savefig(args.output, dpi=190)
    plt.close(fig)


if __name__ == "__main__":
    main()
