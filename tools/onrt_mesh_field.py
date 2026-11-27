"""Build the CAD-seeded Delaunay field for Meshing.Examples.SampledTetStudy.

Inputs are the shaft-aligned W2025 ONRT port propeller STL in metres.
Requires NumPy, SciPy and VTK. The output is an uncut background, ready for
the native Modelica clipping, boundary identification and volume checks.
"""
import argparse
from pathlib import Path

import numpy as np
from scipy.io import savemat
from scipy.spatial import Delaunay, cKDTree
from vtkmodules.util.numpy_support import numpy_to_vtk, vtk_to_numpy
from vtkmodules.vtkCommonCore import vtkPoints
from vtkmodules.vtkCommonDataModel import vtkPolyData
from vtkmodules.vtkFiltersCore import vtkImplicitPolyDataDistance
from vtkmodules.vtkFiltersGeneral import vtkSampleImplicitFunctionFilter
from vtkmodules.vtkIOGeometry import vtkSTLReader


def background(vertices):
    rng = np.random.default_rng(3090)
    sites = []
    surface = cKDTree(vertices)
    # Overlapping site clouds grow coarser away from the rotor.
    for spacing, extent in ((.003, [.022,.06,.06]), (.009, [.06,.09,.09]),
                            (.024, [.16,.16,.16]), (.059, [.3198,.2132,.2132])):
        lo, hi = -np.array(extent), np.array(extent)
        hi[0] *= 2
        axes = [np.linspace(a, b, int(np.ceil((b-a)/spacing))+1) for a,b in zip(lo,hi)]
        coords = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1,3)
        boundary = np.any((coords==lo) | (coords==hi), axis=1)
        jitter = rng.uniform(-.18, .18, size=coords.shape)*spacing
        coords[~boundary] += jitter[~boundary]
        if spacing == .059:
            slip = (coords[boundary]!=lo) & (coords[boundary]!=hi)
            coords[boundary] += rng.uniform(-.1,.1,size=coords[boundary].shape)*spacing*slip
        else:
            coords[boundary] += jitter[boundary]
        keep = surface.query(coords)[0] > .72*spacing
        if sites:
            keep &= cKDTree(np.vstack(sites)).query(coords)[0] > .65*spacing
        sites.append(coords[keep])
    return np.vstack(sites)


def signed_distance(surface_path, points):
    reader = vtkSTLReader()
    reader.SetFileName(str(surface_path))
    reader.Update()
    distance = vtkImplicitPolyDataDistance()
    distance.SetInput(reader.GetOutput())
    vp = vtkPoints()
    vp.SetData(numpy_to_vtk(np.ascontiguousarray(points)))
    data = vtkPolyData()
    data.SetPoints(vp)
    sampler = vtkSampleImplicitFunctionFilter()
    sampler.SetInputData(data)
    sampler.SetImplicitFunction(distance)
    sampler.Update()
    return vtk_to_numpy(sampler.GetOutput().GetPointData().GetScalars()).astype(float)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("surface", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--offset", type=float, default=.00014, help="two-sided sample offset in metres")
    args = parser.parse_args()
    if not np.isfinite(args.offset) or args.offset <= 0:
        parser.error("offset must be positive")
    with args.surface.open() as stream:
        vertices = [line.split()[1:] for line in stream if line.lstrip().startswith("vertex ")]
    v, index = np.unique(np.array(vertices, dtype=float), axis=0, return_inverse=True)
    f = index.reshape(-1,3)
    tri = v[f]
    normal = np.cross(tri[:,1]-tri[:,0], tri[:,2]-tri[:,0])
    area2 = np.linalg.norm(normal,axis=1)
    if np.any(area2==0):
        raise ValueError("degenerate source triangle")
    normal /= area2[:,None]
    vertex_normal = np.zeros_like(v)
    for j in range(3):
        a, b = tri[:,(j+1)%3]-tri[:,j], tri[:,(j+2)%3]-tri[:,j]
        angle = np.arctan2(np.linalg.norm(np.cross(a,b),axis=1), np.sum(a*b,axis=1))
        np.add.at(vertex_normal, f[:,j], normal*angle[:,None])
    vertex_normal /= np.linalg.norm(vertex_normal, axis=1)[:,None]
    centre = tri.mean(axis=1)
    h = args.offset
    p = np.vstack([v+h*vertex_normal, v-h*vertex_normal, centre+h*normal, centre-h*normal, background(v)])
    phi = signed_distance(args.surface, p)
    t = Delaunay(p).simplices
    volume = np.linalg.det(p[t[:,1:]]-p[t[:,:1]])/6
    # Qhull can report zero-volume simplices on the planar outer box.
    t = t[np.abs(volume)>1e-26]
    patch = np.zeros(t.shape)
    for i in range(4):
        face = np.delete(t, i, axis=1)
        for k in range(3):
            for side, limit in enumerate((p[:,k].min(), p[:,k].max())):
                patch[np.all(p[face,k]==limit,axis=1),i] = 2*k+1+side
    savemat(args.output, dict(points=p,tets=t.astype(float)+1,patch=patch,phi=phi[:,None]), format="4")


if __name__ == "__main__":
    main()
