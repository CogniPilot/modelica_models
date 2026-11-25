"""Plot actual finite-volume face intersections and the reference STL on an axial blade section."""
import argparse
import numpy as np
from scipy.io import loadmat
from matplotlib import pyplot as plt
from matplotlib.collections import LineCollection


def section(points, corners, starts, ends, station):
    y = points[corners,1]
    crossing = (np.minimum.reduceat(y, starts)<station) & (np.maximum.reduceat(y, starts)>station)
    segments = []
    for i in np.flatnonzero(crossing):
        a = points[corners[starts[i]:ends[i]]]
        b = np.roll(a, -1, axis=0)
        hits = (a[:,1]>station)!=(b[:,1]>station)
        a, b = a[hits], b[hits]
        xyz = a+((station-a[:,1])/(b[:,1]-a[:,1]))[:,None]*(b-a)
        if len(xyz)==2:
            segments.append(xyz[:,[2,0]]*1000)
    return np.array(segments)


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("mesh")
parser.add_argument("stl")
parser.add_argument("output")
parser.add_argument("--station", type=float, default=.035, help="Cartesian y plane in metres")
args = parser.parse_args()
mesh = loadmat(args.mesh)
starts = mesh["faces"][:,0].astype(int)-1
corners = mesh["corners"][:,0].astype(int)-1
edges = section(mesh["points"],corners,starts,np.r_[starts[1:],len(corners)],args.station)
with open(args.stl) as stream:
    tri = np.array([line.split()[1:] for line in stream if line.lstrip().startswith("vertex ")],float)
cad = section(tri,np.arange(len(tri)),np.arange(0,len(tri),3),np.arange(3,len(tri)+1,3),args.station)
bounds = cad.reshape(-1,2)
fig, (wide,detail) = plt.subplots(1,2,figsize=(12,4.5),gridspec_kw=dict(width_ratios=[1.7,1]))
fig.subplots_adjust(left=.067,right=.975,bottom=.18,top=.75,wspace=.24)
for ax in (wide,detail):
    ax.add_collection(LineCollection(edges,colors="#39818a",linewidths=.46))
    ax.add_collection(LineCollection(cad,colors="#bc4c06",linewidths=1.2))
    ax.set_aspect("equal")
    ax.set_xlabel("Transverse z (mm)")
    ax.set_ylabel("Shaft direction x (mm)")
    ax.spines[["right","top"]].set_visible(False)
lo,hi=bounds.min(axis=0),bounds.max(axis=0)
wide.set(xlim=(lo[0]-3,hi[0]+3),ylim=(lo[1]-3,hi[1]+3),title="Blade section")
tip=bounds[np.argmax(bounds[:,0])]
detail.set(xlim=(tip[0]-1.6,tip[0]+1.6),ylim=(tip[1]-1.6,tip[1]+1.6),title="Blade edge detail")
fig.suptitle("CAD-seeded volume mesh  /  Iowa port propeller",x=.065,ha="left",fontsize=18,weight="bold")
fig.text(.066,.845,f"Plane y = {1000*args.station:g} mm   ·   Teal lines are intersections with fluid cell faces   ·   Orange is the CAD section",fontsize=10)
fig.savefig(args.output,dpi=160)
