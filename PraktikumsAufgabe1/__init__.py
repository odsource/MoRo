import numpy as np
import math

# Liefert eine 2D-Rotationsmatrix mit Drehwinkel theta zurück.
def rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))
    return R

# Liefert eine elementare 3D-Rotationsmatrix mit Drehwinkel theta um Drehachse x zurück.
def rotx(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((1, 0, 0), (0, c, -s), (0, s, c)))
    return R

# Liefert eine elementare 3D-Rotationsmatrix mit Drehwinkel theta um Drehachse y zurück.
def roty(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, 0, s), (0, 1, 0), (-s, 0, c)))
    return R

# Liefert eine elementare 3D-Rotationsmatrix mit Drehwinkel theta um Drehachse z zurück.
def rotz(theta):
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))
    return R

# Wandelt die Rotationsmatrix r in eine homogene Transformationsmatrix um und liefert diese zurück.
def rot2trans(r):
    (n, m) = r.shape
    if n == 2:
        vz = np.array((0, 0, 1))
        hz = np.array(([0], [0]))
        rhz = np.hstack((r, hz))
        trans2d = np.vstack((rhz, vz))
        return trans2d
    else:
        vz = np.array((0, 0, 0, 1))
        hz = np.array(([0], [0], [0]))
        rhz = np.hstack((r, hz))
        trans3d = np.vstack((rhz, vz))
        return trans3d

# Liefert eine homogene Translationsmatrix mit Translation t zurück. t ist ein Tupel der Grö0e 2 bzw.
# 3 für den 2D- bzw. 3D-Fall.
def trans(t):
    n = len(t)
    translation = np.array(t)
    if n == 2:
        identity = np.identity(len(t))
        h = np.array((0, 0))
        zv = np.vstack((identity, h))
        tv = np.vstack(([t[0]], [t[1]], [1]))
        tlh2d = np.hstack((zv, tv))
        return tlh2d
    else:
        identity = np.identity(len(t))
        fh = translation[0:2]
        sh = np.array((translation[2], 1))
        h = np.array((0, 0, 0))
        zv = np.vstack((identity, h))
        tv = np.hstack((fh, sh))
        rh = np.array(([tv[0]], [tv[1]], [tv[2]], [tv[3]]))
        tlh3d = np.hstack((zv, rh))
        return tlh3d

def inverse(z):
    print("")


print("Aufgabe 1.1")
print("2.1")
matrixAB = trans((-2, 0, 0)).dot(rot2trans(rotz(math.pi)))
print(matrixAB)

print("Aufgabe 2")
print("a)\n")
matrixA = trans((2, 1, 0.1)).dot(rot2trans(rotz(math.pi/6)))
matrixB = trans((0.3 - 0.05, 0, 0.2))
matrixC = trans((0, 0, 0.05)).dot(rot2trans(rotz(math.pi/4.5))).dot(rot2trans(rotx(math.pi/2)))
matrixD = trans((0, 0, 0.05)).dot(rot2trans(rotz(math.pi/6))).dot(trans((0.5, 0, 0)))
matrixE = rot2trans(rotz((-1)*math.pi/18)).dot(trans((0.5, 0, 0)))

matrixOP = matrixA.dot(matrixB).dot(matrixC).dot(matrixD).dot(matrixE).dot(([0], [0], [0], [1]))

print("MatrixOP: ")
print(matrixOP)

print("\nb)")
matrixB = trans((0.3, 0, 0.2))
matrixC = rot2trans(rotz(math.pi/4.5)).dot(rot2trans(rotx(math.pi/2)))
matrixD = rot2trans(rotz(math.pi/6)).dot(trans((0.5, 0, 0)))
matrixE = rot2trans(rotz((-1)*math.pi/18)).dot(trans((0.5, 0, 0)))

matrixOP = matrixB.dot(matrixC).dot(matrixD).dot(matrixE).dot(([0], [0], [0], [1]))

print("Punkt P: ")
print(matrixOP)

print("\nc)")