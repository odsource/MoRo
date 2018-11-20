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

l = 0.6
l1 = 0.5
l2 = 0.5
h = 0.2
a = 0.1
b = 0.1

print("Aufgabe 1.1")
print("2.1")
matrixAB = trans((-2, 0, 0)).dot(rot2trans(rotz(math.pi)))
print(matrixAB)

print("2.2")

print("2.3")

print("Aufgabe 2")
print("a)\n")
matrixA = trans((2, 1, 0.1)).dot(rot2trans(rotz(math.pi/6)))
matrixB = trans(((l/2) - 0.05, 0, 0.2))
matrixC = trans((0, 0, 0.05)).dot(rot2trans(rotz(math.pi/4.5))).dot(rot2trans(rotx(math.pi/2)))
matrixD = trans((0, 0, 0.05)).dot(rot2trans(rotz(math.pi/6))).dot(trans((0.5, 0, 0)))
matrixE = rot2trans(rotz((-1)*math.pi/18)).dot(trans((0.5, 0, 0)))

matrixP_O = matrixA.dot(matrixB).dot(matrixC).dot(matrixD).dot(matrixE).dot(([0], [0], [0], [1]))

print("MatrixP_O: ")
print(matrixP_O)

print("\nb)")
matrixB = trans((0.3, 0, 0.2))
matrixC = rot2trans(rotz(math.pi/4.5)).dot(rot2trans(rotx(math.pi/2)))
matrixD = rot2trans(rotz(math.pi/6)).dot(trans((0.5, 0, 0)))
matrixE = rot2trans(rotz((-1)*math.pi/18)).dot(trans((0.5, 0, 0)))

matrixP_R = matrixB.dot(matrixC).dot(matrixD).dot(matrixE).dot(([0], [0], [0], [1]))

print("Punkt P_R: ")
print(matrixP_R)


a = 0
b = 0

def invKinematik(point):
    # Punktkoordinaten im KS R (entsprechend Aufgabe 2.5: x_F, y_F)
    x = point[0]
    print("x: ", x)
    y = point[1]
    print("y: ", y)


    # Punktkoordinaten von R in DB wechseln für alpha
    Trans_DB2R = trans((-(l / 2), 0, -h))
    P_DB = Trans_DB2R.dot(point)

    print("P_DB:")
    print(P_DB)
    x_DB = P_DB[0]
    y_DB = P_DB[1]

    alpha = math.atan2(y_DB, x_DB)
    print("alpha: ", math.degrees(alpha))


    # Punktkoordinaten von R in D wechseln für beta
    Trans_RDB3 = trans(((l / 2) - (a / 2), 0, h))
    Trans_DBD3 = trans((0, 0, b / 2)).dot(rot2trans(rotz(math.radians(40)))).dot(rot2trans(rotx(math.pi/2)))

    P_D = Trans_RDB3.dot(Trans_DBD3).dot(point)

    x_D = P_D[0]
    y_D = P_D[1]

    f = math.sqrt(x_D**2 + y_D**2) # entsprechend A2.5: a = sqrt(x_F² + y_F²)
    print("f: ", f)

    c = (f**2 - l1**2 - l2**2 ) / 2*l1 # entsprechend A2.5: c = (a² - a1² - a2²) | 2a1 ; hier: c = c, a = f, a1 = l1, a2 = l2
    print("c: ", c)

    d = math.sqrt(l2**2-c**2) # entsprechend A2.5: b = sqrt(a2² - c²) | hier: b = d, a2 = l2, c = c
    print("d: ", d)
    beta2_neu = math.atan2(d, c) # entsprechend A2.5: theta2 = atan2(b, c) | hier: theta2 = beta2_neu

    print("beta2: ", math.degrees(beta2_neu))
    beta1_neu = math.atan2(y_D, x_D) + math.atan2(d, l1 + c)    # entsprechend A2.5: theta1 = gamma1 + gamma2,  gamma1 = atan2(y_F, x_F), gamma2 = atan2(b, a1 + c)
                                                                # hier: theta1 = beta1_neu | gamma1 = math.atan2(y, x), gamma2 = math.atan2(d, l1 + c)
    print("beta1: ", math.degrees(beta1_neu))

matrixP = ([matrixP_R[0]],[matrixP_R[1]], [0], [1])

inv = invKinematik(matrixP)

print(inv)

print("\nc)")