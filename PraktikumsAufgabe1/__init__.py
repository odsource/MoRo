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
    #print("x: ", x)
    y = point[1]
    #print("y: ", y)


    # Alpha
    Trans_DB2R = trans((-(l / 2), 0, -h))
    P_DB = Trans_DB2R.dot(point)

    #print("P_DB:")
    #print(P_DB)
    x_DB = P_DB[0]
    y_DB = P_DB[1]
    print("P_DB: ")
    print(P_DB)

    alpha = math.atan2(y_DB, x_DB)
    print("alpha: ", math.degrees(alpha))

    # Beta2
    Rot_DB = rot2trans(rotz(-alpha))

    P_D = Rot_DB.dot(P_DB)

    print("P_D: ", P_D)

    x_D = P_D[0]
    z_D = P_D[2]
    #print("x_D: ", x_D)
    #print("z_D: ", z_D)

    V_DB_P =x_D**2 + z_D**2
    beta2 = -math.acos((V_DB_P - l1**2 - l2**2) / (2*l1*l2))

    # Beta1
    gamma1 = math.asin(l2*math.sin(beta2)/math.sqrt(V_DB_P))
    gamma2 = math.atan2(z_D, x_D)

    beta1 = math.atan2(z_D, x_D) + math.asin(l2*math.sin(-beta2)/math.sqrt(V_DB_P))

    print("Beta2 : ", math.degrees(beta2))
    print("Beta1: ", math.degrees(beta1))

    return alpha, beta1, beta2


matrixP = ([matrixP_R[0]],[matrixP_R[1]], [matrixP_R[2]], [1])

inv = invKinematik(matrixP)

print(inv)

#print("\nc)")

#punktarray = []

#for punkt in punktarray:
#    invKinematik(punkt)