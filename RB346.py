#-------------------------------------------------------------------------------------------------------#
#                                                                                                       #
# Program : Forward and Inverse kinematics for a 6DOF robot arm                                         #
# Author  : Udofia, Silas Silas																			#
# Project : ROBOT ARM 																					#
# Model   : RB346 																						#
# Ser_Nom : RB3466DOFUSSOCT92022																		#
# Date    : October 9, 2022                                                                             #
#                                                                                                       #
# Description: Case_1(forward kinematics) Given the desired joint variables as input to the forward     #
# kinematics, solves for the position and orientation of the end effector                               #
#                                                                                                       #
# Case_2(inverse kinematics) Given the desired end effector position vector and orientation, solves     #
# for the joint variables th1, th2, th3, th4, th5, th6                                                  #
#                                                                                                       #
#-------------------------------------------------------------------------------------------------------#

#from unittest import result
import math

#JOINT OFFSET IN m
D1 = 0.1 #Z
D6 = 0.05 #End effector length


#JOINT ANGLES IN DEGREES(initial pos)
T1 = 20
T2 = 20
T3 = -20
T4 = 180
T5 = 160
T6 = 90

#JOINT ANGLE IN RADIANS
T1 = (T1/180.0)*math.pi 
T2 = (T2/180.0)*math.pi
T3 = (T3/180.0)*math.pi 
T4 = (T4/180.0)*math.pi
T5 = (T5/180.0)*math.pi
T6 = (T6/180.0)*math.pi


#LINK LENGTHS IN m
A1 = 0
A2 = 0.1
A3 = 0.1
A4 = 0 #not used but inclusive
A5 = 0 #not used but inclusive
A6 = 0 #not used but inclusive

#The homogeneous transformation matrix  row.col entries are given by the formulars below

#[a,b,c,d]
#[e,f,g,h]
#[i,j,k,l]
#[0,0,0,1]

#FIRST ROW,COL_1 OF THE HOMOGENEOUS TRANSFORMATION MATRIX R0_6  a 
a1 = (math.cos(T1)*math.cos(T2)*math.cos(T3)-math.cos(T1)*math.sin(T2)*math.sin(T3)) * (math.cos(T4)*math.cos(T5)*math.cos(T6)-math.sin(T4)*math.sin(T6))
a2 = (math.sin(T1)) * (math.sin(T4)*math.cos(T5)*math.cos(T6)+math.cos(T4)*math.sin(T6))
a3 = (math.cos(T1)*math.cos(T2)*math.sin(T3) + math.cos(T1)*math.sin(T2)*math.cos(T3)) * (-math.sin(T5)*math.cos(T6))

a = a1+a2+a3
A = f"{a:.6f}"

#FIRST ROW,COL_2 OF THE HOMOGENEOUS TRANSFORMATION MATRIX R0_6  b
b1 = (math.cos(T1)*math.cos(T2)*math.cos(T3)-math.cos(T1)*math.sin(T2)*math.sin(T3)) * (-math.cos(T4)*math.cos(T5)*math.sin(T6) -math.sin(T4)*math.cos(T6))
b2 = (math.sin(T1)) * (-math.sin(T4)*math.cos(T5)*math.sin(T6)+math.cos(T4)*math.cos(T6))
b3 = (math.cos(T1)*math.cos(T2)*math.sin(T3) + math.cos(T1)*math.sin(T2)*math.cos(T3)) * (math.sin(T5)*math.sin(T6))

b = b1+b2+b3
B = f"{b:.6f}"

#FIRST ROW,COL_3 OF THE HOMOGENEOUS TRANSFORMATION MATRIX R0_6  c
c1 = (math.cos(T1)*math.cos(T2)*math.cos(T3)-math.cos(T1)*math.sin(T2)*math.sin(T3)) * (math.cos(T4)*math.sin(T5))
c2 = (math.sin(T1)) * (math.sin(T4)*math.sin(T5))
c3 = (math.cos(T1)*math.cos(T2)*math.sin(T3)+math.cos(T1)*math.sin(T2)*math.cos(T3)) * (math.cos(T5))

c = c1+c2+c3
C = f"{c:.6f}"

#FIRST ROW,COL_4 OF THE HOMOGENEOUS TRANSFORMATION MATRIX R0_6  d
d1 = (math.cos(T1)*math.cos(T2)*math.cos(T3)-math.cos(T1)*math.sin(T2)*math.sin(T3)) * (math.cos(T4)*math.sin(T5)*D6)
d2 = (math.sin(T1)) * (math.sin(T4)*math.sin(T5)*math.sin(D6))
d3 = (math.cos(T1)*math.cos(T2)*math.sin(T3)+math.cos(T1)*math.sin(T2)*math.cos(T3)) * (math.cos(T5)*D6)
d4 =  A3*math.cos(T1)*math.cos(T2)*math.cos(T3)-A3*math.cos(T1)*math.sin(T2)*math.sin(T3)+A2*math.cos(T1)*math.cos(T2)

d = d1+d2+d3+d4
D = f"{d:.6f}"

#SECOND ROW,COL_1 OF THE HOMOGENEOUS TRANSFORMATION MATRIX R0_6  e
e1 = (math.sin(T1)*math.cos(T2)*math.cos(T3)-math.sin(T1)*math.sin(T2)*math.sin(T3)) * (math.cos(T4)*math.cos(T5)*math.cos(T6)-math.sin(T4)*math.sin(T6))
e2 = (-math.cos(T1)) * (math.sin(T4)*math.cos(T5)*math.cos(T6)+math.cos(T4)*math.sin(T6))
e3 = (math.sin(T1)*math.cos(T2)*math.sin(T3)+math.sin(T1)*math.sin(T2)*math.cos(T3)) * (-math.sin(T5)*math.cos(T6))

e = e1+e2+e3
E =f"{e:.6f}"

#SECOND ROW,COL_2 OF THE HOMOGENEOUS TRANSFORMATION MATRIX R0_6  f
f1 = (math.sin(T1)*math.cos(T2)*math.cos(T3)-math.sin(T1)*math.sin(T2)*math.sin(T3)) * (-math.cos(T4)*math.cos(T5)*math.sin(T6)-math.sin(T4)*math.cos(T6))
f2 = (-math.cos(T1)) * (-math.sin(T4)*math.cos(T5)*math.sin(T6)+math.cos(T4)*math.cos(T6))
f3 = (math.sin(T1)*math.cos(T2)*math.sin(T3)+math.sin(T1)*math.sin(T2)*math.cos(T3)) * (math.sin(T5)*math.sin(T6))

f = f1+f2+f3
F = f"{f:.6f}"

#SECOND ROW,COL_3 OF THE HOMOGENEOUS TRANSFORMATION MATRIX R0_6  g
g1 = (math.sin(T1)*math.cos(T2)*math.cos(T3)-math.sin(T1)*math.sin(T2)*math.sin(T3)) * (math.cos(T4)*math.sin(T5))
g2 = (-math.cos(T1)) * (math.sin(T4)*math.sin(T5))
g3 = (math.sin(T1)*math.cos(T2)*math.sin(T3)+math.sin(T1)*math.sin(T2)*math.cos(T3)) * (math.cos(T5))

g = g1+g2+g3
G = f"{g:.6f}"

#SECOND ROW,COL_4 OF THE HOMOGENEOUS TRANSFORMATION MATRIX R0_6  h
h1 = (math.sin(T1)*math.cos(T2)*math.cos(T3)-math.sin(T1)*math.sin(T2)*math.sin(T3)) * (math.cos(T4)*math.sin(T5)*D6)
h2 = (-math.cos(T1)) * (math.sin(T4)*math.sin(T5)*D6)
h3 = (math.sin(T1)*math.cos(T2)*math.sin(T3)+math.sin(T1)*math.sin(T2)*math.cos(T3)) * (math.cos(T5)*D6)
h4 = A3*math.sin(T1)*math.cos(T2)*math.cos(T3)-A3*math.sin(T1)*math.sin(T2)*math.sin(T3)+A2*math.sin(T1)*math.cos(T2)

h = h1+h2+h3+h4
H = f"{h:.6f}"

#THIRD ROW,COL_1 OF THE HOMOGENEOUS TRANSFORMATION MATRIX R0_6  i
i1 = (math.sin(T2)*math.cos(T3)+math.cos(T2)*math.sin(T3)) * (math.cos(T4)*math.cos(T5)*math.cos(T6)-math.sin(T4)*math.sin(T6))
i2 = (math.sin(T2)*math.sin(T3)-math.cos(T2)*math.cos(T3)) * (-math.sin(T5)*math.cos(T6))

i = i1+i2
I = f"{i:.6f}"

#THIRD ROW,COL_2 OF THE HOMOGENEOUS TRANSFORMATION MATRIX R0_6  j
j1 = (math.sin(T2)*math.cos(T3)+math.cos(T2)*math.sin(T3)) * (-math.cos(T4)*math.cos(T5)*math.sin(T6)-math.sin(T4)*math.cos(T6))
j2 = (math.sin(T2)*math.sin(T3)-math.cos(T2)*math.cos(T3)) * (math.sin(T5)*math.sin(T6))

j = j1+j2
J = f"{j:.6f}"

#THIRD ROW,COL_3 OF THE HOMOGENEOUS TRANSFORMATION MATRIX R0_6  k
k1 = (math.sin(T2)*math.cos(T3)+math.cos(T2)*math.sin(T3)) * (math.cos(T4)*math.sin(T5))
k2 = (math.sin(T2)*math.sin(T3)-math.cos(T2)*math.cos(T3)) * (math.cos(T5))

k = k1+k2
K = f"{k:.6f}"

#THIRD ROW,COL_4 OF THE HOMOGENEOUS TRANSFORMATION MATRIX R0_6  L
l1 = (math.sin(T2)*math.cos(T3)+math.cos(T2)*math.sin(T3)) * (math.cos(T4)*math.sin(T5)*D6)
l2 = (math.sin(T2)*math.sin(T3)-math.cos(T2)*math.cos(T3)) * (math.cos(T5)*D6)
l3 = A3*math.sin(T2)*math.cos(T3)+A3*math.cos(T2)*math.sin(T3)+A2*math.sin(T2)+D1

l = l1+l2+l3
L = f"{l:.6f}"

#THE LAST ROW OF THE HOMOGENEOUS TRANSFORMATION MATRIX IS = [0,0,0,1]
#THE HOMOGENOUS TRANSFORMATION MATRIX FROM FRAME 0 TO FRAME 6
def GET_H0_6():
    H0_6 = ([A, B, C, D],
            [E, F, G, H],
            [I, J, K, L],
            [0, 0, 0, 1])
    return H0_6

#--------------------------END OF FORWARD KINEMATICS-----------------------------



#-----------------------------INVERSE KINEMATICS----------------------------------


#DESIRED POSITION VECTOR(INPUT 1)(d h l) in H0_6 MATRIX
DX = d
DY = h
DZ = l

#DESIRED ORIENTATION(INPUT 2)
#RECALL THAT R0_6 = ([a, b, c],[e, f, g,],[i, j, k,])
na = a
nb = b
nc = c
ne = e
nf = f
ng = g
ni = i
nj = j
nk = k

if (DX > A2+A3 or DZ > D1+A2+A3):
    print('TARGET OUT OF RANGE')
else:    
    print('TARGET IN RANGE')
    print('\n')

#WRIST CENTER CALCULATION (d,h,l)(c,g,k)
U = DX -(nc*D6)
Ux = f"{U:.6f}"

V = DY -(ng*D6)
Vy = f"{V:.6f}"

W = DZ -(nk*D6)
Wz = f"{W:.6f}"

#WRIST CENTER VECTOR
O0_C = [Ux,Vy,Wz]


#Calculate for Theta_1
def GET_TH1_DEG():
    Th_1 = math.atan2(V,U) #in radians
    theta1_deg = Th_1 /math.pi * 180
    th1 = f"{theta1_deg:.6f}"
    return th1

def GET_TH1_RAD():
    Th_1 = math.atan2(V,U) #in radians
    return Th_1   
 
#Calculation for Theta_2
# Theta_2 = phy1 + phy2
#---------------------This robot arm uses an elbow up configuration----------------------------
r1 = math.sqrt(math.pow(U,2) + math.pow(V,2))
r2 = W - D1
r3 = math.sqrt(math.pow(r1,2) + math.pow(r2,2))

#R1 = f"{r1:.6f}"
#R2 = f"{r2:.6f}"
#R3 = f"{r3:.6f}"

phy1 = math.acos((math.pow(A3,2) - math.pow(A2,2)- math.pow(r3,2))/(-2*A2*r3))
phy2 = math.atan2(r2,r1)
phy3 = math.acos((math.pow(r3,2)-math.pow(A2,2)-math.pow(A3,2))/(-2*A2*A3))

#Theta 2
def GET_TH2_DEG():
    Th_2 = (phy2 + phy1) #in radians
    theta2_deg = Th_2 /math.pi * 180
    th2 = f"{theta2_deg:.6f}"
    return th2

def GET_TH2_RAD():
    Th_2 = (phy2 + phy1) #in radians
    return Th_2  

#Theta 3
def GET_TH3_DEG():
    Th_3 = - math.pi + phy3 #in radians
    theta3_deg = Th_3 /math.pi * 180
    th3 = f"{theta3_deg:.6f}"
    return th3

def GET_TH3_RAD():
    Th_3 = - math.pi + phy3 #in radians
    return Th_3

#R0_3 MATRIX DATA
#FIRST ROW,COL_1

Th_1R = GET_TH1_RAD()
Th_2R = GET_TH2_RAD()
Th_3R = GET_TH3_RAD()

RA = math.cos(Th_1R)*math.cos(Th_2R)*math.cos(Th_3R) - math.cos(Th_1R)*math.sin(Th_2R)*math.sin(Th_3R)
ra = f"{RA:.6f}"
#FIRST ROW,COL_2
RB = math.sin(Th_1R)
rb = f"{RB:.6f}"
#FIRST ROW,COL_3
RC = math.cos(Th_1R)*math.cos(Th_2R)*math.sin(Th_3R) + math.cos(Th_1R)*math.sin(Th_2R)*math.cos(Th_3R)
rc = f"{RC:.6f}"
#SECOND ROW,COL_1
RD = math.sin(Th_1R)*math.cos(Th_2R)*math.cos(Th_3R) - math.sin(Th_1R)*math.sin(Th_2R)*math.sin(Th_3R)
rd = f"{RD:.6f}"
#SECOND ROW,COL_2
RE = -math.cos(Th_1R)
re = f"{RE:.6f}"
#SECOND ROW,COL_3
RF = math.sin(Th_1R)*math.cos(Th_2R)*math.sin(Th_3R) + math.sin(Th_1R)*math.sin(Th_2R)*math.cos(Th_3R)
rf = f"{RF:.6f}"
#THIRD ROW,COL_1
RG = math.sin(Th_2R)*math.cos(Th_3R)+math.cos(Th_2R)*math.sin(Th_3R)
rg = f"{RG:.6f}"
#THIRD ROW,COL_2
RH = 0.0
rh = f"{RH:.6f}"
#THIRD ROW,COL_3
RR = math.sin(Th_2R)*math.sin(Th_3R)-math.cos(Th_2R)*math.cos(Th_3R)
rr = f"{RR:.6f}"

#R0_3 MATRIX
def GET_R0_3():
    R0_3A = ([RA, RB, RC],[RD, RE, RF],[RG, RH, RR])
    return R0_3A

#R0_6 MATRIX
R0_6 = ([a, b, c],[e, f, g,], [i, j, k,])

#R0_3_STRING_FOR_DISPLAY 
RS0_3 = ([ra, rb, rc], [ rd, re, rf], [rg, rh, rr])

#R0_6_STRING_FOR_DISPLAY
RS0_6 = ([A, B, C],[E, F, G,], [I, J, K,])

#R3_6 MATRIX CALCULATION


#................DETERMINANT CALCULATION.........................

def isSquare (m):
    return all (len (row) == len (m) for row in m)
    

def DET_Matrix(m):
    if isSquare(m):
        DET = (m[0][0]*(m[1][1]*m[2][2] - m[1][2]*m[2][1])) - (m[0][1]*(m[1][0]*m[2][2] - m[1][2]*m[2][0])) + (m[0][2]*(m[1][0]*m[2][1] - m[1][1]*m[2][0]))
    else:
        print('Error non square matrix inputed')
    return DET
      
def GET_mat_transpose(m):
    Mm = m[1][1]*m[2][2] - m[1][2]*m[2][1]
    Nm = m[1][0]*m[2][2] - m[1][2]*m[2][0]
    Om = m[1][0]*m[2][1] - m[1][1]*m[2][0]
    Pm = m[0][1]*m[2][2] - m[0][2]*m[2][1]
    Qm = m[0][0]*m[2][2] - m[0][2]*m[2][0]
    Rm = m[0][0]*m[2][1] - m[0][1]*m[2][0]
    Sm = m[0][1]*m[1][2] - m[0][2]*m[1][1]
    Tm = m[0][0]*m[1][2] - m[0][2]*m[1][0]
    Um = m[0][0]*m[1][1] - m[0][1]*m[1][0]
    
    MAT_transpose = ([Mm,-Pm,Sm], [-Nm,Qm,-Tm], [Om,-Rm,Um])
    return MAT_transpose



def MAT_inverse(m):
    MD = DET_Matrix(m)
    MT = GET_mat_transpose(m)
    Ad = 1/MD * MT[0][0]
    Bd = 1/MD * MT[0][1]
    Cd = 1/MD * MT[0][2]
    Dd = 1/MD * MT[1][0]
    Ed = 1/MD * MT[1][1]
    Fd = 1/MD * MT[1][2]
    Gd = 1/MD * MT[2][0]
    Hd = 1/MD * MT[2][1]
    Id = 1/MD * MT[2][2]
    
    Inverse = ([Ad,Bd,Cd],[Dd,Ed,Fd],[Gd,Hd,Id])
    return Inverse


def GET_R0_6_IK():
    R0_6_IKA = ([na,nb,nc],[ne,nf,ng],[ni,nj,nk])
    return R0_6_IKA


#R3_6 CALCULATION
R0_3 = GET_R0_3()
R0_6_IK = GET_R0_6_IK()

R0_3_INV = MAT_inverse(R0_3)        
RA = R0_3_INV[0][0]*R0_6_IK[0][0] + R0_3_INV[0][1]*R0_6_IK[1][0] + R0_3_INV[0][2]*R0_6_IK[2][0]
RB = R0_3_INV[0][0]*R0_6_IK[0][1] + R0_3_INV[0][1]*R0_6_IK[1][1] + R0_3_INV[0][2]*R0_6_IK[2][1]
RC = R0_3_INV[0][0]*R0_6_IK[0][2] + R0_3_INV[0][1]*R0_6_IK[1][2] + R0_3_INV[0][2]*R0_6_IK[2][2]

RD = R0_3_INV[1][0]*R0_6_IK[0][0] + R0_3_INV[1][1]*R0_6_IK[1][0] + R0_3_INV[1][2]*R0_6_IK[2][0]

RE = R0_3_INV[1][0]*R0_6_IK[0][1] + R0_3_INV[1][1]*R0_6_IK[1][1] + R0_3_INV[1][2]*R0_6_IK[2][1]
RF = R0_3_INV[1][0]*R0_6_IK[0][2] + R0_3_INV[1][1]*R0_6_IK[1][2] + R0_3_INV[1][2]*R0_6_IK[2][2]

RG = R0_3_INV[2][0]*R0_6_IK[0][0] + R0_3_INV[2][1]*R0_6_IK[1][0] + R0_3_INV[2][2]*R0_6_IK[2][0]
RH = R0_3_INV[2][0]*R0_6_IK[0][1] + R0_3_INV[2][1]*R0_6_IK[1][1] + R0_3_INV[2][2]*R0_6_IK[2][1]
RI = R0_3_INV[2][0]*R0_6_IK[0][2] + R0_3_INV[2][1]*R0_6_IK[1][2] + R0_3_INV[2][2]*R0_6_IK[2][2]

def GET_R3_6():
    R3_6 = ([RA,RB,RC],[RD,RE,RF],[RG,RH,RI])
    return R3_6



#Theta_4 CALCULATION
def GET_TH4_DEG():
    R3_6 = GET_R3_6()
    Th_4 =  math.atan2(R3_6[1][2],R3_6[0][2])/math.pi * 180
    th4 = f"{Th_4:.6f}"
    return th4
    
def GET_TH4_RAD():
    R3_6 = GET_R3_6()
    Th_4 =  math.atan2(R3_6[1][2],R3_6[0][2])
    return Th_4    

#Theta_5 CALCULATION
def GET_TH5_DEG():
    R3_6 = GET_R3_6()
    sqrt_of_item02_12 = math.sqrt( math.pow(R3_6[0][2],2) + math.pow(R3_6[1][2],2))
    Th_5 = math.atan2(sqrt_of_item02_12 , R3_6[2][2])/math.pi * 180
    th5 = f"{Th_5:.6f}"
    return th5

def GET_TH5_RAD():
    R3_6 = GET_R3_6()
    sqrt_of_item02_12 = math.sqrt( math.pow(R3_6[0][2],2) + math.pow(R3_6[1][2],2))
    Th_5 = math.atan2(sqrt_of_item02_12 , R3_6[2][2])
    return Th_5   

#Theta_6 CALCULATION
def GET_TH6_DEG():
    R3_6 = GET_R3_6()
    Th_6 = (math.pi - math.atan2(R3_6[2][1] , R3_6[2][0]))/math.pi * 180 
    th6 = f"{Th_6:.6f}"
    return th6

def GET_TH6_RAD():
    R3_6 = GET_R3_6()
    Th_6 =  math.atan2(-R3_6[2][1] , R3_6[2][0])
    return Th_6    

#-----------------------------------------THE END----------------------------------------------------------------------

#HEY BRO, THIS COMMENT FOR YOU, TRY ADD A FUNCTION TO DISPLAY THIS VALUES th1 to th6 on your OLED OR LCD, Thanks
print('THE HOMOGENEOUS TRANSFORMATION MATRIX')
print(GET_H0_6())
print('\n')
print('Values of theta_1 - theta_6')
print(GET_TH1_DEG())
print(GET_TH2_DEG())
print(GET_TH3_DEG())
print(GET_TH4_DEG())
print(GET_TH5_DEG())
print(GET_TH6_DEG())
print('\n')






                 





