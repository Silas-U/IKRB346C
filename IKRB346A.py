'''-----------------------------------------------------------------------------------------------------#
#                                                                                                       #
# Program : Inverse kinematics for a 6DOF robot arm                                                     #
# Author  : Udofia, Silas Silas                                                                         #
# Project : ROBOT ARM                                                                                   #
# Model   : RB346A                                                                                      #
# Ser_Nom : RB346A6DOFUSSOCT92022                                                                       #
# Date    : October 9, 2022                                                                             #
#                                                                                                       #   
# (inverse kinematics) Given the desired end effector position vector and orientation, solves           #
# for the joint variables th1, th2, th3, th4, th5, th6                                                  #
#                                                                                                       #
-----------------------------------------------------------------------------------------------------'''
import math
from time import sleep
#import time

#JOINT OFFSET IN m
ex = 0.05
D1 = 0.1 + ex #Z
D6 = 0.05 #End effector length

#LINK LENGTHS IN m
A1 = 0.05
A2 = 0.1
A3 = 0.1
A4 = 0 #not used but inclusive
A5 = 0 #not used but inclusive
A6 = 0 #not used but inclusive

#SET INITIAL VALUES OF T1 T2 AND T3 TO 90 DEG
T1 = 0 #INITIAL : CAN BE VARIED TO OBTAIN A DIFFERENT SOLUTION TO THE IK
T2 = 110 #INITIAL
T3 = -110 #INITIAL

#IK INPUTS FROM MPU6050 OR OTHER SENSORS THAT CAN PROVIDE THIS DATA

#DESIRED END EFFECTOR POSITION VECTOR FOR ALL CONDITIONS OF PX !> A1 + A2 + A3 + D6 && PY !> A1 + A2 + A3 + D6 && PZ !> A1 + A2 + A3 + D6 + D1
PX = 0.162783
PY = -0.017101
PZ = 0.243969

#DESIRED ORIENTATION
T4 = 20 #ROLL
T5 = 90 #PITCH
T6 = 90 #YAW


T1 = (T1/180.0)*math.pi
T2 = (T2/180.0)*math.pi
T3 = (T3/180.0)*math.pi
T4 = (T4/180.0)*math.pi
T5 = (T5/180.0)*math.pi
T6 = (T6/180.0)*math.pi


def getMatrix(arr,r,c):
    matrix = []
    a = []
    for i in range(r):
        for j in range(c):
            a.append(arr)
            matrix.append(a)    
    for i in range(r):
        for j in range(c):
            return (matrix[i][j])
        
def setPos(p):
    matrix = []
    a = []
    for i in range(3):
        for j in range(4):
            a.append(p)
            matrix.append(a)    
    for i in range(3):
        for j in range(4):
            return (matrix[i][j])


#The homogeneous transformation matrix [H0_3] articulated robot section
sa = math.cos(T1)*math.cos(T2)*math.cos(T3) + (-math.cos(T1)*math.sin(T2)*math.sin(T3))
sb = math.sin(T1)
sc = math.cos(T1)*math.cos(T2)*math.sin(T3) + (math.cos(T1)*math.sin(T2)*math.cos(T3))

se = math.sin(T1)*math.cos(T2)*math.cos(T3) + (-math.sin(T1)*math.sin(T2)*math.sin(T3))
sf = -math.cos(T1)
sg = math.sin(T1)*math.cos(T2)*math.sin(T3) + math.sin(T1)*math.sin(T2)*math.cos(T3)

si = math.sin(T2)*math.cos(T3) + math.cos(T2)*math.sin(T3)
sj = 0
sk = math.sin(T2)*math.sin(T3) + (-math.cos(T2)*math.cos(T3))


H0_3A = ([sa,sb,sc],[se,sf,sg],[si,sj,sk])


#The homogeneous transformation matrix [H3_6] spherical wrist section
la = math.cos(T4)*math.cos(T5)*math.cos(T6) - math.sin(T4)*math.sin(T6)
lb = -math.cos(T4)*math.cos(T5)*math.sin(T6) - math.sin(T4)*math.cos(T6)
lc = math.cos(T4)*math.sin(T5)

le = math.sin(T4)*math.cos(T5)*math.cos(T6) + math.cos(T4)*math.sin(T6)
lf = -math.sin(T4)*math.cos(T5)*math.sin(T6) + math.cos(T4)*math.cos(T6)
lg = math.sin(T4)*math.sin(T5)


li = -math.sin(T5)*math.cos(T6)
lj = math.sin(T5)*math.sin(T6)
lk = math.cos(T5)


H3_4A = ([la,lb,lc],[le,lf,lg],[li,lj,lk])


print('\n')
#The homogeneous transformation matrix  row.col entries are given by the formulars below

#[a,b,c,d]
#[e,f,g,h]
#[i,j,k,l]
#[0,0,0,1]

#FIRST ROW OF THE HOMOGENEOUS TRANSFORMATION MATRIX H0_6  
A = H0_3A[0][0] * H3_4A[0][0] + H0_3A[0][1] * H3_4A[1][0] + H0_3A[0][2] * H3_4A[2][0] 
B = H0_3A[0][0] * H3_4A[0][1] + H0_3A[0][1] * H3_4A[1][1] + H0_3A[0][2] * H3_4A[2][1] 
C = H0_3A[0][0] * H3_4A[0][2] + H0_3A[0][1] * H3_4A[1][2] + H0_3A[0][2] * H3_4A[2][2] 
D = PX
#SECOND ROW OF THE HOMOGENEOUS TRANSFORMATION MATRIX H0_6
E = H0_3A[1][0] * H3_4A[0][0] + H0_3A[1][1] * H3_4A[1][0] + H0_3A[1][2] * H3_4A[2][0] 
F = H0_3A[1][0] * H3_4A[0][1] + H0_3A[1][1] * H3_4A[1][1] + H0_3A[1][2] * H3_4A[2][1] 
G = H0_3A[1][0] * H3_4A[0][2] + H0_3A[1][1] * H3_4A[1][2] + H0_3A[1][2] * H3_4A[2][2] 
H = PY
#THIRD ROW OF THE HOMOGENEOUS TRANSFORMATION MATRIX H0_6
I = H0_3A[2][0] * H3_4A[0][0] + H0_3A[2][1] * H3_4A[1][0] + H0_3A[2][2] * H3_4A[2][0] 
J = H0_3A[2][0] * H3_4A[0][1] + H0_3A[2][1] * H3_4A[1][1] + H0_3A[2][2] * H3_4A[2][1] 
K = H0_3A[2][0] * H3_4A[0][2] + H0_3A[2][1] * H3_4A[1][2] + H0_3A[2][2] * H3_4A[2][2] 
L = PZ

#THE HOMOGENOUS TRANSFORMATION MATRIX FROM FRAME 0 TO FRAME 6

H0_6 = ([A, B, C, D],
        [E, F, G, H],
        [I, J, K, L],
        [0, 0, 0, 1])

print(getMatrix(H0_6,4,3))
print('\n')




#DESIRED POSITION VECTOR D0_6 FROM THE H0_6 MATRIX
DX = H0_6[0][3]
DY = H0_6[1][3]
DZ = H0_6[2][3]

#DESIRED ORIENTATION R0_6 FROM THE H0_6 MATRIX
na = H0_6[0][0]
nb = H0_6[0][1]
nc = H0_6[0][2]
ne = H0_6[1][0]
nf = H0_6[1][1]
ng = H0_6[1][2]
ni = H0_6[2][0]
nj = H0_6[2][1]
nk = H0_6[2][2]

#CALCULATE FOR THE WRIST CENTER 
UX = DX -(nc*D6)
Ux = f"{UX:.6f}"

VY = DY -(ng*D6)
Vy = f"{VY:.6f}"

WZ = DZ -(nk*D6)
Wz = f"{WZ:.6f}"

#WRIST CENTER VECTOR
O0_C = [UX,VY,WZ]


#Calculate for Theta_1
def GET_TH1_DEG():
    Th_1 = math.atan2(VY,UX) #in radians
    theta1_deg = Th_1 /math.pi * 180
    #th1 = f"{theta1_deg:.6f}"
    return theta1_deg

def GET_TH1_RAD():
    Th_1 = math.atan2(VY,UX) #in radians
    return Th_1

#Calculation for Theta_2
# Theta_2 = phy1 + phy2
#---------------------This robot arm uses an elbow up configuration----------------------------
M = math.sqrt(math.pow(UX,2) + math.pow(VY,2))
r1 = M - A1
r2 = WZ - D1
r3 = math.sqrt(math.pow(r1,2) + math.pow(r2,2))

phy1 = math.acos((math.pow(A3,2) - math.pow(A2,2)- math.pow(r3,2))/(-2*A2*r3))
phy2 = math.atan2(r2,r1)
phy3 = math.acos((math.pow(r3,2)-math.pow(A2,2)-math.pow(A3,2))/(-2*A2*A3))

#Theta 2
def GET_TH2_DEG():
    Th_2 = (phy2 + phy1) #in radians
    theta2_deg = Th_2 /math.pi * 180
    #th2 = f"{theta2_deg:.6f}"
    return theta2_deg

def GET_TH2_RAD():
    Th_2 = (phy2 + phy1) #in radians
    return Th_2  

#Theta 3
def GET_TH3_DEG():
    Th_3 = - math.pi + phy3 #in radians
    theta3_deg = Th_3 /math.pi * 180
    #th3 = f"{theta3_deg:.6f}"
    return theta3_deg

def GET_TH3_RAD():
    Th_3 = - math.pi + phy3 #in radians
    return Th_3

#UPDATE T1, T2 AND T3
T1  = GET_TH1_RAD()
T2  = GET_TH2_RAD()
T3  = GET_TH3_RAD()

T1R = GET_TH1_RAD()
T2R = GET_TH2_RAD()
T3R = GET_TH3_RAD()

#R0_3 MATRIX DATA
#FIRST ROW,COL_1
RA = math.cos(T1R)*math.cos(T2R)*math.cos(T3R) + (-math.cos(T1R)*math.sin(T2R)*math.sin(T3R))

RB = math.sin(T1R)

RC = math.cos(T1R)*math.cos(T2R)*math.sin(T3R) + (math.cos(T1R)*math.sin(T2R)*math.cos(T3R))

RD = math.sin(T1R)*math.cos(T2R)*math.cos(T3R) + (-math.sin(T1R)*math.sin(T2R)*math.sin(T3R))

RE = -math.cos(T1R)

RF = math.sin(T1R)*math.cos(T2R)*math.sin(T3R) + math.sin(T1R)*math.sin(T2R)*math.cos(T3R)

RG = math.sin(T2R)*math.cos(T3R) + math.cos(T2R)*math.sin(T3R)

RH = 0

RR = math.sin(T2R)*math.sin(T3R) + (-math.cos(T2R)*math.cos(T3R))

#R0_3 MATRIX
def GET_R0_3():
    R0_3A = ([RA, RB, RC],[RD, RE, RF],[RG, RH, RR])
    return R0_3A


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
    Th_4 =  math.atan2(R3_6[1][2] , R3_6[0][2])/math.pi * 180
    #th4 = f"{Th_4:.6f}"
    return Th_4

def getTh4():
    R3_6 = GET_R3_6()
    Th_4 =  math.atan2(R3_6[1][2] , R3_6[0][2])/math.pi * 180
    th4 = f"{Th_4:.6f}"
    return th4
    
def GET_TH4_RAD():
    R3_6 = GET_R3_6()
    Th_4 =  math.atan2(R3_6[1][2] , R3_6[0][2])
    return Th_4    

#Theta_5 CALCULATION
def GET_TH5_DEG():
    R3_6 = GET_R3_6()
    sqrt_of_item02_12 = math.sqrt( math.pow(R3_6[0][2],2) + math.pow(R3_6[1][2],2))
    Th_5 = math.atan2(sqrt_of_item02_12 , R3_6[2][2])/math.pi * 180
    #th5 = f"{Th_5:.6f}"
    return Th_5

 
def getTh5():
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
    Th_6 = (math.pi + math.atan2(-R3_6[2][1] , R3_6[2][0]))/math.pi * 180
    if Th_6 == 0:
        Th_6 = 180 - (math.atan2(R3_6[2][1] , R3_6[2][0]))/math.pi * 180 
    #th6 = f"{Th_6:.6f}"
    return Th_6

def getTh6():
    R3_6 = GET_R3_6()
    Th_6 = (math.pi + math.atan2(-R3_6[2][1] , R3_6[2][0]))/math.pi * 180
    if Th_6 == 0:
        Th_6 = 180 - (math.atan2(R3_6[2][1] , R3_6[2][0]))/math.pi * 180 
    th6 = f"{Th_6:.6f}"
    return th6

def GET_TH6_RAD():
    R3_6 = GET_R3_6()
    Th_6 = (math.pi + math.atan2(-R3_6[2][1] , R3_6[2][0]))
    if Th_6 == 0:
        Th_6 = 180 - (math.atan2(R3_6[2][1] , R3_6[2][0]))
    return Th_6    

#-----------------------------------------THE END----------------------------------------------------------------------
print("NEW T1,T2,T3")
print(T1/math.pi * 180, T2/math.pi * 180, T3/math.pi * 180)
print('\n')
print(GET_TH1_DEG())
print(GET_TH2_DEG())
print(GET_TH3_DEG())
print(getTh4())
print(getTh5())
print(getTh6())
print('\n')

sleep(0.1)


