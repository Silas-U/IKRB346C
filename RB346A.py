#-------------------------------------------------------------------------------------------------------#
#                                                                                                       #
# Program : Forward and Inverse kinematics for a 6DOF robot arm                                         #
# Author  : Udofia, Silas Silas                                                                         #
# Project : ROBOT ARM                                                                                   #
# Model   : RB346A                                                                                      #
# Ser_Nom : RB346A6DOFUSSOCT92022                                                                       #
# Date    : October 9, 2022                                                                             #
#                                                                                                       #
# Description: Case_1(forward kinematics) Given the desired joint variables as input to the forward     #
# kinematics, solves for the position and orientation of the end effector                               #
#                                                                                                       #
# Case_2(inverse kinematics) Given the desired end effector position vector and orientation, solves     #
# for the joint variables th1, th2, th3, th4, th5, th6                                                  #
#                                                                                                       #
#-------------------------------------------------------------------------------------------------------#

import math
from time import sleep
#from machine import Pin
#from machine import PWM
from imu import MPU6050
from machine import Pin, SoftI2C
import time

i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=400000)
imu = MPU6050(i2c)
earths_gravitation = 9.8
st = time.time_ns()


def accel_mps(g):
    mps = g * earths_gravitation
    return mps




#JOINT OFFSET IN m
ex = 0.05
D1 = 0.1 + ex #Z
D6 = 0.05 #End effector length


#INITIAL JOINT ANGLES IN DEGREES(initial pos) min 1 deg
TI1 = 10
TI2 = 10
TI3 = 10
TI4 = 10
TI5 = 10
TI6 = 10

#FINAL JOINT ANGLES IN DEGREES(initial pos) min 1 deg
T1 = 56.30993247402023
T2 = 78.42246010791024
T3 = -156.84492021582048
T4 = -14.621238152253465
T5 = 100.66272294447377
T6 = 59.30699073006936

#JOINT ANGLE IN RADIANS
T1 = (T1/180.0)*math.pi 
T2 = (T2/180.0)*math.pi
T3 = (T3/180.0)*math.pi 
T4 = (T4/180.0)*math.pi
T5 = (T5/180.0)*math.pi
T6 = (T6/180.0)*math.pi


#LINK LENGTHS IN m
A1 = 0.05
A2 = 0.1
A3 = 0.1
A4 = 0 #not used but inclusive
A5 = 0 #not used but inclusive
A6 = 0 #not used but inclusive


#The homogeneous transformation matrix [H0_3] articulated robot section
sa = math.cos(T1)*math.cos(T2)*math.cos(T3) + (-math.cos(T1)*math.sin(T2)*math.sin(T3))
sb = math.sin(T1)
sc = math.cos(T1)*math.cos(T2)*math.sin(T3) + (math.cos(T1)*math.sin(T2)*math.cos(T3))
sd = A3*math.cos(T1)*math.cos(T2)*math.cos(T3) + (-A3*math.cos(T1)*math.sin(T2)*math.sin(T3)) + A2*math.cos(T1)*math.cos(T2) + A1*math.cos(T1)

se = math.sin(T1)*math.cos(T2)*math.cos(T3) + (-math.sin(T1)*math.sin(T2)*math.sin(T3))
sf = -math.cos(T1)
sg = math.sin(T1)*math.cos(T2)*math.sin(T3) + math.sin(T1)*math.sin(T2)*math.cos(T3)
sh = A3*math.sin(T1)*math.cos(T2)*math.cos(T3) + (-A3*math.sin(T1)*math.sin(T2)*math.sin(T3)) + A2*math.sin(T1)*math.cos(T2) + A1*math.sin(T1)

si = math.sin(T2)*math.cos(T3) + math.cos(T2)*math.sin(T3)
sj = 0
sk = math.sin(T2)*math.sin(T3) + (-math.cos(T2)*math.cos(T3))
sl = A3*math.sin(T2)*math.cos(T3) + A3*math.cos(T2)*math.sin(T3) + A2*math.sin(T2) + D1


H0_3A = ([sa,sb,sc,sd],[se,sf,sg,sh],[si,sj,sk,sl],[0,0,0,1])

for a in H0_3A :
    print(a)
print('\n')

#The homogeneous transformation matrix [H3_6] spherical wrist section
la = math.cos(T4)*math.cos(T5)*math.cos(T6) - math.sin(T4)*math.sin(T6)
lb = -math.cos(T4)*math.cos(T5)*math.sin(T6) - math.sin(T4)*math.cos(T6)
lc = math.cos(T4)*math.sin(T5)
ld = math.cos(T4)*math.sin(T5)*D6

le = math.sin(T4)*math.cos(T5)*math.cos(T6) + math.cos(T4)*math.sin(T6)
lf = -math.sin(T4)*math.cos(T5)*math.sin(T6) + math.cos(T4)*math.cos(T6)
lg = math.sin(T4)*math.sin(T5)
lh = math.sin(T4)*math.sin(T5)*D6

li = -math.sin(T5)*math.cos(T6)
lj = math.sin(T5)*math.sin(T6)
lk = math.cos(T5)
ll = math.cos(T5)*D6

H3_4A = ([la,lb,lc,ld],[le,lf,lg,lh],[li,lj,lk,ll],[0,0,0,1])


print('\n')
#The homogeneous transformation matrix  row.col entries are given by the formulars below

#[a,b,c,d]
#[e,f,g,h]
#[i,j,k,l]
#[0,0,0,1]

#FIRST ROW OF THE HOMOGENEOUS TRANSFORMATION MATRIX H0_6  
A = H0_3A[0][0] * H3_4A[0][0] + H0_3A[0][1] * H3_4A[1][0] + H0_3A[0][2] * H3_4A[2][0] + H0_3A[0][3] * H3_4A[3][0]
B = H0_3A[0][0] * H3_4A[0][1] + H0_3A[0][1] * H3_4A[1][1] + H0_3A[0][2] * H3_4A[2][1] + H0_3A[0][3] * H3_4A[3][1]
C = H0_3A[0][0] * H3_4A[0][2] + H0_3A[0][1] * H3_4A[1][2] + H0_3A[0][2] * H3_4A[2][2] + H0_3A[0][3] * H3_4A[3][2]
D = H0_3A[0][0] * H3_4A[0][3] + H0_3A[0][1] * H3_4A[1][3] + H0_3A[0][2] * H3_4A[2][3] + H0_3A[0][3] * H3_4A[3][3]
#SECOND ROW OF THE HOMOGENEOUS TRANSFORMATION MATRIX H0_6
E = H0_3A[1][0] * H3_4A[0][0] + H0_3A[1][1] * H3_4A[1][0] + H0_3A[1][2] * H3_4A[2][0] + H0_3A[1][3] * H3_4A[3][0]
F = H0_3A[1][0] * H3_4A[0][1] + H0_3A[1][1] * H3_4A[1][1] + H0_3A[1][2] * H3_4A[2][1] + H0_3A[1][3] * H3_4A[3][1]
G = H0_3A[1][0] * H3_4A[0][2] + H0_3A[1][1] * H3_4A[1][2] + H0_3A[1][2] * H3_4A[2][2] + H0_3A[1][3] * H3_4A[3][2]
H = H0_3A[1][0] * H3_4A[0][3] + H0_3A[1][1] * H3_4A[1][3] + H0_3A[1][2] * H3_4A[2][3] + H0_3A[1][3] * H3_4A[3][3]
#THIRD ROW OF THE HOMOGENEOUS TRANSFORMATION MATRIX H0_6
I = H0_3A[2][0] * H3_4A[0][0] + H0_3A[2][1] * H3_4A[1][0] + H0_3A[2][2] * H3_4A[2][0] + H0_3A[2][3] * H3_4A[3][0]
J = H0_3A[2][0] * H3_4A[0][1] + H0_3A[2][1] * H3_4A[1][1] + H0_3A[2][2] * H3_4A[2][1] + H0_3A[2][3] * H3_4A[3][1]
K = H0_3A[2][0] * H3_4A[0][2] + H0_3A[2][1] * H3_4A[1][2] + H0_3A[2][2] * H3_4A[2][2] + H0_3A[2][3] * H3_4A[3][2]
L = H0_3A[2][0] * H3_4A[0][3] + H0_3A[2][1] * H3_4A[1][3] + H0_3A[2][2] * H3_4A[2][3] + H0_3A[2][3] * H3_4A[3][3]
#FOURTH ROW OF THE HOMOGENEOUS TRANSFORMATION MATRIX H0_6
M1 = H0_3A[3][0] * H3_4A[0][0] + H0_3A[3][1] * H3_4A[1][0] + H0_3A[3][2] * H3_4A[2][0] + H0_3A[3][3] * H3_4A[3][0]
N = H0_3A[3][0] * H3_4A[0][1] + H0_3A[3][1] * H3_4A[1][1] + H0_3A[3][2] * H3_4A[2][1] + H0_3A[3][3] * H3_4A[3][1]
O = H0_3A[3][0] * H3_4A[0][2] + H0_3A[3][1] * H3_4A[1][2] + H0_3A[3][2] * H3_4A[2][2] + H0_3A[3][3] * H3_4A[3][2]
P = H0_3A[3][0] * H3_4A[0][3] + H0_3A[3][1] * H3_4A[1][3] + H0_3A[3][2] * H3_4A[2][3] + H0_3A[3][3] * H3_4A[3][3]

#THE HOMOGENOUS TRANSFORMATION MATRIX FROM FRAME 0 TO FRAME 6
def GET_H0_6():
    H0_6 = ([A, B, C, D],
            [E, F, G, H],
            [I, J, K, L],
            [M1,N, O, P])
    
    for i in H0_6:
        for j in i:
            i_matrix = f"{j:.6f}"
            i_m = [i_matrix]
            print(i_m)




#-----------------------------INVERSE KINEMATICS----------------------------------
          
            
TSW4 = 90            
TSW5 = 90
TSW6 = 0


#JOINT ANGLE IN RADIANS
TSW4 = (TSW4/180.0)*math.pi 
TSW5 = (TSW5/180.0)*math.pi
TSW6 = (TSW6/180.0)*math.pi 

#The orientation  matrix [R3_6] spherical wrist section            
qa = math.cos(TSW4)*math.cos(TSW5)*math.cos(TSW6) - math.sin(TSW4)*math.sin(TSW6)
qb = -math.cos(TSW4)*math.cos(TSW5)*math.sin(TSW6) - math.sin(TSW4)*math.cos(TSW6)
qc = math.cos(TSW4)*math.sin(TSW5)


qe = math.sin(TSW4)*math.cos(TSW5)*math.cos(TSW6) + math.cos(TSW4)*math.sin(TSW6)
qf = -math.sin(TSW4)*math.cos(TSW5)*math.sin(TSW6) + math.cos(TSW4)*math.cos(TSW6)
qg = math.sin(TSW4)*math.sin(TSW5)

qi = -math.sin(TSW5)*math.cos(TSW6)
qj = math.sin(TSW5)*math.sin(TSW6)
qk = math.cos(TSW5)


R3_4Q = ([qa,qb,qc],[qe,qf,qg],[qi,qj,qk])

def GET_R3_4Q():
    for i in R3_4Q:
        i_m = [i]
        print(i_m)            
            
            
            
            

#DESIRED POSITION VECTOR(INPUT 1)(d h l) in H0_6 MATRIX
DX = D
DY = H
DZ = L

#DESIRED ORIENTATION(INPUT 2)
#RECALL THAT R0_6 = ([a, b, c],[e, f, g,],[i, j, k,])
na = A
nb = B
nc = C
ne = E
nf = F
ng = G
ni = I
nj = J
nk = K


#WRIST CENTER CALCULATION (d,h,l)(c,g,k)
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

#R0_3 MATRIX DATA

T1R = GET_TH1_RAD()
T2R = GET_TH2_RAD()
T3R = GET_TH3_RAD()

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

def GET_TH6_RAD():
    R3_6 = GET_R3_6()
    Th_6 = (math.pi + math.atan2(-R3_6[2][1] , R3_6[2][0]))
    if Th_6 == 0:
        Th_6 = 180 - (math.atan2(R3_6[2][1] , R3_6[2][0]))
    return Th_6    

#-----------------------------------------THE END----------------------------------------------------------------------

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
#print(GET_R3_4Q())

# pwm = PWM(Pin(2))
# pwm.freq(50)
# 
# def setServoCycle(position):
#     pwm.duty_u16(position)
#     sleep(0.01)
#     
# def angle(a):
#     deg = a * 50
#     return deg
# 
# servo1Angle = GET_TH1_DEG()
# servo2Angle = GET_TH2_DEG()
# servo3Angle = GET_TH3_DEG()
# servo4Angle = GET_TH4_DEG()
# servo5Angle = GET_TH5_DEG()
# servo6Angle = GET_TH6_DEG()
# 
# init_pos = angle(TI1)
# final_pos = angle(-servo3Angle)
# print(-servo3Angle)
# for pos in range(init_pos, final_pos, 30):
#     setServoCycle(pos)
#     
#
# while True: 
#     ax = round(imu.accel.x,2)
#     ay = round(imu.accel.y,2)
#     az = round(imu.accel.z,2)
# 
#     accAngleX = (math.atan2(accel_mps(ay) , math.sqrt(math.pow(accel_mps(ax),2) + math.pow(accel_mps(az),2))) * 180 / math.pi) + 5.0
#     accAngleY = (-1 * math.atan2(accel_mps(ax) , math.sqrt(math.pow(accel_mps(ay),2) + math.pow(accel_mps(az),2))) * 180 / math.pi) +1.55
# 
#     print(accAngleX)
#     print(accAngleY)
          

