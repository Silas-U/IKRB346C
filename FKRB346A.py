'''-----------------------------------------------------------------------------------------------------#
#                                                                                                       #
# Program : Forward kinematics for a 6DOF robot arm                                                     #
# Author  : Udofia, Silas Silas                                                                         #
# Project : ROBOT ARM                                                                                   #
# Model   : RB346A                                                                                      #
# Ser_Nom : RB346A6DOFUSSOCT92022                                                                       #
# Date    : October 9, 2022                                                                             #
#                                                                                                       #   
# (forward kinematics) Given the desired end effector position vector and orientation, solves           #
# for the joint variables th1, th2, th3, th4, th5, th6                                                  #
#                                                                                                       #
-----------------------------------------------------------------------------------------------------'''
import math

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
T1 = 90
T2 = 90
T3 = -90
T4 = 90
T5 = 90
T6 = 90

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
            
print('THE HOMOGENEOUS TRANSFORMATION MATRIX')
print(GET_H0_6())
print('\n')            
