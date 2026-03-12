#
#	MBsysTran - Release 8.1
#
#	Copyright 
#	Universite catholique de Louvain (UCLouvain) 
#	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Thu Mar 12 11:14:09 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Livrable2
#
#	==> Number of joints: 41
#
#	==> Function: F7 - Link Forces (1D)
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos, sqrt

def link(frc, trq, Flink, Z, Zd, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S17 = sin(q[17])
    C17 = cos(q[17])
    S29 = sin(q[29])
    C29 = cos(q[29])
    S33 = sin(q[33])
    C33 = cos(q[33])
    S37 = sin(q[37])
    C37 = cos(q[37])
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    PPlnk1 = s.dpt[1,3]*s.dpt[1,3]
    Z1 = sqrt(PPlnk1)
    e11 = -s.dpt[1,3]/Z1
    Zd1 = 0
    PPlnk2 = s.dpt[1,7]*s.dpt[1,7]
    Z2 = sqrt(PPlnk2)
    e12 = s.dpt[1,7]/Z2
    Zd2 = 0
    Plnk13 = s.dpt[1,10]-s.dpt[1,11]
    Plnk23 = s.dpt[2,10]-s.dpt[2,11]
    Plnk33 = s.dpt[3,10]-s.dpt[3,11]
    PPlnk3 = Plnk13*Plnk13+Plnk23*Plnk23+Plnk33*Plnk33
    Z3 = sqrt(PPlnk3)
    e13 = Plnk13/Z3
    e23 = Plnk23/Z3
    e33 = Plnk33/Z3
    Zd3 = 0
    Z4 = sqrt(0)
    Zd4 = 0

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
    Flink2 = s.user_LinkForces(Z2,Zd2,s,tsim,2)
    Flink3 = s.user_LinkForces(Z3,Zd3,s,tsim,3)
    Flink4 = s.user_LinkForces(Z4,Zd4,s,tsim,4)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk11 = Flink1*e11
    trqlnk6_1_2 = -fPlnk11*s.l[3,6]
    fSlnk11 = Flink1*e11
    fPlnk12 = Flink2*e12
    fSlnk12 = Flink2*e12
    frclnk6_2_1 = fPlnk11-fSlnk12
    trqlnk6_2_2 = trqlnk6_1_2+fSlnk12*s.l[3,6]
    fPlnk13 = Flink3*e13
    fPlnk23 = Flink3*(e23*C33+e33*S33)
    fPlnk33 = Flink3*(-e23*S33+e33*C33)
    fSlnk13 = Flink3*e13
    fSlnk23 = Flink3*e23
    fSlnk33 = Flink3*e33
    frclnk6_3_1 = -fSlnk13+frclnk6_2_1
    trqlnk6_3_1 = fSlnk23*(s.dpt[3,10]-s.l[3,6])-fSlnk33*s.dpt[2,10]
    trqlnk6_3_2 = trqlnk6_2_2-fSlnk13*(s.dpt[3,10]-s.l[3,6])+fSlnk33*(s.dpt[1,10]-s.l[1,6])
    trqlnk6_3_3 = fSlnk13*s.dpt[2,10]-fSlnk23*(s.dpt[1,10]-s.l[1,6])
 
# Symbolic model output

    frc[1,6] = s.frc[1,6]+frclnk6_3_1
    frc[2,6] = s.frc[2,6]-fSlnk23
    frc[3,6] = s.frc[3,6]-fSlnk33
    trq[1,6] = s.trq[1,6]+trqlnk6_3_1
    trq[2,6] = s.trq[2,6]+trqlnk6_3_2
    trq[3,6] = s.trq[3,6]+trqlnk6_3_3
    frc[1,17] = s.frc[1,17]-fSlnk11
    frc[1,29] = s.frc[1,29]+fPlnk12
    frc[1,33] = s.frc[1,33]+fPlnk13
    frc[2,33] = s.frc[2,33]+fPlnk23
    frc[3,33] = s.frc[3,33]+fPlnk33
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1
    Z[2] = Z2
    Zd[2] = Zd2
    Flink[2] = Flink2
    Z[3] = Z3
    Zd[3] = Zd3
    Flink[3] = Flink3
    Z[4] = Z4
    Zd[4] = Zd4
    Flink[4] = Flink4

# Number of continuation lines = 0


