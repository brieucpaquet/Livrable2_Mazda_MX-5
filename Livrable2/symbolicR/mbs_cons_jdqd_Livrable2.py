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
#	==> Function: F18 - Constraints Quadratic Velocity Terms (Jdqd)
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def cons_jdqd(Jdqd, s):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S17 = sin(q[17])
    C17 = cos(q[17])
    S12 = sin(q[12])
    C12 = cos(q[12])
    S13 = sin(q[13])
    C13 = cos(q[13])
    S14 = sin(q[14])
    C14 = cos(q[14])
    S15 = sin(q[15])
    C15 = cos(q[15])
    S21 = sin(q[21])
    C21 = cos(q[21])
    S22 = sin(q[22])
    C22 = cos(q[22])
    S23 = sin(q[23])
    C23 = cos(q[23])
    S24 = sin(q[24])
    C24 = cos(q[24])
    S25 = sin(q[25])
    C25 = cos(q[25])
    S26 = sin(q[26])
    C26 = cos(q[26])
    S27 = sin(q[27])
    C27 = cos(q[27])
    S28 = sin(q[28])
    C28 = cos(q[28])
    S7 = sin(q[7])
    C7 = cos(q[7])
    S19 = sin(q[19])
    C19 = cos(q[19])
    S20 = sin(q[20])
    C20 = cos(q[20])
    S8 = sin(q[8])
    C8 = cos(q[8])
    S9 = sin(q[9])
    C9 = cos(q[9])
    S10 = sin(q[10])
    C10 = cos(q[10])
    S29 = sin(q[29])
    C29 = cos(q[29])
    S30 = sin(q[30])
    C30 = cos(q[30])
    S31 = sin(q[31])
    C31 = cos(q[31])
    S33 = sin(q[33])
    C33 = cos(q[33])
    S37 = sin(q[37])
    C37 = cos(q[37])
    S34 = sin(q[34])
    C34 = cos(q[34])
    S35 = sin(q[35])
    C35 = cos(q[35])
    S38 = sin(q[38])
    C38 = cos(q[38])
    S39 = sin(q[39])
    C39 = cos(q[39])
    S40 = sin(q[40])
    C40 = cos(q[40])
    S41 = sin(q[41])
    C41 = cos(q[41])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    ROjdqd13_52 = C30*C31-S30*S31
    ROjdqd13_62 = C30*S31+S30*C31
    ROjdqd13_82 = -C30*S31-S30*C31
    ROjdqd13_92 = C30*C31-S30*S31
    RLjdqd13_22 = s.dpt[2,39]*C30
    RLjdqd13_32 = s.dpt[2,39]*S30
    OMjdqd13_12 = qd[30]+qd[31]
    ORjdqd13_22 = -RLjdqd13_32*qd[30]
    ORjdqd13_32 = RLjdqd13_22*qd[30]
    Apqpjdqd13_22 = -ORjdqd13_32*qd[30]
    Apqpjdqd13_32 = ORjdqd13_22*qd[30]
    RLjdqd13_23 = ROjdqd13_52*s.dpt[2,41]+ROjdqd13_82*s.dpt[3,41]
    RLjdqd13_33 = ROjdqd13_62*s.dpt[2,41]+ROjdqd13_92*s.dpt[3,41]
    ORjdqd13_23 = -OMjdqd13_12*RLjdqd13_33
    ORjdqd13_33 = OMjdqd13_12*RLjdqd13_23
    Apqpjdqd13_23 = Apqpjdqd13_22-OMjdqd13_12*ORjdqd13_33
    Apqpjdqd13_33 = Apqpjdqd13_32+OMjdqd13_12*ORjdqd13_23
    RLjdqd14_22 = s.dpt[2,43]*C33-s.dpt[3,43]*S33
    RLjdqd14_32 = s.dpt[2,43]*S33+s.dpt[3,43]*C33
    ORjdqd14_22 = -RLjdqd14_32*qd[33]
    ORjdqd14_32 = RLjdqd14_22*qd[33]
    Apqpjdqd14_22 = -ORjdqd14_32*qd[33]
    Apqpjdqd14_32 = ORjdqd14_22*qd[33]
    jdqd20 = Apqpjdqd13_23-Apqpjdqd14_22
    jdqd21 = Apqpjdqd13_33-Apqpjdqd14_32
    Jdqd[1] = 0
    Jdqd[2] = 0
    Jdqd[3] = jdqd20
    Jdqd[4] = jdqd21

# Number of continuation lines = 0


