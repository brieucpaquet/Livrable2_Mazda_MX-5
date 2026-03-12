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
#	==> Function: F8 - Constraints and Constraints Jacobian(h, J)
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def cons_hJ(h, Jac, s):
    q = s.q
 
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

    ROlp13_52 = C30*C31-S30*S31
    ROlp13_62 = C30*S31+S30*C31
    ROlp13_82 = -C30*S31-S30*C31
    ROlp13_92 = C30*C31-S30*S31
    RLlp13_22 = s.dpt[2,39]*C30
    RLlp13_32 = s.dpt[2,39]*S30
    POlp13_22 = RLlp13_22+s.dpt[2,9]
    POlp13_32 = RLlp13_32+s.dpt[3,9]
    RLlp13_23 = ROlp13_52*s.dpt[2,41]+ROlp13_82*s.dpt[3,41]
    RLlp13_33 = ROlp13_62*s.dpt[2,41]+ROlp13_92*s.dpt[3,41]
    POlp13_23 = POlp13_22+RLlp13_23
    POlp13_33 = POlp13_32+RLlp13_33
    JTlp13_23_1 = -RLlp13_32-RLlp13_33
    JTlp13_33_1 = RLlp13_22+RLlp13_23
    RLlp14_22 = s.dpt[2,43]*C33-s.dpt[3,43]*S33
    RLlp14_32 = s.dpt[2,43]*S33+s.dpt[3,43]*C33
    POlp14_22 = RLlp14_22+s.dpt[2,11]
    POlp14_32 = RLlp14_32+s.dpt[3,11]
    h_20 = POlp13_23-POlp14_22
    h_21 = POlp13_33-POlp14_32
    h[1] = q[18]
    h[2] = q[18]
    h[3] = h_20
    h[4] = h_21
    Jac[1,1] = 0
    Jac[1,2] = 0
    Jac[1,3] = 0
    Jac[1,4] = 0
    Jac[1,5] = 0
    Jac[1,6] = 0
    Jac[1,7] = 0
    Jac[1,8] = 0
    Jac[1,9] = 0
    Jac[1,10] = 0
    Jac[1,11] = 0
    Jac[1,12] = 0
    Jac[1,13] = 0
    Jac[1,14] = 0
    Jac[1,15] = 0
    Jac[1,16] = 0
    Jac[1,17] = 0
    Jac[1,18] = (1.0)
    Jac[1,19] = 0
    Jac[1,20] = 0
    Jac[1,21] = 0
    Jac[1,22] = 0
    Jac[1,23] = 0
    Jac[1,24] = 0
    Jac[1,25] = 0
    Jac[1,26] = 0
    Jac[1,27] = 0
    Jac[1,28] = 0
    Jac[1,29] = 0
    Jac[1,30] = 0
    Jac[1,31] = 0
    Jac[1,32] = 0
    Jac[1,33] = 0
    Jac[1,34] = 0
    Jac[1,35] = 0
    Jac[1,36] = 0
    Jac[1,37] = 0
    Jac[1,38] = 0
    Jac[1,39] = 0
    Jac[1,40] = 0
    Jac[1,41] = 0
    Jac[2,1] = 0
    Jac[2,2] = 0
    Jac[2,3] = 0
    Jac[2,4] = 0
    Jac[2,5] = 0
    Jac[2,6] = 0
    Jac[2,7] = 0
    Jac[2,8] = 0
    Jac[2,9] = 0
    Jac[2,10] = 0
    Jac[2,11] = 0
    Jac[2,12] = 0
    Jac[2,13] = 0
    Jac[2,14] = 0
    Jac[2,15] = 0
    Jac[2,16] = 0
    Jac[2,17] = 0
    Jac[2,18] = (1.0)
    Jac[2,19] = 0
    Jac[2,20] = 0
    Jac[2,21] = 0
    Jac[2,22] = 0
    Jac[2,23] = 0
    Jac[2,24] = 0
    Jac[2,25] = 0
    Jac[2,26] = 0
    Jac[2,27] = 0
    Jac[2,28] = 0
    Jac[2,29] = 0
    Jac[2,30] = 0
    Jac[2,31] = 0
    Jac[2,32] = 0
    Jac[2,33] = 0
    Jac[2,34] = 0
    Jac[2,35] = 0
    Jac[2,36] = 0
    Jac[2,37] = 0
    Jac[2,38] = 0
    Jac[2,39] = 0
    Jac[2,40] = 0
    Jac[2,41] = 0
    Jac[3,1] = 0
    Jac[3,2] = 0
    Jac[3,3] = 0
    Jac[3,4] = 0
    Jac[3,5] = 0
    Jac[3,6] = 0
    Jac[3,7] = 0
    Jac[3,8] = 0
    Jac[3,9] = 0
    Jac[3,10] = 0
    Jac[3,11] = 0
    Jac[3,12] = 0
    Jac[3,13] = 0
    Jac[3,14] = 0
    Jac[3,15] = 0
    Jac[3,16] = 0
    Jac[3,17] = 0
    Jac[3,18] = 0
    Jac[3,19] = 0
    Jac[3,20] = 0
    Jac[3,21] = 0
    Jac[3,22] = 0
    Jac[3,23] = 0
    Jac[3,24] = 0
    Jac[3,25] = 0
    Jac[3,26] = 0
    Jac[3,27] = 0
    Jac[3,28] = 0
    Jac[3,29] = 0
    Jac[3,30] = JTlp13_23_1
    Jac[3,31] = -RLlp13_33
    Jac[3,32] = 0
    Jac[3,33] = RLlp14_32
    Jac[3,34] = 0
    Jac[3,35] = 0
    Jac[3,36] = 0
    Jac[3,37] = 0
    Jac[3,38] = 0
    Jac[3,39] = 0
    Jac[3,40] = 0
    Jac[3,41] = 0
    Jac[4,1] = 0
    Jac[4,2] = 0
    Jac[4,3] = 0
    Jac[4,4] = 0
    Jac[4,5] = 0
    Jac[4,6] = 0
    Jac[4,7] = 0
    Jac[4,8] = 0
    Jac[4,9] = 0
    Jac[4,10] = 0
    Jac[4,11] = 0
    Jac[4,12] = 0
    Jac[4,13] = 0
    Jac[4,14] = 0
    Jac[4,15] = 0
    Jac[4,16] = 0
    Jac[4,17] = 0
    Jac[4,18] = 0
    Jac[4,19] = 0
    Jac[4,20] = 0
    Jac[4,21] = 0
    Jac[4,22] = 0
    Jac[4,23] = 0
    Jac[4,24] = 0
    Jac[4,25] = 0
    Jac[4,26] = 0
    Jac[4,27] = 0
    Jac[4,28] = 0
    Jac[4,29] = 0
    Jac[4,30] = JTlp13_33_1
    Jac[4,31] = RLlp13_23
    Jac[4,32] = 0
    Jac[4,33] = -RLlp14_22
    Jac[4,34] = 0
    Jac[4,35] = 0
    Jac[4,36] = 0
    Jac[4,37] = 0
    Jac[4,38] = 0
    Jac[4,39] = 0
    Jac[4,40] = 0
    Jac[4,41] = 0

# Number of continuation lines = 0


