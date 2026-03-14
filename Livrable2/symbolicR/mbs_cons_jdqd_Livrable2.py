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
#	==> Generation Date: Sat Mar 14 14:36:55 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Livrable2
#
#	==> Number of joints: 43
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
    S42 = sin(q[42])
    C42 = cos(q[42])
    S43 = sin(q[43])
    C43 = cos(q[43])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    RLjdqd1_22 = s.dpt[2,26]*C17-s.dpt[3,26]*S17
    RLjdqd1_32 = s.dpt[2,26]*S17+s.dpt[3,26]*C17
    ORjdqd1_22 = -RLjdqd1_32*qd[17]
    ORjdqd1_32 = RLjdqd1_22*qd[17]
    Apqpjdqd1_22 = -ORjdqd1_32*qd[17]
    Apqpjdqd1_32 = ORjdqd1_22*qd[17]
    ROjdqd2_22 = S12*S13
    ROjdqd2_32 = -C12*S13
    ROjdqd2_82 = -S12*C13
    ROjdqd2_92 = C12*C13
    ROjdqd2_43 = S13*S14
    ROjdqd2_53 = ROjdqd2_82*S14+C12*C14
    ROjdqd2_63 = ROjdqd2_92*S14+S12*C14
    ROjdqd2_73 = S13*C14
    ROjdqd2_83 = ROjdqd2_82*C14-C12*S14
    ROjdqd2_93 = ROjdqd2_92*C14-S12*S14
    ROjdqd2_44 = ROjdqd2_43*C15-C13*S15
    ROjdqd2_54 = -ROjdqd2_22*S15+ROjdqd2_53*C15
    ROjdqd2_64 = -ROjdqd2_32*S15+ROjdqd2_63*C15
    RLjdqd2_22 = s.dpt[2,22]*C12
    RLjdqd2_32 = s.dpt[2,22]*S12
    OMjdqd2_22 = qd[13]*C12
    OMjdqd2_32 = qd[13]*S12
    ORjdqd2_22 = -RLjdqd2_32*qd[12]
    ORjdqd2_32 = RLjdqd2_22*qd[12]
    Ompqpjdqd2_22 = -qd[12]*qd[13]*S12
    Ompqpjdqd2_32 = qd[12]*qd[13]*C12
    Apqpjdqd2_22 = -ORjdqd2_32*qd[12]
    Apqpjdqd2_32 = ORjdqd2_22*qd[12]
    OMjdqd2_13 = qd[12]+qd[14]*C13
    OMjdqd2_23 = OMjdqd2_22+ROjdqd2_22*qd[14]
    OMjdqd2_33 = OMjdqd2_32+ROjdqd2_32*qd[14]
    Ompqpjdqd2_13 = qd[14]*(OMjdqd2_22*ROjdqd2_32-OMjdqd2_32*ROjdqd2_22)
    Ompqpjdqd2_23 = Ompqpjdqd2_22+qd[14]*(OMjdqd2_32*C13-ROjdqd2_32*qd[12])
    Ompqpjdqd2_33 = Ompqpjdqd2_32+qd[14]*(-OMjdqd2_22*C13+ROjdqd2_22*qd[12])
    OMjdqd2_14 = OMjdqd2_13+ROjdqd2_73*qd[15]
    OMjdqd2_24 = OMjdqd2_23+ROjdqd2_83*qd[15]
    OMjdqd2_34 = OMjdqd2_33+ROjdqd2_93*qd[15]
    Ompqpjdqd2_14 = Ompqpjdqd2_13+qd[15]*(OMjdqd2_23*ROjdqd2_93-OMjdqd2_33*ROjdqd2_83)
    Ompqpjdqd2_24 = Ompqpjdqd2_23+qd[15]*(-OMjdqd2_13*ROjdqd2_93+OMjdqd2_33*ROjdqd2_73)
    Ompqpjdqd2_34 = Ompqpjdqd2_33+qd[15]*(OMjdqd2_13*ROjdqd2_83-OMjdqd2_23*ROjdqd2_73)
    RLjdqd2_15 = ROjdqd2_44*s.dpt[2,23]+ROjdqd2_73*s.dpt[3,23]
    RLjdqd2_25 = ROjdqd2_54*s.dpt[2,23]+ROjdqd2_83*s.dpt[3,23]
    RLjdqd2_35 = ROjdqd2_64*s.dpt[2,23]+ROjdqd2_93*s.dpt[3,23]
    ORjdqd2_15 = OMjdqd2_24*RLjdqd2_35-OMjdqd2_34*RLjdqd2_25
    ORjdqd2_25 = -OMjdqd2_14*RLjdqd2_35+OMjdqd2_34*RLjdqd2_15
    ORjdqd2_35 = OMjdqd2_14*RLjdqd2_25-OMjdqd2_24*RLjdqd2_15
    Apqpjdqd2_15 = OMjdqd2_24*ORjdqd2_35-OMjdqd2_34*ORjdqd2_25+Ompqpjdqd2_24*RLjdqd2_35-Ompqpjdqd2_34*RLjdqd2_25
    Apqpjdqd2_25 = Apqpjdqd2_22-OMjdqd2_14*ORjdqd2_35+OMjdqd2_34*ORjdqd2_15-Ompqpjdqd2_14*RLjdqd2_35+Ompqpjdqd2_34* \
 	  RLjdqd2_15
    Apqpjdqd2_35 = Apqpjdqd2_32+OMjdqd2_14*ORjdqd2_25-OMjdqd2_24*ORjdqd2_15+Ompqpjdqd2_14*RLjdqd2_25-Ompqpjdqd2_24* \
 	  RLjdqd2_15
    jdqd2 = Apqpjdqd1_22-Apqpjdqd2_25
    jdqd3 = Apqpjdqd1_32-Apqpjdqd2_35
    ROjdqd3_53 = C21*C22
    ROjdqd3_63 = S21*C22
    OMjdqd3_23 = -qd[22]*S21
    OMjdqd3_33 = qd[22]*C21
    Ompqpjdqd3_23 = -qd[21]*qd[22]*C21
    Ompqpjdqd3_33 = -qd[21]*qd[22]*S21
    RLjdqd3_14 = -s.dpt[2,31]*S22
    RLjdqd3_24 = ROjdqd3_53*s.dpt[2,31]
    RLjdqd3_34 = ROjdqd3_63*s.dpt[2,31]
    ORjdqd3_14 = OMjdqd3_23*RLjdqd3_34-OMjdqd3_33*RLjdqd3_24
    ORjdqd3_24 = OMjdqd3_33*RLjdqd3_14-RLjdqd3_34*qd[21]
    ORjdqd3_34 = -OMjdqd3_23*RLjdqd3_14+RLjdqd3_24*qd[21]
    Apqpjdqd3_14 = OMjdqd3_23*ORjdqd3_34-OMjdqd3_33*ORjdqd3_24+Ompqpjdqd3_23*RLjdqd3_34-Ompqpjdqd3_33*RLjdqd3_24
    Apqpjdqd3_24 = OMjdqd3_33*ORjdqd3_14-ORjdqd3_34*qd[21]+Ompqpjdqd3_33*RLjdqd3_14
    Apqpjdqd3_34 = -OMjdqd3_23*ORjdqd3_14+ORjdqd3_24*qd[21]-Ompqpjdqd3_23*RLjdqd3_14
    ROjdqd4_22 = S12*S13
    ROjdqd4_32 = -C12*S13
    ROjdqd4_82 = -S12*C13
    ROjdqd4_92 = C12*C13
    ROjdqd4_43 = S13*S14
    ROjdqd4_53 = ROjdqd4_82*S14+C12*C14
    ROjdqd4_63 = ROjdqd4_92*S14+S12*C14
    ROjdqd4_73 = S13*C14
    ROjdqd4_83 = ROjdqd4_82*C14-C12*S14
    ROjdqd4_93 = ROjdqd4_92*C14-S12*S14
    ROjdqd4_14 = ROjdqd4_43*S15+C13*C15
    ROjdqd4_24 = ROjdqd4_22*C15+ROjdqd4_53*S15
    ROjdqd4_34 = ROjdqd4_32*C15+ROjdqd4_63*S15
    ROjdqd4_44 = ROjdqd4_43*C15-C13*S15
    ROjdqd4_54 = -ROjdqd4_22*S15+ROjdqd4_53*C15
    ROjdqd4_64 = -ROjdqd4_32*S15+ROjdqd4_63*C15
    RLjdqd4_22 = s.dpt[2,22]*C12
    RLjdqd4_32 = s.dpt[2,22]*S12
    OMjdqd4_22 = qd[13]*C12
    OMjdqd4_32 = qd[13]*S12
    ORjdqd4_22 = -RLjdqd4_32*qd[12]
    ORjdqd4_32 = RLjdqd4_22*qd[12]
    Ompqpjdqd4_22 = -qd[12]*qd[13]*S12
    Ompqpjdqd4_32 = qd[12]*qd[13]*C12
    Apqpjdqd4_22 = -ORjdqd4_32*qd[12]
    Apqpjdqd4_32 = ORjdqd4_22*qd[12]
    OMjdqd4_13 = qd[12]+qd[14]*C13
    OMjdqd4_23 = OMjdqd4_22+ROjdqd4_22*qd[14]
    OMjdqd4_33 = OMjdqd4_32+ROjdqd4_32*qd[14]
    Ompqpjdqd4_13 = qd[14]*(OMjdqd4_22*ROjdqd4_32-OMjdqd4_32*ROjdqd4_22)
    Ompqpjdqd4_23 = Ompqpjdqd4_22+qd[14]*(OMjdqd4_32*C13-ROjdqd4_32*qd[12])
    Ompqpjdqd4_33 = Ompqpjdqd4_32+qd[14]*(-OMjdqd4_22*C13+ROjdqd4_22*qd[12])
    OMjdqd4_14 = OMjdqd4_13+ROjdqd4_73*qd[15]
    OMjdqd4_24 = OMjdqd4_23+ROjdqd4_83*qd[15]
    OMjdqd4_34 = OMjdqd4_33+ROjdqd4_93*qd[15]
    Ompqpjdqd4_14 = Ompqpjdqd4_13+qd[15]*(OMjdqd4_23*ROjdqd4_93-OMjdqd4_33*ROjdqd4_83)
    Ompqpjdqd4_24 = Ompqpjdqd4_23+qd[15]*(-OMjdqd4_13*ROjdqd4_93+OMjdqd4_33*ROjdqd4_73)
    Ompqpjdqd4_34 = Ompqpjdqd4_33+qd[15]*(OMjdqd4_13*ROjdqd4_83-OMjdqd4_23*ROjdqd4_73)
    RLjdqd4_15 = ROjdqd4_14*s.dpt[1,24]+ROjdqd4_44*s.dpt[2,24]+ROjdqd4_73*s.dpt[3,24]
    RLjdqd4_25 = ROjdqd4_24*s.dpt[1,24]+ROjdqd4_54*s.dpt[2,24]+ROjdqd4_83*s.dpt[3,24]
    RLjdqd4_35 = ROjdqd4_34*s.dpt[1,24]+ROjdqd4_64*s.dpt[2,24]+ROjdqd4_93*s.dpt[3,24]
    ORjdqd4_15 = OMjdqd4_24*RLjdqd4_35-OMjdqd4_34*RLjdqd4_25
    ORjdqd4_25 = -OMjdqd4_14*RLjdqd4_35+OMjdqd4_34*RLjdqd4_15
    ORjdqd4_35 = OMjdqd4_14*RLjdqd4_25-OMjdqd4_24*RLjdqd4_15
    Apqpjdqd4_15 = OMjdqd4_24*ORjdqd4_35-OMjdqd4_34*ORjdqd4_25+Ompqpjdqd4_24*RLjdqd4_35-Ompqpjdqd4_34*RLjdqd4_25
    Apqpjdqd4_25 = Apqpjdqd4_22-OMjdqd4_14*ORjdqd4_35+OMjdqd4_34*ORjdqd4_15-Ompqpjdqd4_14*RLjdqd4_35+Ompqpjdqd4_34* \
 	  RLjdqd4_15
    Apqpjdqd4_35 = Apqpjdqd4_32+OMjdqd4_14*ORjdqd4_25-OMjdqd4_24*ORjdqd4_15+Ompqpjdqd4_14*RLjdqd4_25-Ompqpjdqd4_24* \
 	  RLjdqd4_15
    jdqd4 = Apqpjdqd3_14-Apqpjdqd4_15
    jdqd5 = Apqpjdqd3_24-Apqpjdqd4_25
    jdqd6 = Apqpjdqd3_34-Apqpjdqd4_35
    ROjdqd5_12 = C23*C24-S23*S24
    ROjdqd5_32 = -C23*S24-S23*C24
    ROjdqd5_72 = C23*S24+S23*C24
    ROjdqd5_92 = C23*C24-S23*S24
    ROjdqd5_43 = ROjdqd5_72*S25
    ROjdqd5_63 = ROjdqd5_92*S25
    ROjdqd5_73 = ROjdqd5_72*C25
    ROjdqd5_93 = ROjdqd5_92*C25
    ROjdqd5_74 = ROjdqd5_12*S26+ROjdqd5_73*C26
    ROjdqd5_84 = -S25*C26
    ROjdqd5_94 = ROjdqd5_32*S26+ROjdqd5_93*C26
    RLjdqd5_12 = s.dpt[1,32]*C23+s.dpt[3,32]*S23
    RLjdqd5_32 = -s.dpt[1,32]*S23+s.dpt[3,32]*C23
    OMjdqd5_22 = qd[23]+qd[24]
    ORjdqd5_12 = RLjdqd5_32*qd[23]
    ORjdqd5_32 = -RLjdqd5_12*qd[23]
    Apqpjdqd5_12 = ORjdqd5_32*qd[23]
    Apqpjdqd5_32 = -ORjdqd5_12*qd[23]
    OMjdqd5_13 = ROjdqd5_12*qd[25]
    OMjdqd5_33 = ROjdqd5_32*qd[25]
    Ompqpjdqd5_13 = OMjdqd5_22*ROjdqd5_32*qd[25]
    Ompqpjdqd5_33 = -OMjdqd5_22*ROjdqd5_12*qd[25]
    OMjdqd5_14 = OMjdqd5_13+ROjdqd5_43*qd[26]
    OMjdqd5_24 = OMjdqd5_22+qd[26]*C25
    OMjdqd5_34 = OMjdqd5_33+ROjdqd5_63*qd[26]
    Ompqpjdqd5_14 = Ompqpjdqd5_13+qd[26]*(OMjdqd5_22*ROjdqd5_63-OMjdqd5_33*C25)
    Ompqpjdqd5_24 = qd[26]*(-OMjdqd5_13*ROjdqd5_63+OMjdqd5_33*ROjdqd5_43)
    Ompqpjdqd5_34 = Ompqpjdqd5_33+qd[26]*(OMjdqd5_13*C25-OMjdqd5_22*ROjdqd5_43)
    RLjdqd5_15 = ROjdqd5_74*s.dpt[3,35]
    RLjdqd5_25 = ROjdqd5_84*s.dpt[3,35]
    RLjdqd5_35 = ROjdqd5_94*s.dpt[3,35]
    ORjdqd5_15 = OMjdqd5_24*RLjdqd5_35-OMjdqd5_34*RLjdqd5_25
    ORjdqd5_25 = -OMjdqd5_14*RLjdqd5_35+OMjdqd5_34*RLjdqd5_15
    ORjdqd5_35 = OMjdqd5_14*RLjdqd5_25-OMjdqd5_24*RLjdqd5_15
    Apqpjdqd5_15 = Apqpjdqd5_12+OMjdqd5_24*ORjdqd5_35-OMjdqd5_34*ORjdqd5_25+Ompqpjdqd5_24*RLjdqd5_35-Ompqpjdqd5_34* \
 	  RLjdqd5_25
    Apqpjdqd5_25 = -OMjdqd5_14*ORjdqd5_35+OMjdqd5_34*ORjdqd5_15-Ompqpjdqd5_14*RLjdqd5_35+Ompqpjdqd5_34*RLjdqd5_15
    Apqpjdqd5_35 = Apqpjdqd5_32+OMjdqd5_14*ORjdqd5_25-OMjdqd5_24*ORjdqd5_15+Ompqpjdqd5_14*RLjdqd5_25-Ompqpjdqd5_24* \
 	  RLjdqd5_15
    RLjdqd6_22 = s.dpt[2,21]*C12
    RLjdqd6_32 = s.dpt[2,21]*S12
    ORjdqd6_22 = -RLjdqd6_32*qd[12]
    ORjdqd6_32 = RLjdqd6_22*qd[12]
    Apqpjdqd6_22 = -ORjdqd6_32*qd[12]
    Apqpjdqd6_32 = ORjdqd6_22*qd[12]
    jdqd8 = Apqpjdqd5_25-Apqpjdqd6_22
    jdqd9 = Apqpjdqd5_35-Apqpjdqd6_32
    ROjdqd7_42 = S23*S27
    ROjdqd7_62 = C23*S27
    ROjdqd7_72 = S23*C27
    ROjdqd7_92 = C23*C27
    ROjdqd7_73 = ROjdqd7_72*C28+C23*S28
    ROjdqd7_83 = -S27*C28
    ROjdqd7_93 = ROjdqd7_92*C28-S23*S28
    RLjdqd7_12 = s.dpt[3,33]*S23
    RLjdqd7_32 = s.dpt[3,33]*C23
    OMjdqd7_12 = qd[27]*C23
    OMjdqd7_32 = -qd[27]*S23
    ORjdqd7_12 = RLjdqd7_32*qd[23]
    ORjdqd7_32 = -RLjdqd7_12*qd[23]
    Ompqpjdqd7_12 = -qd[23]*qd[27]*S23
    Ompqpjdqd7_32 = -qd[23]*qd[27]*C23
    Apqpjdqd7_12 = ORjdqd7_32*qd[23]
    Apqpjdqd7_32 = -ORjdqd7_12*qd[23]
    OMjdqd7_13 = OMjdqd7_12+ROjdqd7_42*qd[28]
    OMjdqd7_23 = qd[23]+qd[28]*C27
    OMjdqd7_33 = OMjdqd7_32+ROjdqd7_62*qd[28]
    Ompqpjdqd7_13 = Ompqpjdqd7_12+qd[28]*(-OMjdqd7_32*C27+ROjdqd7_62*qd[23])
    Ompqpjdqd7_23 = qd[28]*(-OMjdqd7_12*ROjdqd7_62+OMjdqd7_32*ROjdqd7_42)
    Ompqpjdqd7_33 = Ompqpjdqd7_32+qd[28]*(OMjdqd7_12*C27-ROjdqd7_42*qd[23])
    RLjdqd7_14 = ROjdqd7_73*s.dpt[3,36]
    RLjdqd7_24 = ROjdqd7_83*s.dpt[3,36]
    RLjdqd7_34 = ROjdqd7_93*s.dpt[3,36]
    ORjdqd7_14 = OMjdqd7_23*RLjdqd7_34-OMjdqd7_33*RLjdqd7_24
    ORjdqd7_24 = -OMjdqd7_13*RLjdqd7_34+OMjdqd7_33*RLjdqd7_14
    ORjdqd7_34 = OMjdqd7_13*RLjdqd7_24-OMjdqd7_23*RLjdqd7_14
    Apqpjdqd7_14 = Apqpjdqd7_12+OMjdqd7_23*ORjdqd7_34-OMjdqd7_33*ORjdqd7_24+Ompqpjdqd7_23*RLjdqd7_34-Ompqpjdqd7_33* \
 	  RLjdqd7_24
    Apqpjdqd7_24 = -OMjdqd7_13*ORjdqd7_34+OMjdqd7_33*ORjdqd7_14-Ompqpjdqd7_13*RLjdqd7_34+Ompqpjdqd7_33*RLjdqd7_14
    Apqpjdqd7_34 = Apqpjdqd7_32+OMjdqd7_13*ORjdqd7_24-OMjdqd7_23*ORjdqd7_14+Ompqpjdqd7_13*RLjdqd7_24-Ompqpjdqd7_23* \
 	  RLjdqd7_14
    RLjdqd8_22 = s.dpt[2,17]*C7
    RLjdqd8_32 = s.dpt[2,17]*S7
    ORjdqd8_22 = -RLjdqd8_32*qd[7]
    ORjdqd8_32 = RLjdqd8_22*qd[7]
    Apqpjdqd8_22 = -ORjdqd8_32*qd[7]
    Apqpjdqd8_32 = ORjdqd8_22*qd[7]
    jdqd11 = Apqpjdqd7_24-Apqpjdqd8_22
    jdqd12 = Apqpjdqd7_34-Apqpjdqd8_32
    ROjdqd9_53 = C19*C20
    ROjdqd9_63 = S19*C20
    OMjdqd9_23 = -qd[20]*S19
    OMjdqd9_33 = qd[20]*C19
    Ompqpjdqd9_23 = -qd[19]*qd[20]*C19
    Ompqpjdqd9_33 = -qd[19]*qd[20]*S19
    RLjdqd9_14 = -s.dpt[2,30]*S20
    RLjdqd9_24 = ROjdqd9_53*s.dpt[2,30]
    RLjdqd9_34 = ROjdqd9_63*s.dpt[2,30]
    ORjdqd9_14 = OMjdqd9_23*RLjdqd9_34-OMjdqd9_33*RLjdqd9_24
    ORjdqd9_24 = OMjdqd9_33*RLjdqd9_14-RLjdqd9_34*qd[19]
    ORjdqd9_34 = -OMjdqd9_23*RLjdqd9_14+RLjdqd9_24*qd[19]
    Apqpjdqd9_14 = OMjdqd9_23*ORjdqd9_34-OMjdqd9_33*ORjdqd9_24+Ompqpjdqd9_23*RLjdqd9_34-Ompqpjdqd9_33*RLjdqd9_24
    Apqpjdqd9_24 = OMjdqd9_33*ORjdqd9_14-ORjdqd9_34*qd[19]+Ompqpjdqd9_33*RLjdqd9_14
    Apqpjdqd9_34 = -OMjdqd9_23*ORjdqd9_14+ORjdqd9_24*qd[19]-Ompqpjdqd9_23*RLjdqd9_14
    ROjdqd10_22 = S7*S8
    ROjdqd10_32 = -C7*S8
    ROjdqd10_82 = -S7*C8
    ROjdqd10_92 = C7*C8
    ROjdqd10_43 = S8*S9
    ROjdqd10_53 = ROjdqd10_82*S9+C7*C9
    ROjdqd10_63 = ROjdqd10_92*S9+S7*C9
    ROjdqd10_73 = S8*C9
    ROjdqd10_83 = ROjdqd10_82*C9-C7*S9
    ROjdqd10_93 = ROjdqd10_92*C9-S7*S9
    ROjdqd10_14 = ROjdqd10_43*S10+C10*C8
    ROjdqd10_24 = ROjdqd10_22*C10+ROjdqd10_53*S10
    ROjdqd10_34 = ROjdqd10_32*C10+ROjdqd10_63*S10
    ROjdqd10_44 = ROjdqd10_43*C10-S10*C8
    ROjdqd10_54 = -ROjdqd10_22*S10+ROjdqd10_53*C10
    ROjdqd10_64 = -ROjdqd10_32*S10+ROjdqd10_63*C10
    RLjdqd10_22 = s.dpt[2,16]*C7
    RLjdqd10_32 = s.dpt[2,16]*S7
    OMjdqd10_22 = qd[8]*C7
    OMjdqd10_32 = qd[8]*S7
    ORjdqd10_22 = -RLjdqd10_32*qd[7]
    ORjdqd10_32 = RLjdqd10_22*qd[7]
    Ompqpjdqd10_22 = -qd[7]*qd[8]*S7
    Ompqpjdqd10_32 = qd[7]*qd[8]*C7
    Apqpjdqd10_22 = -ORjdqd10_32*qd[7]
    Apqpjdqd10_32 = ORjdqd10_22*qd[7]
    OMjdqd10_13 = qd[7]+qd[9]*C8
    OMjdqd10_23 = OMjdqd10_22+ROjdqd10_22*qd[9]
    OMjdqd10_33 = OMjdqd10_32+ROjdqd10_32*qd[9]
    Ompqpjdqd10_13 = qd[9]*(OMjdqd10_22*ROjdqd10_32-OMjdqd10_32*ROjdqd10_22)
    Ompqpjdqd10_23 = Ompqpjdqd10_22+qd[9]*(OMjdqd10_32*C8-ROjdqd10_32*qd[7])
    Ompqpjdqd10_33 = Ompqpjdqd10_32+qd[9]*(-OMjdqd10_22*C8+ROjdqd10_22*qd[7])
    OMjdqd10_14 = OMjdqd10_13+ROjdqd10_73*qd[10]
    OMjdqd10_24 = OMjdqd10_23+ROjdqd10_83*qd[10]
    OMjdqd10_34 = OMjdqd10_33+ROjdqd10_93*qd[10]
    Ompqpjdqd10_14 = Ompqpjdqd10_13+qd[10]*(OMjdqd10_23*ROjdqd10_93-OMjdqd10_33*ROjdqd10_83)
    Ompqpjdqd10_24 = Ompqpjdqd10_23+qd[10]*(-OMjdqd10_13*ROjdqd10_93+OMjdqd10_33*ROjdqd10_73)
    Ompqpjdqd10_34 = Ompqpjdqd10_33+qd[10]*(OMjdqd10_13*ROjdqd10_83-OMjdqd10_23*ROjdqd10_73)
    RLjdqd10_15 = ROjdqd10_14*s.dpt[1,18]+ROjdqd10_44*s.dpt[2,18]+ROjdqd10_73*s.dpt[3,18]
    RLjdqd10_25 = ROjdqd10_24*s.dpt[1,18]+ROjdqd10_54*s.dpt[2,18]+ROjdqd10_83*s.dpt[3,18]
    RLjdqd10_35 = ROjdqd10_34*s.dpt[1,18]+ROjdqd10_64*s.dpt[2,18]+ROjdqd10_93*s.dpt[3,18]
    ORjdqd10_15 = OMjdqd10_24*RLjdqd10_35-OMjdqd10_34*RLjdqd10_25
    ORjdqd10_25 = -OMjdqd10_14*RLjdqd10_35+OMjdqd10_34*RLjdqd10_15
    ORjdqd10_35 = OMjdqd10_14*RLjdqd10_25-OMjdqd10_24*RLjdqd10_15
    Apqpjdqd10_15 = OMjdqd10_24*ORjdqd10_35-OMjdqd10_34*ORjdqd10_25+Ompqpjdqd10_24*RLjdqd10_35-Ompqpjdqd10_34* \
 	  RLjdqd10_25
    Apqpjdqd10_25 = Apqpjdqd10_22-OMjdqd10_14*ORjdqd10_35+OMjdqd10_34*ORjdqd10_15-Ompqpjdqd10_14*RLjdqd10_35+ \
 	  Ompqpjdqd10_34*RLjdqd10_15
    Apqpjdqd10_35 = Apqpjdqd10_32+OMjdqd10_14*ORjdqd10_25-OMjdqd10_24*ORjdqd10_15+Ompqpjdqd10_14*RLjdqd10_25- \
 	  Ompqpjdqd10_24*RLjdqd10_15
    jdqd13 = -Apqpjdqd10_15+Apqpjdqd9_14
    jdqd14 = -Apqpjdqd10_25+Apqpjdqd9_24
    jdqd15 = -Apqpjdqd10_35+Apqpjdqd9_34
    RLjdqd11_22 = s.dpt[2,37]*C29-s.dpt[3,37]*S29
    RLjdqd11_32 = s.dpt[2,37]*S29+s.dpt[3,37]*C29
    ORjdqd11_22 = -RLjdqd11_32*qd[29]
    ORjdqd11_32 = RLjdqd11_22*qd[29]
    Apqpjdqd11_22 = -ORjdqd11_32*qd[29]
    Apqpjdqd11_32 = ORjdqd11_22*qd[29]
    ROjdqd12_22 = S7*S8
    ROjdqd12_32 = -C7*S8
    ROjdqd12_82 = -S7*C8
    ROjdqd12_92 = C7*C8
    ROjdqd12_43 = S8*S9
    ROjdqd12_53 = ROjdqd12_82*S9+C7*C9
    ROjdqd12_63 = ROjdqd12_92*S9+S7*C9
    ROjdqd12_73 = S8*C9
    ROjdqd12_83 = ROjdqd12_82*C9-C7*S9
    ROjdqd12_93 = ROjdqd12_92*C9-S7*S9
    ROjdqd12_44 = ROjdqd12_43*C10-S10*C8
    ROjdqd12_54 = -ROjdqd12_22*S10+ROjdqd12_53*C10
    ROjdqd12_64 = -ROjdqd12_32*S10+ROjdqd12_63*C10
    RLjdqd12_22 = s.dpt[2,16]*C7
    RLjdqd12_32 = s.dpt[2,16]*S7
    OMjdqd12_22 = qd[8]*C7
    OMjdqd12_32 = qd[8]*S7
    ORjdqd12_22 = -RLjdqd12_32*qd[7]
    ORjdqd12_32 = RLjdqd12_22*qd[7]
    Ompqpjdqd12_22 = -qd[7]*qd[8]*S7
    Ompqpjdqd12_32 = qd[7]*qd[8]*C7
    Apqpjdqd12_22 = -ORjdqd12_32*qd[7]
    Apqpjdqd12_32 = ORjdqd12_22*qd[7]
    OMjdqd12_13 = qd[7]+qd[9]*C8
    OMjdqd12_23 = OMjdqd12_22+ROjdqd12_22*qd[9]
    OMjdqd12_33 = OMjdqd12_32+ROjdqd12_32*qd[9]
    Ompqpjdqd12_13 = qd[9]*(OMjdqd12_22*ROjdqd12_32-OMjdqd12_32*ROjdqd12_22)
    Ompqpjdqd12_23 = Ompqpjdqd12_22+qd[9]*(OMjdqd12_32*C8-ROjdqd12_32*qd[7])
    Ompqpjdqd12_33 = Ompqpjdqd12_32+qd[9]*(-OMjdqd12_22*C8+ROjdqd12_22*qd[7])
    OMjdqd12_14 = OMjdqd12_13+ROjdqd12_73*qd[10]
    OMjdqd12_24 = OMjdqd12_23+ROjdqd12_83*qd[10]
    OMjdqd12_34 = OMjdqd12_33+ROjdqd12_93*qd[10]
    Ompqpjdqd12_14 = Ompqpjdqd12_13+qd[10]*(OMjdqd12_23*ROjdqd12_93-OMjdqd12_33*ROjdqd12_83)
    Ompqpjdqd12_24 = Ompqpjdqd12_23+qd[10]*(-OMjdqd12_13*ROjdqd12_93+OMjdqd12_33*ROjdqd12_73)
    Ompqpjdqd12_34 = Ompqpjdqd12_33+qd[10]*(OMjdqd12_13*ROjdqd12_83-OMjdqd12_23*ROjdqd12_73)
    RLjdqd12_15 = ROjdqd12_44*s.dpt[2,20]+ROjdqd12_73*s.dpt[3,20]
    RLjdqd12_25 = ROjdqd12_54*s.dpt[2,20]+ROjdqd12_83*s.dpt[3,20]
    RLjdqd12_35 = ROjdqd12_64*s.dpt[2,20]+ROjdqd12_93*s.dpt[3,20]
    ORjdqd12_15 = OMjdqd12_24*RLjdqd12_35-OMjdqd12_34*RLjdqd12_25
    ORjdqd12_25 = -OMjdqd12_14*RLjdqd12_35+OMjdqd12_34*RLjdqd12_15
    ORjdqd12_35 = OMjdqd12_14*RLjdqd12_25-OMjdqd12_24*RLjdqd12_15
    Apqpjdqd12_15 = OMjdqd12_24*ORjdqd12_35-OMjdqd12_34*ORjdqd12_25+Ompqpjdqd12_24*RLjdqd12_35-Ompqpjdqd12_34* \
 	  RLjdqd12_25
    Apqpjdqd12_25 = Apqpjdqd12_22-OMjdqd12_14*ORjdqd12_35+OMjdqd12_34*ORjdqd12_15-Ompqpjdqd12_14*RLjdqd12_35+ \
 	  Ompqpjdqd12_34*RLjdqd12_15
    Apqpjdqd12_35 = Apqpjdqd12_32+OMjdqd12_14*ORjdqd12_25-OMjdqd12_24*ORjdqd12_15+Ompqpjdqd12_14*RLjdqd12_25- \
 	  Ompqpjdqd12_24*RLjdqd12_15
    jdqd17 = Apqpjdqd11_22-Apqpjdqd12_25
    jdqd18 = Apqpjdqd11_32-Apqpjdqd12_35
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
    RLjdqd15_22 = s.dpt[2,49]*C37-s.dpt[3,49]*S37
    RLjdqd15_32 = s.dpt[2,49]*S37+s.dpt[3,49]*C37
    ORjdqd15_22 = -RLjdqd15_32*qd[37]
    ORjdqd15_32 = RLjdqd15_22*qd[37]
    Apqpjdqd15_22 = -ORjdqd15_32*qd[37]
    Apqpjdqd15_32 = ORjdqd15_22*qd[37]
    ROjdqd16_52 = C34*C35-S34*S35
    ROjdqd16_62 = C34*S35+S34*C35
    ROjdqd16_82 = -C34*S35-S34*C35
    ROjdqd16_92 = C34*C35-S34*S35
    RLjdqd16_22 = s.dpt[2,45]*C34
    RLjdqd16_32 = s.dpt[2,45]*S34
    OMjdqd16_12 = qd[34]+qd[35]
    ORjdqd16_22 = -RLjdqd16_32*qd[34]
    ORjdqd16_32 = RLjdqd16_22*qd[34]
    Apqpjdqd16_22 = -ORjdqd16_32*qd[34]
    Apqpjdqd16_32 = ORjdqd16_22*qd[34]
    RLjdqd16_23 = ROjdqd16_52*s.dpt[2,47]+ROjdqd16_82*s.dpt[3,47]
    RLjdqd16_33 = ROjdqd16_62*s.dpt[2,47]+ROjdqd16_92*s.dpt[3,47]
    ORjdqd16_23 = -OMjdqd16_12*RLjdqd16_33
    ORjdqd16_33 = OMjdqd16_12*RLjdqd16_23
    Apqpjdqd16_23 = Apqpjdqd16_22-OMjdqd16_12*ORjdqd16_33
    Apqpjdqd16_33 = Apqpjdqd16_32+OMjdqd16_12*ORjdqd16_23
    jdqd23 = Apqpjdqd15_22-Apqpjdqd16_23
    jdqd24 = Apqpjdqd15_32-Apqpjdqd16_33
    ROjdqd17_42 = S38*S39
    ROjdqd17_62 = C38*S39
    ROjdqd17_72 = S38*C39
    ROjdqd17_92 = C38*C39
    ROjdqd17_73 = ROjdqd17_72*C40+C38*S40
    ROjdqd17_83 = -S39*C40
    ROjdqd17_93 = ROjdqd17_92*C40-S38*S40
    RLjdqd17_12 = s.dpt[1,51]*C38
    RLjdqd17_32 = -s.dpt[1,51]*S38
    OMjdqd17_12 = qd[39]*C38
    OMjdqd17_32 = -qd[39]*S38
    ORjdqd17_12 = RLjdqd17_32*qd[38]
    ORjdqd17_32 = -RLjdqd17_12*qd[38]
    Ompqpjdqd17_12 = -qd[38]*qd[39]*S38
    Ompqpjdqd17_32 = -qd[38]*qd[39]*C38
    Apqpjdqd17_12 = ORjdqd17_32*qd[38]
    Apqpjdqd17_32 = -ORjdqd17_12*qd[38]
    OMjdqd17_13 = OMjdqd17_12+ROjdqd17_42*qd[40]
    OMjdqd17_23 = qd[38]+qd[40]*C39
    OMjdqd17_33 = OMjdqd17_32+ROjdqd17_62*qd[40]
    Ompqpjdqd17_13 = Ompqpjdqd17_12+qd[40]*(-OMjdqd17_32*C39+ROjdqd17_62*qd[38])
    Ompqpjdqd17_23 = qd[40]*(-OMjdqd17_12*ROjdqd17_62+OMjdqd17_32*ROjdqd17_42)
    Ompqpjdqd17_33 = Ompqpjdqd17_32+qd[40]*(OMjdqd17_12*C39-ROjdqd17_42*qd[38])
    RLjdqd17_14 = ROjdqd17_73*s.dpt[3,53]
    RLjdqd17_24 = ROjdqd17_83*s.dpt[3,53]
    RLjdqd17_34 = ROjdqd17_93*s.dpt[3,53]
    ORjdqd17_14 = OMjdqd17_23*RLjdqd17_34-OMjdqd17_33*RLjdqd17_24
    ORjdqd17_24 = -OMjdqd17_13*RLjdqd17_34+OMjdqd17_33*RLjdqd17_14
    ORjdqd17_34 = OMjdqd17_13*RLjdqd17_24-OMjdqd17_23*RLjdqd17_14
    Apqpjdqd17_14 = Apqpjdqd17_12+OMjdqd17_23*ORjdqd17_34-OMjdqd17_33*ORjdqd17_24+Ompqpjdqd17_23*RLjdqd17_34- \
 	  Ompqpjdqd17_33*RLjdqd17_24
    Apqpjdqd17_24 = -OMjdqd17_13*ORjdqd17_34+OMjdqd17_33*ORjdqd17_14-Ompqpjdqd17_13*RLjdqd17_34+Ompqpjdqd17_33* \
 	  RLjdqd17_14
    Apqpjdqd17_34 = Apqpjdqd17_32+OMjdqd17_13*ORjdqd17_24-OMjdqd17_23*ORjdqd17_14+Ompqpjdqd17_13*RLjdqd17_24- \
 	  Ompqpjdqd17_23*RLjdqd17_14
    RLjdqd18_22 = s.dpt[2,40]*C30
    RLjdqd18_32 = s.dpt[2,40]*S30
    ORjdqd18_22 = -RLjdqd18_32*qd[30]
    ORjdqd18_32 = RLjdqd18_22*qd[30]
    Apqpjdqd18_22 = -ORjdqd18_32*qd[30]
    Apqpjdqd18_32 = ORjdqd18_22*qd[30]
    jdqd26 = Apqpjdqd17_24-Apqpjdqd18_22
    jdqd27 = Apqpjdqd17_34-Apqpjdqd18_32
    ROjdqd19_12 = C38*C41-S38*S41
    ROjdqd19_32 = -C38*S41-S38*C41
    ROjdqd19_72 = C38*S41+S38*C41
    ROjdqd19_92 = C38*C41-S38*S41
    ROjdqd19_43 = ROjdqd19_72*S42
    ROjdqd19_63 = ROjdqd19_92*S42
    ROjdqd19_73 = ROjdqd19_72*C42
    ROjdqd19_93 = ROjdqd19_92*C42
    ROjdqd19_74 = ROjdqd19_12*S43+ROjdqd19_73*C43
    ROjdqd19_84 = -S42*C43
    ROjdqd19_94 = ROjdqd19_32*S43+ROjdqd19_93*C43
    OMjdqd19_22 = qd[38]+qd[41]
    RLjdqd19_13 = ROjdqd19_12*s.dpt[1,54]
    RLjdqd19_33 = ROjdqd19_32*s.dpt[1,54]
    OMjdqd19_13 = ROjdqd19_12*qd[42]
    OMjdqd19_33 = ROjdqd19_32*qd[42]
    ORjdqd19_13 = OMjdqd19_22*RLjdqd19_33
    ORjdqd19_33 = -OMjdqd19_22*RLjdqd19_13
    Ompqpjdqd19_13 = OMjdqd19_22*ROjdqd19_32*qd[42]
    Ompqpjdqd19_33 = -OMjdqd19_22*ROjdqd19_12*qd[42]
    Apqpjdqd19_13 = OMjdqd19_22*ORjdqd19_33
    Apqpjdqd19_33 = -OMjdqd19_22*ORjdqd19_13
    OMjdqd19_14 = OMjdqd19_13+ROjdqd19_43*qd[43]
    OMjdqd19_24 = OMjdqd19_22+qd[43]*C42
    OMjdqd19_34 = OMjdqd19_33+ROjdqd19_63*qd[43]
    Ompqpjdqd19_14 = Ompqpjdqd19_13+qd[43]*(OMjdqd19_22*ROjdqd19_63-OMjdqd19_33*C42)
    Ompqpjdqd19_24 = qd[43]*(-OMjdqd19_13*ROjdqd19_63+OMjdqd19_33*ROjdqd19_43)
    Ompqpjdqd19_34 = Ompqpjdqd19_33+qd[43]*(OMjdqd19_13*C42-OMjdqd19_22*ROjdqd19_43)
    RLjdqd19_15 = ROjdqd19_74*s.dpt[3,55]
    RLjdqd19_25 = ROjdqd19_84*s.dpt[3,55]
    RLjdqd19_35 = ROjdqd19_94*s.dpt[3,55]
    ORjdqd19_15 = OMjdqd19_24*RLjdqd19_35-OMjdqd19_34*RLjdqd19_25
    ORjdqd19_25 = -OMjdqd19_14*RLjdqd19_35+OMjdqd19_34*RLjdqd19_15
    ORjdqd19_35 = OMjdqd19_14*RLjdqd19_25-OMjdqd19_24*RLjdqd19_15
    Apqpjdqd19_15 = Apqpjdqd19_13+OMjdqd19_24*ORjdqd19_35-OMjdqd19_34*ORjdqd19_25+Ompqpjdqd19_24*RLjdqd19_35- \
 	  Ompqpjdqd19_34*RLjdqd19_25
    Apqpjdqd19_25 = -OMjdqd19_14*ORjdqd19_35+OMjdqd19_34*ORjdqd19_15-Ompqpjdqd19_14*RLjdqd19_35+Ompqpjdqd19_34* \
 	  RLjdqd19_15
    Apqpjdqd19_35 = Apqpjdqd19_33+OMjdqd19_14*ORjdqd19_25-OMjdqd19_24*ORjdqd19_15+Ompqpjdqd19_14*RLjdqd19_25- \
 	  Ompqpjdqd19_24*RLjdqd19_15
    RLjdqd20_22 = s.dpt[2,46]*C34
    RLjdqd20_32 = s.dpt[2,46]*S34
    ORjdqd20_22 = -RLjdqd20_32*qd[34]
    ORjdqd20_32 = RLjdqd20_22*qd[34]
    Apqpjdqd20_22 = -ORjdqd20_32*qd[34]
    Apqpjdqd20_32 = ORjdqd20_22*qd[34]
    jdqd29 = Apqpjdqd19_25-Apqpjdqd20_22
    jdqd30 = Apqpjdqd19_35-Apqpjdqd20_32
    Jdqd[1] = -Apqpjdqd2_15
    Jdqd[2] = jdqd2
    Jdqd[3] = jdqd3
    Jdqd[4] = jdqd4
    Jdqd[5] = jdqd5
    Jdqd[6] = jdqd6
    Jdqd[7] = Apqpjdqd5_15
    Jdqd[8] = jdqd8
    Jdqd[9] = jdqd9
    Jdqd[10] = Apqpjdqd7_14
    Jdqd[11] = jdqd11
    Jdqd[12] = jdqd12
    Jdqd[13] = jdqd13
    Jdqd[14] = jdqd14
    Jdqd[15] = jdqd15
    Jdqd[16] = -Apqpjdqd12_15
    Jdqd[17] = jdqd17
    Jdqd[18] = jdqd18
    Jdqd[19] = jdqd20
    Jdqd[20] = jdqd21
    Jdqd[21] = jdqd23
    Jdqd[22] = jdqd24
    Jdqd[23] = Apqpjdqd17_14
    Jdqd[24] = jdqd26
    Jdqd[25] = jdqd27
    Jdqd[26] = Apqpjdqd19_15
    Jdqd[27] = jdqd29
    Jdqd[28] = jdqd30

# Number of continuation lines = 1


