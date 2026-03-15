import MBsysPy

def user_JointForces(mbs_data, tsim):
    mbs_data.Qq[1:] = 0.0
    um = mbs_data.user_model

    # ----------------------------------------------------------------
    # 1. BARRE ANTI-ROULIS — effet différentiel gauche/droite
    # ----------------------------------------------------------------
    # Avant : R2 de Bras_sup_AV_G (q8) et R2 de Bras_sup_AV_D (q13)
    C_bar_av = um['FrontSuspension']['C_bar']
    delta_av = mbs_data.q[8] - mbs_data.q[13]
    mbs_data.Qq[8]  += -C_bar_av * delta_av
    mbs_data.Qq[13] +=  C_bar_av * delta_av

    # Arrière : R1 de Bras_sup_AR_G (q24) et R1 de Bras_sup_AR_D (q28)
    C_bar_ar = um['RearSuspension']['C_bar']
    delta_ar = mbs_data.q[24] - mbs_data.q[28]
    mbs_data.Qq[24] += -C_bar_ar * delta_ar
    mbs_data.Qq[28] +=  C_bar_ar * delta_ar

    # ----------------------------------------------------------------
    # 2. STABILISATION CRÉMAILLÈRE — T2 de Bras_Direction (q18)
    # Sans ça, les roues avant oscillent librement pendant la simu
    # ----------------------------------------------------------------
    K_rack = 50000.0  # [N/m]  raideur de rappel virtuelle vers centre
    C_rack =  1000.0  # [N.s/m] amortissement virtuel
    mbs_data.Qq[18] = -K_rack * mbs_data.q[18] - C_rack * mbs_data.qd[18]

    return mbs_data.Qq
