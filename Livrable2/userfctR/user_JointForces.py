
import MBsysPy

def user_JointForces(mbs_data, tsim):

    """Compute the force and torques in the joint.

    It fills the MBsysPy.MbsData.Qq array.

    Parameters
    ----------
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.

    Notes
    -----
    The numpy.ndarray MBsysPy.MbsData.Qq is 1D array with index starting at 1.
    The first index (array[0]) must not be modified. The first index to be
    filled is array[1].

    Returns
    -------
    None
    """


    mbs_data.Qq[1:] = 0.0
    um = mbs_data.user_model

    # --- 1. BARRES ANTI-ROULIS ---
    # On vérifie si les groupes existent pour éviter les plantages
    if hasattr(um, 'FrontSuspension') and 'Joint_9' in mbs_data.joint_id:
        idx = mbs_data.joint_id['Joint_9']
        mbs_data.Qq[idx] = -um.FrontSuspension['C_bar'] * mbs_data.q[idx]
    
    if hasattr(um, 'RearSuspension') and 'Joint_31' in mbs_data.joint_id:
        idx = mbs_data.joint_id['Joint_31']
        mbs_data.Qq[idx] = -um.RearSuspension['C_bar'] * mbs_data.q[idx]

    # --- 2. PROPULSION (Seulement si EquilQ existe) ---
    if hasattr(um, 'EquilQ'):
        # On applique sur les roues arrières : Joint_25 et Joint_30
        for r_name in ['Joint_25', 'Joint_30']:
            if r_name in mbs_data.joint_id:
                mbs_data.Qq[mbs_data.joint_id[r_name]] = um.EquilQ['Qpropulsion']
    
    # --- 3. DIRECTION ( ts > 0.5s ) ---
    if 0.5 < tsim < 5.0:
        if 'Joint_6' in mbs_data.joint_id:
            mbs_data.Qq[mbs_data.joint_id['Joint_6']] = -400.0

    return mbs_data.Qq
    mbs_data.Qq[1:] = 0.0
    um = mbs_data.user_model

    # --- 1. BARRES ANTI-ROULIS ---
    # Avant
    if 'R2_barre_av' in mbs_data.joint_id:
        idx = mbs_data.joint_id['R2_barre_av']
        mbs_data.Qq[idx] = -um.FrontSuspension['C_bar'] * mbs_data.q[idx]
    
    # Arrière
    if 'R2_barre_ar' in mbs_data.joint_id:
        idx = mbs_data.joint_id['R2_barre_ar']
        mbs_data.Qq[idx] = -um.RearSuspension['C_bar'] * mbs_data.q[idx]

    # --- 2. AMORTISSEURS (Joints Prismatiques) ---
    # On applique F = -K*z - C*v pour chaque amortisseur défini dans ton MBS
    for name in ['P_amortisseur_av_g', 'P_amortisseur_av_d', 'P_amortisseur_ar_g', 'P_amortisseur_ar_d']:
        if name in mbs_data.joint_id:
            idx = mbs_data.joint_id[name]
            # Valeurs par défaut (à ajuster dans ton User Model idéalement)
            K = 45000.0; C = 2500.0
            mbs_data.Qq[idx] = -K * mbs_data.q[idx] - C * mbs_data.qd[idx]

    # --- 3. PROPULSION & DIRECTION (Équilibre à t=0) ---
    if tsim == 0.0:
        # Propulsion sur roues arrières
        mbs_data.Qq[mbs_data.joint_id['R2_roue_ar_g']] = um.EquilQ['Qpropulsion']
        mbs_data.Qq[mbs_data.joint_id['R2_roue_ar_d']] = um.EquilQ['Qpropulsion']
        # Direction
        if 'T2_rack' in mbs_data.joint_id:
            mbs_data.Qq[mbs_data.joint_id['T2_rack']] = um.EquilQ['Qsteer']

    return mbs_data.Qq
