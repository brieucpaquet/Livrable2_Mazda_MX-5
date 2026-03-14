# -*- coding: utf-8 -*-
"""Module for defining user function required to compute external forces."""
# Author: Robotran Team
# (c) Universite catholique de Louvain, 2019

# -*- coding: utf-8 -*-
import numpy as np

def user_ExtForces(PxF, RxF, VxF, OMxF, AxF, OMPxF, mbs_data, tsim, ixF):

    """Compute an user-specified external force.

    Parameters
    ----------
    PxF : numpy.ndarray
        Position vector (index starting at 1) of the force sensor expressed in
        the inertial frame: PxF[1:4] = [P_x, P_y, P_z]
    RxF : numpy.ndarray
        Rotation matrix (index starting at 1) from the inertial frame to the
        force sensor frame: Frame_sensor = RxF[1:4,1:4] * Frame_inertial
    VxF : numpy.ndarray
        Velocity vector (index starting at 1) of the force sensor expressed in
        the inertial frame: VxF[1:4] = [V_x, V_y, V_z]
    OMxF : numpy.ndarray
        Angular velocity vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMxF[1:4] = [OM_x, OM_y, OM_z]
    AxF : numpy.ndarray
        Acceleration vector (index starting at 1) of the force sensor expressed
        in the inertial frame: AxF[1:4] = [A_x, A_y, A_z]
    OMPxF : numpy.ndarray
        Angular acceleration vector (index starting at 1) of the force sensor
        expressed in the inertial frame: OMPxF[1:4] = [OMP_x, OMP_y, OMP_z]
    mbs_data : MBsysPy.MbsData
        The multibody system associated to this computation.
    tsim : float
        The current time of the simulation.
    ixF : int
        The ID identifying the computed force sensor.

    Notes
    -----
    For 1D numpy.ndarray with index starting at 1, the first index (array[0])
    must not be modified. The first index to be filled is array[1].

    For 2D numpy.ndarray with index starting at 1, the first row (mat[0, :]) and
    line (mat[:,0]) must not be modified. The subarray to be filled is mat[1:, 1:].

    Returns
    -------
    Swr : numpy.ndarray
        An array of length 10 equal to [0., Fx, Fy, Fz, Mx, My, Mz, dxF].
        F_# are the forces components expressed in inertial frame.
        M_# are the torques components expressed in inertial frame.
        dxF is an array of length 3 containing the component of the forces/torque
        application point expressed in the BODY-FIXED frame.
        This array is a specific line of MbsData.SWr.
    """

    Fx = Fy = Fz = Mx = My = Mz = 0.0
    idpt = mbs_data.xfidpt[ixF]
    dxF = mbs_data.dpt[1:, idpt]
    um = mbs_data.user_model

    # Identification des pneus par leur nom dans le .mbs
    pneu_ids = [mbs_data.extforce_id[n] for n in ['F_pneu_av_g', 'F_pneu_av_d', 'F_pneu_ar_g', 'F_pneu_ar_d']]

    if ixF in pneu_ids:
        # 1. Paramètres (Front ou Rear)
        is_front = ixF in [mbs_data.extforce_id['F_pneu_av_g'], mbs_data.extforce_id['F_pneu_av_d']]
        R = um.FrontTire['R'] if is_front else um.RearTire['R']
        K = um.FrontTire['K'] if is_front else um.RearTire['K']

        # 2. Gestion du SOL & BOSSE
        x_wheel = PxF[1]
        z_sol = 0.0
        # Exemple : Bosse de 10cm entre x=10m et x=12m
        if 10.0 <= x_wheel <= 12.0:
            z_sol = 0.10 

        # 3. Pénétration (Z_roue - Rayon vs Z_sol)
        # Attention: Robotran Z est souvent vers le bas ou le haut selon ta base
        pen = z_sol - (PxF[3] - R)

        if pen > 0:
            Fz = K * pen
            # Pour le Livrable 2, on peut simplifier Bakker si l'addon n'est pas compilé
            # Ici on reste sur la force verticale pure pour l'équilibre
        else:
            Fz = 0.0

    Swr = mbs_data.SWr[ixF]
    Swr[1:] = [Fx, Fy, Fz, Mx, My, Mz, dxF[0], dxF[1], dxF[2]]
    return Swr