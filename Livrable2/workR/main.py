#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Script to run a direct dynamic analysis on a multibody system.

Summary
-------
This template loads the data file *.mbs and execute:
 - the coordinate partitioning module
 - the equilibrium module
 - the modal module
 - the direct dynamic module (time integration of equations of motion).
"""

# %%===========================================================================
# Packages loading
# =============================================================================
try:
    import MBsysPy as Robotran
except:
    raise ImportError("MBsysPy not found/installed."
                      "See: https://www.robotran.eu/download/how-to-install/")
import os

# %%===========================================================================
# Project loading
# =============================================================================
work_dir = os.path.dirname(os.path.abspath(__file__))
mbs_file = os.path.normpath(os.path.join(work_dir, "..", "dataR", "Livrable2.mbs"))

print(f"DEBUG: Tentative d'ouverture de : {mbs_file}")

try:
    mbs_data = Robotran.MbsData(mbs_file)
    print(">> LOAD XML >> OK")
except Exception as e:
    print(f"ERREUR CRITIQUE : Impossible de charger le fichier .mbs à {mbs_file}")
    raise e

# %%===========================================================================
# Partitionning
# =============================================================================
mbs_data.process = 1
mbs_part = Robotran.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

# =============================================================================
# Equilibrium
# =============================================================================
# --- 1. Équilibre Statique (La voiture se pose sur ses suspensions) ---
mbs_data.process = 1 
mbs_equil = Robotran.MbsEquil(mbs_data)
mbs_equil.set_options(method=1, senstol=1e-2, verbose=1, resfilename="equil_static")
mbs_equil.run()

# --- 2. Équilibre Straight Line (Vitesse constante 10 m/s) ---
V_cible = 10.0 
# q1 est le joint T1 de ton châssis dans ton .mbs
mbs_data.qd[mbs_data.joint_id['q1']] = V_cible

# Vitesse de rotation des roues (V/R avec R=0.3m)
# Noms : Joint_21 (AV_G), Joint_4 (AV_D), Joint_25 (AR_G), Joint_30 (AR_D)
roues = ['Joint_21', 'Joint_4', 'Joint_25', 'Joint_30']
for wheel in roues:
    if wheel in mbs_data.joint_id:
        mbs_data.qd[mbs_data.joint_id[wheel]] = V_cible / 0.3

mbs_data.process = 2 
mbs_equil_sl = Robotran.MbsEquil(mbs_data)
mbs_equil_sl.set_options(mode=2, verbose=1, resfilename="equil_straightlane")

# Correction de la syntaxe add_exchange (Arguments positionnels obligatoires)
try:
    # (Nom_UserModel, Nom_Variable, Joint_ID)
    mbs_equil_sl.add_exchange('EquilQ', 'Qpropulsion', mbs_data.joint_id['q1'])
    mbs_equil_sl.run()
except Exception as e:
    print(f"\n[NOTE] L'équilibre Straight Line a rencontré un souci : {e}")
    # On force le flag_stop à 0 pour ne pas bloquer la suite
    mbs_data.flag_stop = 0

# =============================================================================
# Modal Analysis
# =============================================================================
mbs_data.flag_stop = 0 # Sécurité pour permettre le lancement
mbs_data.process = 4
mbs_modal = Robotran.MbsModal(mbs_data)
mbs_modal.set_options(save_result=1, save_anim=1, mode_ampl=0.2)
try:
    mbs_modal.run()
except Exception as e:
    print(f"Note: Modal interrompu ou impossible: {e}")

# =============================================================================
# Direct Dynamics (Simulation temporelle)
# =============================================================================
mbs_data.flag_stop = 0 # On s'assure que le simulateur n'est pas bloqué
mbs_data.process = 3
mbs_dirdyn = Robotran.MbsDirdyn(mbs_data)
# tf=5.0 secondes de simulation
mbs_dirdyn.set_options(dt0=1e-3, tf=5.0, save2file=1)
mbs_dirdyn.run()

print("\n>> SIMULATION TERMINEE. Vous pouvez charger 'dirdyn_q.res' dans MBsysPad.")