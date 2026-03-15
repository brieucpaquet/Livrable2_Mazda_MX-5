#!/usr/bin/env python3
import MBsysPy
import os

print("Starting Mazda MX-5 MBS project!")
work_dir = os.path.dirname(os.path.abspath(__file__))
mbs_file = os.path.normpath(os.path.join(work_dir, "..", "dataR", "Livrable2.mbs"))

mbs_data = MBsysPy.MbsData(mbs_file)

# =========================================================
# INITIALISATION DU USER MODEL
# =========================================================
um = {}
um['FrontTire']       = {'R': 0.295, 'K': 200000.0}
um['RearTire']        = {'R': 0.295, 'K': 200000.0}
um['FrontSuspension'] = {'K': 25000.0, 'C': 2000.0, 'C_bar': 1200.0}
um['RearSuspension']  = {'K': 22000.0, 'C': 2000.0, 'C_bar':  800.0}
mbs_data.user_model = um

# =========================================================
# CONDITIONS INITIALES
# =========================================================
mbs_data.q[3] = 0.118  # roues légèrement en contact avec le sol

# =========================================================
# PARTITIONNEMENT  (doit venir avant MbsEquil)
# =========================================================
print("\n>> PARTITIONNEMENT...")
mbs_part = MBsysPy.MbsPart(mbs_data)
mbs_part.set_options(rowperm=1, verbose=1)
mbs_part.run()

# =========================================================
# EQUILIBRE STATIQUE  (après PART)
# =========================================================
print("\n>> RECHERCHE D'EQUILIBRE...")
mbs_equil = MBsysPy.MbsEquil(mbs_data)
mbs_equil.set_options(senstol=1e-6)
mbs_equil.run()

# =========================================================
# DYNAMIQUE DIRECTE
# =========================================================
print("\n>> LANCEMENT DE LA DYNAMIQUE (Dirdyn)...")
mbs_dirdyn = MBsysPy.MbsDirdyn(mbs_data)
mbs_dirdyn.set_options(dt0=1e-3, tf=2.0, save2file=1)
mbs_dirdyn.run()

print("\nSimulation terminée avec succès !")

