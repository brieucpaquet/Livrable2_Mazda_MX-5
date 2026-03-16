#!/usr/bin/env python3
import MBsysPy
import os
import numpy as np
import matplotlib.pyplot as plt

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

# =========================================================
# EQUILIBRE STATIQUE
# =========================================================
mbs_equil.run()

# --- NOUVEAU : ON LANCE LA VOITURE ---
vitesse_kmh = 7.0
vitesse_ms = vitesse_kmh / 3.6  # 10 m/s

# 1. On pousse le châssis vers l'avant (Joint T1 du châssis, souvent q[1])
mbs_data.qd[1] = vitesse_ms 

# 2. On fait tourner les 4 roues à la bonne vitesse pour ne pas déraper
# Vitesse angulaire (omega) = Vitesse (v) / Rayon (R)
omega = vitesse_ms / 0.295 

mbs_data.qd[11] = omega  # Remplace 11, 16, 32, 36 par les vrais numéros de tes roues !
mbs_data.qd[16] = omega
mbs_data.qd[26] = omega
mbs_data.qd[30] = omega

# =========================================================
# DYNAMIQUE DIRECTE
# =========================================================
mbs_dirdyn.run()
print("\nSimulation terminée avec succès !")


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

# =========================================================
# EQUILIBRE STATIQUE
# =========================================================
mbs_equil.run()

# --- NOUVEAU : ON LANCE LA VOITURE ---
vitesse_kmh = 7
.0
vitesse_ms = vitesse_kmh / 3.6  # 10 m/s

# 1. On pousse le châssis vers l'avant (Joint T1 du châssis, souvent q[1])
mbs_data.qd[1] = vitesse_ms 

# 2. On fait tourner les 4 roues à la bonne vitesse pour ne pas déraper
# Vitesse angulaire (omega) = Vitesse (v) / Rayon (R)
omega = vitesse_ms / 0.295 

mbs_data.qd[11] = omega  # Remplace 11, 16, 32, 36 par les vrais numéros de tes roues !
mbs_data.qd[16] = omega
mbs_data.qd[26] = omega
mbs_data.qd[30] = omega

# =========================================================
# DYNAMIQUE DIRECTE
# =========================================================
mbs_dirdyn.run()
print("\nSimulation terminée avec succès !")



# =========================================================
# AFFICHAGE ET SAUVEGARDE DES GRAPHES DE SUSPENSION
# =========================================================
print("\n>> Génération des graphes de suspension...")

# 1. Chargement des données de sortie
# Le fichier dirdyn_q.res contient toutes les positions articulaires 'q' au cours du temps
results_dir = os.path.normpath(os.path.join(work_dir, "..", "resultsR"))
results_path = os.path.join(results_dir, "dirdyn_q.res")

# On charge le fichier avec numpy. loadtxt ignore la première ligne d'en-tête
# La première colonne est le temps, les suivantes sont les q1, q2, ...
results = np.loadtxt(results_path)

time = results[:, 0]  # Temps (toutes les lignes, colonne 0)

# Pour les suspensions, nous nous intéressons aux coordonnées qui font bouger les bras.
# D'après la structure double triangle standard, ce sont les pivots des bras inférieurs :
# AV_G: q8, AV_D: q13, AR_G: q24, AR_D: q28.
# ATTENTION: numpy commence à compter à 0, donc q8 correspond à l'indice 8 dans le tableau
q_av_g = results[:, 8]   # q8  (AV Gauche)
q_av_d = results[:, 13]  # q13 (AV Droit)
q_ar_g = results[:, 24]  # q24 (AR Gauche)
q_ar_d = results[:, 28]  # q28 (AR Droit)


# 2. Création de la figure avec 4 sous-graphes (2x2)
fig, axs = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
fig.suptitle('Angle des bras de suspension vs Temps - Mazda MX-5', fontsize=16)

# Sous-graphe AVANT GAUCHE (axs[0, 0])
axs[0, 0].plot(time, q_av_g, color='blue')
axs[0, 0].set_title('Avant Gauche (q8)')
axs[0, 0].grid(True)
axs[0, 0].set_ylabel('Angle (rad)')

# Sous-graphe AVANT DROIT (axs[0, 1])
axs[0, 1].plot(time, q_av_d, color='darkblue')
axs[0, 1].set_title('Avant Droit (q13)')
axs[0, 1].grid(True)

# Sous-graphe ARRIÈRE GAUCHE (axs[1, 0])
axs[1, 0].plot(time, q_ar_g, color='red')
axs[1, 0].set_title('Arrière Gauche (q24)')
axs[1, 0].grid(True)
axs[1, 0].set_ylabel('Angle (rad)')
axs[1, 0].set_xlabel('Temps (s)')

# Sous-graphe ARRIÈRE DROIT (axs[1, 1])
axs[1, 1].plot(time, q_ar_d, color='darkred')
axs[1, 1].set_title('Arrière Droit (q28)')
axs[1, 1].grid(True)
axs[1, 1].set_xlabel('Temps (s)')


# 3. Sauvegarde de la figure (SAVE FIG)
# Le fichier sera sauvegardé dans le dossier resultsR sous le nom suspension_plots.png
plot_save_path = os.path.join(results_dir, "suspension_plots.png")
plt.savefig(plot_save_path, dpi=300) # dpi=300 pour une bonne résolution

plt.show()
