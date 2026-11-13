# Package `autonomy` : conduite autonome sur le circuit (ENIMIA)

Documentation de ce qui a été fait et de ce qui manque. État au 2026-11-05, **simulation seulement** (rien testé sur la vraie voiture).

## 1. Objectif

La voiture doit faire un tour de 1000 m (`shell_eco_track.sdf`) à 20 km/h (5,5 m/s) :
- sans sortir des lignes blanches,
- sans toucher d'obstacle,
- sans s'arrêter devant un obstacle : elle ralentit, prend la voie libre, dépasse, puis réaccélère.

## 2. Ce qui a été fait

### Architecture

```
caméra → line_detector → offset/cap de voie ─┐
lidar  → lidar_obstacle_detector → obstacles ─┤
/odom (vitesse) + angle de braquage ──────────┴→ fusion_node
        (Kalman EKF → LaneChanger → Stanley)  → /cmd_vel → chaîne de conduite
```

| Fichier | Rôle |
|---|---|
| `autonomy/kalman.py` | `LaneEKF` : état [décalage latéral, cap, vitesse]. Mises à jour scalaires avec porte d'innovation. Après `max_rejects` rejets consécutifs, il se réinitialise sur la mesure. |
| `autonomy/avoidance.py` | `LaneChanger` (logique pure) : choisit la voie, calcule la référence latérale, dit s'il faut ralentir ou freiner. |
| `autonomy/lane_controller.py` | Stanley avec référence latérale et anticipation de cap. Vitesse réduite à `v_avoid` près des obstacles. Freinage si les deux voies sont bloquées. |
| `autonomy/fusion_node.py` | Nœud ROS : relie Kalman, planificateur et contrôleur. Compense le retard caméra (`camera_delay` 0,45 s). |
| `launch/lap.launch.py` | Une commande : Gazebo + perception + autonomie. `ros2 launch autonomy lap.launch.py [v_cruise:=5.5]` |
| `tools/eval_lap.py` | Notation avec la vérité terrain : progression, croisements de ligne, collisions, marges, arrêts. |
| `tools/trace_lap.py` | Trace par seconde : position vraie contre ce que la voiture « croit ». |
| `test/` | 22 tests unitaires (`cd src/autonomy && PYTHONPATH=. python3 -m pytest test`). |

### Comportement face à un obstacle

1. Le lidar détecte l'obstacle (utile jusqu'à environ 8 m).
2. La voiture descend à `v_avoid` (2,5 m/s) dès `slow_dist` (18 m).
3. Si l'obstacle bloque la trajectoire et que l'autre voie est libre, la référence latérale glisse vers l'autre voie (pente `lane_change_slope` 0,3).
4. Après le dépassement, la voiture reste dans la voie libre (`prefer_right=False`) puis réaccélère jusqu'à `v_cruise`.
5. Elle ne freine que si les deux voies sont bloquées.

### Défauts trouvés et corrigés

| Problème | Correction |
|---|---|
| Kalman rejetait toute mesure après un saut caméra (offset dérivé jusqu'à 8 m) | Réinitialisation après `max_rejects` + test |
| `line_detector` ne publiait rien (exécuteur mono-thread saturé par `/clock` à 400-900 Hz) | `MultiThreadedExecutor` dans `line_detector` et `lidar_obstacle_detector` |
| Oscillations de ±2 m (retard caméra ~0,45 s) | Compensation `camera_delay`, `k_stanley` 1,0 → 0,6 |
| Cônes vus trop tard | `single_point_range` lidar 5 → 12 m |
| Géométrie du planificateur (test au centre au lieu du pare-chocs) | Évaluation au pare-chocs avant et à l'arrière |
| Blocage à côté d'un obstacle | Freinage seulement pour ce qui est devant (`ahead_only`) |
| Retour inutile dans la voie de droite → blocage | `prefer_right=False` |
| Vitesse bloquée à 3 m/s (ancien `cmd_vel_to_drive`) | Tuer tous les processus avant de relancer (`kill_all.sh`) |

### Réglages de vitesse (20 km/h)

`v_cruise` 5,5 ; `max_speed` 5,6 dans `vehicle.yaml`, `ackermann_controller`, `cmd_vel_to_drive`, et `esp32_bldc.ino` (**source modifiée, NON flashée**). Nav2 réglé pour 5,5 m/s. `decel` 3,0.

### Résultats mesurés en simulation

| Essai | Distance | Obstacles dépassés | Collisions | Lignes touchées | Arrêts |
|---|---|---|---|---|---|
| Meilleur essai | 577 m | 42 | 0 | 0 | aucun (vitesse max 5,49 m/s, marge mini 0,72 m) |
| Dernier réglage | 280-345 m | 20-24 | 0 | 0 | un blocage par essai |

## 3. Ce qui manque

### Bloquant pour le tour complet

1. **Tour de 1000 m non terminé.** La voiture se bloque dans les groupes d'obstacles à moins de 10 m dans des voies opposées (ex. s=243/253, 338/350). Causes : le lidar voit à ~8 m seulement, la voiture ne va que vers l'avant, et les règles de décision sont rigides.
   Piste : choisir une trajectoire parmi quelques candidates (échantillonnage) avec marge adaptative, décider plus tôt, mieux gérer le cas « deux voies bloquées » (ralentir à l'extrême plutôt que s'arrêter à côté).
2. **Arrêts aux stop (s=150, 300, …, 900).** Non implémentés : aucun arrêt de plus d'1 s observé. Il n'y a pas de détecteur de panneau, seulement la ligne d'arrêt vue par la caméra.

### Physique et réglages

3. **Freinage.** Simulé : arrêt en ~5 m depuis 5,5 m/s (freins arrière seuls, 3-3,5 m/s²). Un arrêt en 3 m demande environ 5 m/s², donc des freins avant (non modélisés). À mesurer sur la vraie voiture.
4. **Dépassement du but Nav2** d'environ 3,9 m à 20 km/h (Nav2 déclare le succès à 1 m puis freine). Le ralentissement à l'approche n'a pas été efficace.
5. **Limite de braquage selon la vitesse.** À 5,5 m/s en plein braquage, l'accélération latérale atteint ~3,8 m/s².
6. **Gain Stanley** à ré-ajuster après la correction de géométrie.

### Matériel et divers

7. `ticks_per_rev: 90` est un **placeholder** : à mesurer sur la vraie voiture.
8. Le firmware (`MAX_SPEED 5.6`) n'est pas flashé. Il faut demander avant tout flash.
9. Rien n'a été testé sur le matériel réel (lidar RPLIDAR, caméra, ESP32).
10. Bug cosmétique `world_markers` : chemin `file://` doublé (RViz).
11. Les tests couvrent la logique pure, pas les nœuds ROS ni une boucle fermée automatisée.

## 4. Pièges connus

- Tuer **tous** les processus avant de relancer (`kill_all.sh`) : des nœuds périmés ont causé un plafond à 3 m/s et un TF en double.
- Ne jamais faire `pkill -f` avec un motif présent dans sa propre ligne de commande (tue l'appelant, code 144).
- Fermer la fenêtre `ign gazebo gui` arrête aussi le serveur.
- Les sources Python de `perception` sont copiées dans `build/` : refaire `colcon build --packages-select perception` après modification.
- Gazebo tourne en rendu logiciel (RTF ~0,1, caméra < 1 Hz) : attendre 6-9 s après chaque téléportation.
- Détails complets dans `PROGRESS.md` (entrées (1) à (6)) et `CLAUDE.md`.
