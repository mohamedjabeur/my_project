# ENIMIA Black Edition : documentation du projet (simulation)

Voiture RC autonome d'extérieur de l'équipe ENIM Team Tunisia, pour un circuit de type Shell Eco-marathon.
Ce document décrit l'état du projet au **2026-11-08**. Tout ce qui est décrit ici a été testé **en simulation (Gazebo)** uniquement.
Le comportement sur la vraie voiture n'a pas été testé (voir §14).

Autres fichiers de référence : `CLAUDE.md` (mémo pour reprendre le travail), `PROGRESS.md` (journal daté de chaque étape),
`src/autonomy/README.md` (historique détaillé de la conduite autonome, en français).

---

## 1. Objectif

Un tour de circuit fermé d'environ 1 km (route de 7 m, 2 voies de 3,5 m), en conduite autonome :
- accélérer jusqu'à **20 km/h** (5,56 m/s) sur route libre ;
- **ralentir à 10 km/h** (2,78 m/s) près des obstacles et dans les virages, puis réaccélérer ;
- contourner les obstacles en changeant de voie, sans sortir des lignes blanches ni toucher un obstacle ;
- **ne s'arrêter que lorsqu'elle atteint le point cible** cliqué dans RViz (outil « 2D Goal Pose »).

Le circuit contient 68 obstacles (cônes, caisses, barils, murs, personnes), en alternance sur les deux voies, environ tous les 10 à 12 m.

## 2. La voiture

| Élément | Valeur |
|---|---|
| Type | Ackermann, propulsion arrière (moteur BLDC), direction avant, **marche avant seulement** |
| Empattement | 1,704 m |
| Braquage maximal | ±0,21 rad, rayon de braquage minimal environ 8 m |
| Repère de base `base_footprint` | milieu de l'empattement, au sol (l'essieu arrière est 0,852 m derrière) |
| Vitesse | 10 à 20 km/h ; en dessous de 10 km/h seulement pour s'arrêter |
| Freinage (mesuré en sim) | roues arrière seulement : 3 à 3,5 m/s², arrêt depuis 5,5 m/s en environ 5 m |

### Capteurs : réel et simulé

La simulation imite les capteurs réels, de façon que tout ce qui est au-dessus du pilote de capteur soit identique.

| Capteur | Voiture réelle | Simulation (`enimia.urdf`) |
|---|---|---|
| Lidar | Livox Mid-360, nuage 3D, 40 m | `gpu_lidar` 720 × 17 couches (de -3° à +5° dans le repère du capteur), 10 Hz, 0,15 à 40 m |
| Caméra | OAK-D Lite, 960×540, 15 i/s, champ horizontal 69° | caméra 960×540, 15 i/s, champ 69° (1,2043 rad), fx = fy = 698 px |
| IMU | BNO055 (I2C), 50 Hz | IMU 50 Hz, bruit gaussien |
| Contrôleur | ESP32 (Arduino, BLDC + servo, série 115200) | `virtual_esp32` (chaîne `vehicle/sim.launch.py`) ou chemin direct (`localization`) |

Positions : lidar à 0,78 m de haut (incliné de 4° vers le bas), caméra à 0,732 m (incliné de 10° vers le bas, 0,13 m devant le support).
L'URDF `src/vehicle_description/urdf/enimia.urdf` est l'**unique source de la géométrie** (repères, capteurs, plugins Gazebo).

**Chaîne lidar** (identique en simulation et sur la voiture, sauf le premier maillon) :
`lidar Gazebo / Livox → /livox/lidar (PointCloud2) → conversion en LaserScan → /scan` (dans le repère horizontal `lidar_level_link`,
points entre 0,18 et 1,08 m au-dessus de la route). Sur la voiture, la conversion est faite par `pointcloud_to_laserscan` ;
dans la simulation, par `vehicle_drivers/scripts/cloud_to_scan.py` (même fichier de réglages `pointcloud_to_scan.yaml`),
car `pointcloud_to_laserscan` n'est pas installé ici.

## 3. Prérequis, compilation, lancement

- Ubuntu 22.04, **ROS 2 Humble**, Ignition Gazebo (`ign gazebo`, `ros_gz_bridge`, `ros_gz_sim`), Python 3.10, numpy, OpenCV.
- Pas de dépôt git dans ce dossier. Une copie de sauvegarde d'avant le nettoyage est dans `~/ros2_backup_2026-11-08.tar.gz`.

```bash
cd ~/ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install          # ou --packages-select <paquet>
source install/setup.bash
```

Ne jamais modifier `build/`, `install/`, `log/` (générés). Après une modification de code dans `perception`, relancer
`colcon build --packages-select perception` : ses sources Python sont copiées dans `build/` (les fichiers yaml et launch sont liés).

### Lancer la simulation

```bash
ros2 launch autonomy drive.launch.py
```

Cette commande démarre, en une fois : Gazebo (circuit + voiture), les ponts ROS, la perception (lidar + caméra), la conduite autonome et RViz.
La voiture **reste immobile jusqu'au premier clic** « 2D Goal Pose » dans RViz (repère fixe `odom`), puis roule jusqu'au point cliqué et s'y arrête.
Un indicateur de vitesse s'affiche au-dessus de la voiture dans RViz (vert en croisière, orange en dessous de 11 km/h, rouge à l'arrêt).

Autres lancements :

| Commande | Rôle |
|---|---|
| `ros2 launch autonomy lap.launch.py [v_cruise:=5.56] [require_goal:=false] [stop_at_lines:=false] [open_rviz:=true]` | même chose, avec arguments ; par défaut la voiture roule sans attendre de but (tour continu) |
| `ros2 launch navigation bringup.launch.py` | simulation avec Nav2 (autre pilote ; **non retesté** depuis le changement des capteurs) |
| `ros2 run navigation send_goal X Y [YAW]` | envoie un but à Nav2 |
| `ros2 launch vehicle sim.launch.py` | chaîne de conduite « comme la vraie voiture » (ESP32 virtuel + odométrie des roues) ; ne pas lancer avec `localization/teleop.launch.py` |

Argument à ne pas nommer `rviz` dans un launch qui inclut `description.launch.py` (il l'écraserait) : utiliser `open_rviz`.

**Une seule simulation à la fois.** `lap.launch.py` refuse de démarrer si un `ign gazebo server` tourne déjà (variable
`ENIMIA_ALLOW_SECOND_SIM=1` pour forcer). Pour arrêter : Ctrl+C dans le terminal du lancement.

### Envoyer un but sans RViz

```bash
ros2 topic pub --once /nav_goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 158.9, y: -9.9, z: 0.0}, orientation: {w: 1.0}}}"
```

La voiture démarre en (x = -0,55 ; y = -1,66), voie de droite, cap -0,321 rad.

## 4. Arborescence des paquets (`src/`)

| Paquet | Type | Rôle |
|---|---|---|
| `vehicle_description` | CMake | `enimia.urdf` (géométrie, capteurs), maillages, config RViz, `description.launch.py` (robot_state_publisher) |
| `vehicle_drivers` | CMake | lancement des capteurs : `lidar.launch.py`, `camera.launch.py`, `imu.launch.py` (`use_sim:=true` = pont Gazebo, sinon vrai pilote) ; `cloud_to_scan.py` |
| `vehicle` | Python | chaîne de conduite : `ackermann_controller`, `virtual_esp32`, `esp32_bridge`, `wheel_odometry`, `ackermann.py` ; config `vehicle.yaml` |
| `localization` | Python | `sim.launch.py` (Gazebo + apparition de la voiture), `teleop.launch.py` (chemin direct), `cmd_vel_to_drive`, `odom_tf` |
| `perception` | Python | `lidar_obstacle_detector`, `line_detector`, `camera_fov_marker` + modules purs (`clustering`, `lane_pixels`, `lane_geometry`, `ground_projection`) |
| `autonomy` | Python | `fusion_node` (Kalman + conduite), modules purs (`kalman`, `lane_controller`, `avoidance`, `goal_gate`), `speed_display`, outils `eval_lap` / `trace_lap` |
| `navigation` | Python | `world_markers` (obstacles dans RViz, utilisé par `drive.launch.py`) ; Nav2 : `goal_relay`, `send_goal`, `nav2_params.yaml`, arbres de comportement |
| `world` | (pas un paquet ROS) | circuit `track/shell_eco_track.sdf` généré par `generate_track.py`, `track_info.json` (ligne centrale, obstacles), `gui.config` |

`simulation/` n'existe plus ; `src/world/.claude/` est un dossier de réglages local sans lien avec la simulation.

## 5. Architecture et flux de données

```
 Gazebo ──► lidar (PointCloud2) ──► cloud_to_scan ──► /scan ──► lidar_obstacle_detector ──► /perception/lidar/obstacles ─┐
    │                                                                                                                     │
    ├────► caméra /camera/image_raw ──► line_detector ──► offset, cap, lane_valid, distance ligne d'arrêt ───────────────┤
    │                                                                                                                     ▼
    ├────► /enimia/odom (vérité terrain) ──► odom_tf ──► /odom + TF odom→base_footprint ────────────────────────────► fusion_node
    │                                                                                                                     │
    │                                                       RViz « 2D Goal Pose » ──► /nav_goal ───────────────────────►  │
    │                                                                                                                     ▼
    └◄── /enimia/rear_wheel_cmd, steer_*_cmd ◄── cmd_vel_to_drive ◄───────────────────────────────────────────────── /cmd_vel
```

Sur la voiture, la fin de la chaîne est : `/cmd_vel → ackermann_controller → /drive_cmd → esp32_bridge → ESP32 (consigne « C vitesse braquage »)`,
et l'odométrie revient par `/wheel_feedback → wheel_odometry → /odom`.

**Deux chemins de conduite en simulation, à ne jamais lancer ensemble :**
1. chemin direct (`localization/teleop.launch.py`, utilisé par `drive.launch.py` et `navigation/bringup`) : `cmd_vel_to_drive` + odométrie vérité terrain ;
2. chemin `vehicle/sim.launch.py` : ESP32 virtuel + odométrie à partir des codeurs (reproduit la vraie voiture).

### Repères (TF)

`odom → base_footprint → base_link → …` : `base_footprint` est au sol au milieu de l'empattement ; ensuite `body_link`, les roues et les
pivots de direction, `sensor_mount_link → lidar_link`, `lidar_level_link` (repère horizontal du lidar), `camera_link → camera_optical_link`,
`imu_link`, `gps_link`. `odom → base_footprint` est publié par `odom_tf` ; le reste par `robot_state_publisher` (+ `joint_state_publisher`).
Il n'y a plus de repère `map` (pas de cartographie, voir §12).

### Topics principaux

| Topic | Type | Publié par | Lu par |
|---|---|---|---|
| `/clock` | Clock | pont Gazebo (`drive_bridge`) | tous les nœuds (`use_sim_time`) |
| `/scan` | LaserScan | `cloud_to_scan` | `lidar_obstacle_detector` |
| `/camera/image_raw`, `/camera/camera_info` | Image, CameraInfo | pont Gazebo | `line_detector` |
| `/perception/lidar/obstacles` | Detection3DArray | `lidar_obstacle_detector` | `fusion_node` |
| `/perception/lidar/ego_clear_distance`, `adjacent_clear_distance` | Float32 | `lidar_obstacle_detector` | (informations) |
| `/perception/camera/ego_lane_offset`, `heading_error`, `lane_valid`, `stop_line_distance` | Float32 / Bool | `line_detector` | `fusion_node` |
| `/odom` | Odometry | `odom_tf` (ou `wheel_odometry`) | `fusion_node`, `goal_gate` |
| `/nav_goal` | PoseStamped | RViz (« 2D Goal Pose ») | `fusion_node` |
| `/nav_goal_sent` | PoseStamped | `fusion_node` (but accepté) | RViz |
| `/cmd_vel` | Twist | `fusion_node` | `cmd_vel_to_drive` / `ackermann_controller` |
| `/autonomy/state` | Float32MultiArray | `fusion_node` | `trace_lap.py` (décalage, cap, vitesse, braquage, voie visée, référence) |
| `/speed_marker`, `/world_markers`, `/camera/fov_marker`, `/perception/lidar/markers`, `/perception/camera/lane_lines` | Marker(Array) | divers | RViz |

## 6. Perception

### 6.1 Lidar : `lidar_obstacle_detector`
Entrée `/scan`. Les points sont transformés dans `base_footprint`, ceux du sol (z < 0,12 m) et de la voiture elle-même sont ignorés, puis regroupés
en amas (`clustering.py`, distance adaptée à la distance et à la résolution angulaire). Sortie : une boîte par amas dans `/perception/lidar/obstacles`
(position de la face la plus proche, centre latéral, demi-largeur) et les distances libres dans la voie de la voiture et dans la voie voisine.
Réglages : `config/lidar_obstacle_detector.yaml` ; `lidar_obstacle_detector_mid360.yaml` ajoute `max_beam_gap: 3`, `single_point_range: 22`, `x_max: 25`.
Avec le lidar Mid-360 simulé, un cône est détecté vers **11 à 12 m** (avant : 7 à 8 m avec l'ancien lidar plan).

### 6.2 Caméra : `line_detector` (vision classique, sans IA)
Seuil de blanc (canal max ≥ 150 et faible saturation) → `lane_pixels.py` (blobs, rejet des objets verticaux par point de fuite, rejet des formes épaisses) →
projection au sol (`ground_projection.py`, modèle sténopé avec hauteur et inclinaison de la caméra) → ajustement de lignes (`lane_geometry.py`) →
estimation de la voie. Sorties : décalage latéral, erreur de cap, validité, distance de la ligne d'arrêt.
La piste n'a **pas de ligne centrale** : l'estimation s'appuie sur la ligne de bord la plus proche et une largeur de voie nominale de 3,5 m.
Réglages : `config/line_detector_oak.yaml` (utilisé en simulation et pour la vraie OAK-D) ; `line_detector.yaml` = ancienne caméra 800×450, 100°.

Dans les deux nœuds, les abonnements aux capteurs ont leur propre groupe de rappels et l'exécuteur est multi-thread :
avec `use_sim_time`, `/clock` arrive à 400-900 Hz et saturerait sinon l'exécuteur.

## 7. Conduite autonome : `fusion_node` (`src/autonomy`)

Boucle à 20 Hz. À chaque pas :
1. **Filtre de Kalman étendu** (`kalman.py`, état = décalage latéral, cap relatif à la voie, vitesse) : la mesure caméra est avancée de `camera_delay`
   (0,45 s) pour compenser son retard ; la vitesse vient de `/odom`. Le filtre couvre les trous de détection de la caméra.
2. **Planificateur de voie** (`avoidance.py`, classe `LaneChanger`) : décide dans quelle voie rouler d'après les amas lidar, produit une référence latérale
   limitée en pente (`lane_change_slope` 0,3 : un changement complet de 3,4 m demande environ 11 m) et indique s'il faut ralentir.
   Un obstacle bloque le trajet s'il est plus proche du chemin que la demi-largeur de la voiture + marge + sa demi-largeur. Si la voie visée est bloquée
   et l'autre libre sur `look_ahead` m, la voiture change de voie ; si les deux sont bloquées, elle freine devant l'obstacle (`obstacle_margin` 3 m).
3. **Contrôleur de voie** (`lane_controller.py`) : direction de **Stanley** (`k_stanley`, `k_soft`) sur la référence latérale ; vitesse = `v_cruise`
   (5,56 m/s) en ligne droite, réduite vers `v_curve` (2,78 m/s) selon le braquage, `v_avoid` (2,78 m/s) près d'obstacles ou pendant un changement de voie,
   `v_curve` si la caméra perd la voie ; freinage en `sqrt(2·decel·distance)` devant un obstacle ou une ligne d'arrêt. L'accélération est limitée
   (`accel` 1,5 m/s²), le ralentissement est immédiat. Sans lidar pendant plus d'une seconde, la voiture avance au pas (`v_curve`).
   Les arrêts aux lignes d'arrêt (`stop_hold`, `stop_trigger`) sont désactivés par `stop_at_lines:=false` (réglage de `drive.launch.py`).
4. **Porte du but** (`goal_gate.py`, classe `GoalGate`) : limite uniquement la **vitesse** pour s'arrêter au point cliqué ; la voiture suit toujours la voie.
   - Un but à moins de `goal_near_dist` (40 m) derrière la voiture est refusé (marche arrière impossible). Un but plus loin est toujours accepté : le circuit
     est une boucle, la voiture suit la voie (et la boucle) jusqu'à lui.
   - À plus de 40 m, aucun freinage ; en dessous, freinage en ligne droite (`goal_latency` 0,5 s compensée).
   - But atteint (arrêt maintenu) : à moins de 3 m et lent, ou lorsque la voiture dépasse le point au plus près (cas d'un but hors route).
   - Un nouveau clic libère l'arrêt.
5. Publication de `/cmd_vel` (vitesse, vitesse angulaire = v·tan(braquage)/empattement) et de `/autonomy/state`.

Sécurités : pas de `/odom` (capteur de vitesse) pendant plus de `sensor_timeout` → commande nulle ; pas de caméra → `lane_valid` faux.
Il n'y a **pas de plancher de vitesse dur** dans le contrôleur ni le firmware (il empêcherait les arrêts) : la plage 10-20 km/h résulte des limites de conduite.

## 8. Chaîne de conduite : paquet `vehicle`

- `ackermann.py` : géométrie Ackermann (braquage des roues avant), protocole série (voir ci-dessous), limitation de pente.
- `ackermann_controller` : `/cmd_vel → /drive_cmd`, limites d'accélération (1,5 m/s²), de décélération (5 m/s²), de vitesse de braquage (1 rad/s).
- `virtual_esp32` : modèle du moteur BLDC (réponse du premier ordre, τ = 0,25 s) et du servo ; commande les roues dans Gazebo, simule un codeur.
- `wheel_odometry` : `/wheel_feedback → /odom` + TF.
- `esp32_bridge` : liaison série vers la vraie carte (non utilisé en simulation).
- Protocole série (115200) : PC → ESP32 `C <vitesse> <braquage>` à 50 Hz, `E` = arrêt d'urgence ; ESP32 → PC `S <ms> <ticks> <braquage>` ;
  surveillance (watchdog) de 500 ms.
- `ticks_per_rev: 90` dans `vehicle.yaml` est une **valeur provisoire** à mesurer sur la vraie voiture.
- La voiture réelle doit utiliser `cmd_timeout: 0.5` ; mettre 0 pour une télécommande clavier.

## 9. Paramètres importants

`src/autonomy/config/autonomy.yaml` (nœud `fusion_node`) :

| Paramètre | Valeur | Sens |
|---|---|---|
| `v_cruise` | 5,56 | 20 km/h sur route libre |
| `v_curve`, `v_avoid` | 2,78 | 10 km/h (virages, obstacles, voie perdue) |
| `accel` / `decel` | 1,5 / 3,0 m/s² | accélération limitée ; freinage de calcul (mesuré 3 à 3,5 en sim) |
| `k_stanley`, `k_soft` | 0,6 ; 1,0 | gain de la direction (1,0 oscillait de ±2 m avec le retard caméra) |
| `lane_shift` | 3,4 m | distance entre les centres des voies |
| `lane_change_slope` | 0,3 | m latéraux par m avancé |
| `look_ahead` / `slow_dist` | 14 / 18 m | distance de décision de changement de voie / de ralentissement |
| `obstacle_margin` | 3 m | distance d'arrêt devant un obstacle infranchissable |
| `camera_delay` | 0,45 s | retard image → détecteur |
| `lidar_timeout`, `sensor_timeout` | 1,0 ; 0,35 s | délais de sécurité |
| `goal_near_dist` | 40 m | zone où le but est jugé en ligne droite |
| `require_goal`, `stop_at_lines` | selon le lancement | attendre un but ; s'arrêter aux lignes d'arrêt |

`src/vehicle/config/vehicle.yaml` : `max_steer` 0,21 rad, `max_speed` 5,6 m/s, `max_accel` 1,5, `max_decel` 5,0, `max_steer_rate` 1,0, rayon de roue arrière 0,18 m.

## 10. Outils de test et de mesure (par commande)

Les tests unitaires (dossiers `test/`) ont été **supprimés** lors du nettoyage du 2026-11-08 (ils sont dans la sauvegarde). Le projet se teste par commandes :

| Outil | Usage |
|---|---|
| `python3 src/autonomy/tools/eval_lap.py [secondes]` | score avec la vérité terrain : progression, voie, franchissements de ligne blanche, collisions, distances minimales aux obstacles, arrêts |
| `python3 src/autonomy/tools/trace_lap.py [secondes]` | une ligne par seconde : position sur la piste, vitesse, braquage, état du filtre, caméra, obstacles lidar |
| `python3 src/perception/tools/eval_lane_offset.py` | contrôle du décalage latéral de la caméra contre la vérité terrain |

Procédure de test complète : lancer `ros2 launch autonomy drive.launch.py`, démarrer `eval_lap.py` et `trace_lap.py`, publier un but avec `ros2 topic pub /nav_goal` (§3),
attendre « goal reached » dans le journal. Contrôles de santé après lancement :

```bash
ros2 topic info /clock -v            # Publisher count: 1
ros2 node list | sort | uniq -d      # doit rester vide (pas de doublon)
ros2 param get /fusion_node use_sim_time    # True
```

## 11. Résultats mesurés (simulation)

Test de référence : but à 160 m (158,9 ; -9,9) sur le circuit complet d'obstacles, après le nettoyage et le remplacement des capteurs :

- vitesse maximale 5,52 m/s (≈ 20 km/h) sur route libre, 2,78 m/s près des obstacles ;
- 0 collision, 0 franchissement de ligne blanche, 10 obstacles passés (distance minimale 1,12 m) ;
- arrêts uniquement au départ et au but ; voie de droite 93 à 98 % du temps ;
- buts à 4,8 m, 59 m et 222 m en ligne droite (ce dernier à l'autre bout de la boucle, 540 m de piste) : atteints à 3 m près ; un but du même genre était refusé ou « atteint » 170 m trop tôt avant la correction de `goal_gate.py` ; un but à 227 m (-76,9 ; 138,9) est accepté (non conduit jusqu'au bout).

**Cas défaillant relevé :** un test (après le nettoyage de la cartographie) s'est arrêté à s = 82 m, à 0,17 m d'un cône dans la voie de droite, et n'a pas atteint le but (§14).

## 12. Ce qui a été retiré du projet

Pour ne garder que la simulation (sauvegarde : `~/ros2_backup_2026-11-08.tar.gz`, 4,4 Mo) :
- les dossiers de tests unitaires, les caches et `build/`, `install/`, `log/` (régénérés par `colcon build`) ;
- le firmware Arduino de l'ESP32, `hardware.launch.py`, `real.launch.py`, `fake_esp32` ;
- les fichiers Gazebo classique obsolètes (`terrain*`), les maillages STL en double ;
- **la cartographie** : `slam_toolbox`, AMCL, cartes sauvegardées, `mapping.launch.py`. La conduite ne lit jamais de carte (caméra + lidar + odométrie,
  buts dans le repère `odom`) ; Nav2 utilise des cartes de coûts glissantes sans carte statique.

Les pilotes matériels (`vehicle_drivers`) et `esp32_bridge` sont conservés : leurs lancements contiennent aussi les ponts de la simulation.

## 13. Dépannage

| Symptôme | Cause et remède |
|---|---|
| Avalanche de « Detected jump back in time » (RViz, tf2) | **deux simulations en même temps** : deux Gazebo, deux `/clock`. Faire Ctrl+C sur l'une. Vérifier `ros2 topic info /clock -v` (1 éditeur) et `ros2 node list \| sort \| uniq -d` (vide). `lap.launch.py` refuse maintenant de démarrer une seconde simulation |
| Même message avec une seule simulation | un nœud sans `use_sim_time` (y compris `robot_state_publisher` : `description.launch.py use_sim_time:=true` en simulation) |
| Voiture aveugle, `line_detector` à 100 % de CPU sans rien publier | cause trouvée : `publish_debug` construisait l'image de debug avec `bytes` (validation octet par octet, plusieurs secondes par image). Corrigé ; vérifier `ros2 topic hz /perception/camera/lane_valid` |
| `line_detector` plante au démarrage avec `car_mask: []` | une liste vide dans un yaml arrive en `None` ; corrigé |
| La voiture ne bouge pas après un clic | but refusé (moins de 40 m derrière la voiture, ou moins de 1 m) : lire le journal de `fusion_node` ; repère fixe de RViz différent de `odom` |
| Relancer sans nœuds orphelins | Ctrl+C du lancement ; vérifier `ps aux \| grep -E "ign\|ros2 launch\|rviz2"`. Dans un script, ne jamais faire `pkill -f` avec un motif présent dans la ligne de commande de l'appelant (le script se tuerait lui-même, code 144) |
| Gazebo lent | rendu logiciel, temps réel ≈ 0,1× : attendre quelques secondes après un lancement avant de lire la perception |
| Fermer la fenêtre Gazebo | arrête aussi le serveur Gazebo ; les nœuds ROS continuent : faire Ctrl+C du lancement |

## 14. Limites et travail restant

1. **Blocage contre un obstacle.** La voiture roule en marche avant seulement avec un rayon de braquage d'au moins 8 m. Si un obstacle est vu trop tard
   (détection intermittente, suivi latéral en retard), elle peut freiner jusqu'à l'arrêt à quelques dizaines de centimètres et ne peut plus se dégager
   (à l'arrêt, la référence latérale n'avance plus). Le planificateur ne connaît que deux positions (centre de chaque voie) alors qu'un cône n'exige
   qu'un décalage d'environ 1 m. Pistes : décalages partiels, décision plus précoce, ne jamais freiner jusqu'à l'arrêt à côté d'un obstacle si un contournement au pas est possible.
2. **Rien n'est testé sur la vraie voiture** (lidar Mid-360, OAK-D Lite, BNO055, ESP32). Les valeurs `ticks_per_rev`, la calibration de la caméra
   et la hauteur/inclinaison réelles des capteurs sont à mesurer. Le lidar réel est monté de niveau (voir `vehicle_drivers/README.md`), le modèle simulé est incliné de 4°.
3. **Nav2** (`navigation/bringup.launch.py`) n'a pas été retesté avec le lidar et la caméra simulés actuels.
4. L'IMU est simulée mais **non utilisée** par la conduite (la vitesse vient de `/odom`).
5. Il n'y a pas de détecteur de panneaux « stop » ; les lignes d'arrêt sont détectées par la caméra mais ignorées dans `drive.launch.py`.
6. La caméra, plus étroite que l'ancienne (69° au lieu de 100°), perd la voie plus souvent pendant les changements de voie ; le filtre de Kalman comble ces trous.
7. Le freinage n'est qu'aux roues arrière (pas de frein avant modélisé) : un arrêt en 3 m n'est pas possible.

## 15. Conventions de code

- Style : petits modules, docstring en tête de chaque fichier, commentaires courts expliquant le *pourquoi*, idiome
  `p = lambda n, d: self.declare_parameter(n, d).value`, logique pure séparée des nœuds ROS.
- Ne pas modifier `build/`, `install/`, `log/`.
- Demander avant tout ce qui touche au matériel réel, flashe un firmware ou est difficile à annuler.
- Après chaque étape importante : une entrée datée dans `PROGRESS.md` ; en fin de session : mettre à jour `CLAUDE.md` et `PROGRESS.md`.
