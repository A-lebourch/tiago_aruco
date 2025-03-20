# ArUco Move - Détection et Navigation vers un Marqueur ArUco

## 📌 Description
Ce package ROS2 permet à un robot mobile d'identifier un marqueur ArUco à l'aide de sa caméra, d'estimer sa position et de naviguer automatiquement vers celui-ci. Il utilise **OpenCV**, **Nav2** pour la détection, le traitement d'image et la navigation.

## 🛠️ Technologies Utilisées
- **ROS2 (Foxy/Humble)** : Framework de robotique
- **Gazebo** : Simulation du robot
- **Nav2** : Navigation autonome
- **OpenCV** : Détection et traitement d’image
- **ArUco** : Bibliothèque de reconnaissance de marqueurs

## 🚀 Installation
Assurez-vous d'avoir un environnement ROS2 configuré et que votre workspace est bien compilé.

```bash
cd ~/ros2_ws/src
git clone <URL_DU_REPO>
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## ▶️ Lancement du Projet
### 1️⃣ Lancer la simulation Gazebo avec Tiago
```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=house_pick_and_place
```
### 2️⃣ Démarrer la navigation avec Nav2
```bash
ros2 launch tiago_2dnav tiago_nav_bringup.launch.py is_public_sim:=True world_name:=our_map
```
### 3️⃣ Lancer le nœud de détection ArUco et de navigation
```bash
ros2 run aruco_move aruco_node
```
## 🎯 Fonctionnalités
- ✔ Détection en temps réel des marqueurs ArUco
- ✔ Estimation de la position et orientation du marqueur
- ✔ Commande automatique du robot vers la cible
- ✔ Rotation et recherche du marqueur si non détecté