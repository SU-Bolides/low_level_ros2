# Bolide STM32

Ce package ROS2 gère la communication avec le microcontrôleur STM32 et les composants électroniques associés à la voiture autonome Bolide.

## Description

Le package assure l'interface entre ROS2 et le STM32 qui contrôle :
- Les capteurs de navigation (IMU, capteurs infrarouges arrière, capteur optique de vitesse)
- Le moteur de propulsion via l'ESC (Electronic Speed Controller)
- La communication SPI bidirectionnelle

## Architecture

Le package comprend trois nœuds principaux :

### stm32_node
Nœud principal de communication avec le STM32 via SPI. Il :
- Reçoit les données des capteurs (IMU, IR, fork)
- Convertit et publie les données sur les topics appropriés
- Transmet les commandes PWM au STM32

### cmd_vel_node
Nœud de contrôle de vitesse qui :
- Reçoit les commandes de vitesse sur `/cmd_vel`
- Convertit les valeurs normalisées (-1.0 à 1.0) en signaux PWM
- Gère la sécurité avec timeout d'arrêt d'urgence
- Implémente la logique de changement de direction

### esc_setup
Outil de calibration et test de l'ESC Tamiya TBLE-04S.

## Capteurs gérés

### IMU (Inertial Measurement Unit)
- **Accélération** : axe X (m/s²)
- **Vitesse angulaire** : autour de l'axe Z (rad/s)
- **Orientation** : lacet (yaw) en radians

### Capteurs infrarouges arrière
- **Portée** : 6cm à 30cm
- **Position** : gauche et droite du véhicule
- **Utilisation** : détection d'obstacles arrière

### Capteur optique de vitesse (Fork)
- **Mesure** : vitesse linéaire de la roue (m/s)
- **Direction** : sens de rotation (avant/arrière)

## Commandes de vitesse

Le système accepte des commandes normalisées :
- `0.0` : point neutre (roue libre)
- `0.01` à `1.0` : marche avant (vitesse croissante)
- `-0.01` à `-1.0` : marche arrière (vitesse croissante)
- `2.0` : frein d'urgence (non implémenté dans cmd_vel_node)

**Attention** : Éviter les valeurs `1.0` et `-1.0` qui peuvent causer des chutes de tension du Raspberry Pi.

## Topics

### Publishers (stm32_node)
- `/raw_fork_data` (bolide_interfaces/ForkSpeed) : vitesse de la roue
- `/raw_imu_data` (sensor_msgs/Imu) : données IMU
- `/raw_rear_range_data` (bolide_interfaces/MultipleRange) : capteurs IR arrière
- `/stm32_sensors` (std_msgs/Float32MultiArray) : tableau des données brutes

### Subscribers (stm32_node)
- `/stm32_data` (std_msgs/Int16) : commandes PWM pour l'ESC

### Subscribers (cmd_vel_node)
- `/cmd_vel` (std_msgs/Float32) : commandes de vitesse normalisées

## Paramètres

### stm32_node
- `debug` : active les logs de debug (défaut : False)

### cmd_vel_node
- `debug` : active les logs de debug (défaut : False)
- `minimal_speed` : vitesse minimale PWM pour l'ESC (défaut : 8.3)

## Lancement

```bash
# Nœud principal STM32
ros2 run bolide_stm32 stm32_node

# Nœud de contrôle de vitesse
ros2 run bolide_stm32 cmd_vel_node

# Outil de calibration ESC
ros2 run bolide_stm32 esc_setup
```

## Calibration de l'ESC

L'ESC doit être calibré avant la première utilisation :

1. Lancer `esc_setup` et choisir l'option 1 (calibration)
2. Allumer l'ESC
3. Suivre les instructions pour enregistrer MAX, MIN et NEUTRE
4. Tester avec l'option 2

## Sécurité

- **Timeout de sécurité** : arrêt automatique après 0.5s sans commande
- **Changement de direction** : nécessite un arrêt préalable pour éviter les à-coups
- **Limites de tension** : éviter les valeurs extrêmes pour préserver le Raspberry Pi

## Dépannage

### Problèmes de communication SPI
- Vérifier les connexions physiques (bus SPI 0, device 1)
- Contrôler la vitesse SPI (112500 Hz)

### ESC ne répond pas
- Recalibrer l'ESC avec `esc_setup`
- Vérifier l'alimentation de l'ESC
- Tester les valeurs PWM avec l'option 3 de `esc_setup`

### Capteurs défaillants
- Vérifier les connexions sur le STM32
- Consulter les logs de debug (`debug: true`)
- Les capteurs IR peuvent nécessiter un étalonnage manuel

## Architecture matérielle

Le STM32 communique via SPI avec le Raspberry Pi et contrôle :
- IMU 9DOF (accéléromètre, gyroscope, magnétomètre)
- 2 capteurs IR Sharp GP2Y0A21YK (arrière)
- Capteur optique de vitesse (fork)
- ESC Tamiya TBLE-04S (moteur de propulsion)