# ROS 2 ve Gazebo ile 4 Tekerli Araba Robotu

Bu proje, ROS 2 ve Gazebo kullanarak 4 tekerli bir araba robotu simülasyonu içerir. Robot GPS ve LIDAR sensörleri ile donatılmıştır, AWSD tuşlarıyla kontrol edilebilir. Opsiyonel olarak şerit takibi (line follower), EKF lokalizasyon (IMU+GPS füzyonu) ve LIDAR tabanlı SLAM haritalama desteklenir.

## Özellikler

- ✅ 4 tekerli araba robot modeli
- ✅ GPS sensörü
- ✅ LIDAR sensörü
- ✅ AWSD tuşlarıyla keyboard kontrolü
- ✅ Gazebo simülasyonu

**Ekstra (opsiyonel özellikler):**
- ✅ Şerit takibi (Line follower) – sahte çizgi sapması ile
- ✅ EKF lokalizasyon – IMU + GPS füzyonu (sahte sensör ile)
- ✅ SLAM haritalama – LIDAR ile canlı harita


## Kurulum

### 1. ROS 2 Kurulumu

ROS 2'yi henüz kurmadıysanız, [ROS 2 resmi dokümantasyonunu](https://docs.ros.org/en/humble/Installation.html) takip edin.

### 2. Gerekli Paketleri Yükleyin

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs \
                 ros-humble-robot-state-publisher \
                 ros-humble-xacro \
                 ros-humble-geometry-msgs \
                 ros-humble-sensor-msgs
```

Opsiyonel (EKF veya SLAM kullanacaksanız):

```bash
# EKF lokalizasyon için
sudo apt install ros-humble-robot-localization

# SLAM haritalama için
sudo apt install ros-humble-slam-toolbox
```

### 3. Workspace'i Derleyin

```bash
cd robot_ws
colcon build
source install/setup.bash
```

## Kullanım

### Simülasyonu Başlatma

**Terminal 1** – Gazebo ve robotu başlat:

```bash
cd robot_ws
source install/setup.bash
ros2 launch robot_gazebo robot_gazebo.launch.py
```

Bu komut Gazebo'yu açar, özel parkur dünyasını yükler ve robotu spawn eder.

### Robotu Kontrol Etme (Klavye)

**Terminal 2** – Aynı bilgisayarda yeni bir terminal açıp klavye ile sür:

```bash
cd robot_ws
source install/setup.bash
ros2 run robot_control keyboard_teleop
```

Terminal 2'de (pencerenin odakta olduğundan emin olarak) şu tuşları kullanın:

- **W** - İleri git
- **S** - Geri git
- **A** - Sola dön
- **D** - Sağa dön
- **Q** - Çıkış
- **Space** - Dur

### Şerit Takibi (Line Follower)

Sahte çizgi sapması ile test etmek için önce sahte sensörleri başlatın, sonra şerit takip node'unu çalıştırın:

```bash
# Terminal 2: Sahte sensörler (/line_error, /imu/data, /gps/fix)
ros2 launch robot_gazebo fake_sensors.launch.py

# Terminal 3: Şerit takip (robot_interfaces/LineError dinler)
ros2 run robot_control line_follower
```

### Opsiyonel: EKF Lokalizasyon

Sahte IMU/GPS ile füzyon için (simülasyon çalışırken):

```bash
ros2 launch robot_gazebo localization.launch.py
```

### Opsiyonel: SLAM Haritalama

LIDAR ile canlı harita için (simülasyon çalışırken):

```bash
ros2 launch robot_gazebo slam_mapping.launch.py
```

### Sensör Verilerini Görüntüleme

Yeni bir terminal açın ve aşağıdaki komutları kullanın:

**GPS verilerini görüntüle:**
```bash
ros2 topic echo /gps/fix
```

**LIDAR verilerini görüntüle:**
```bash
ros2 topic echo /scan
```

**Robot pozisyonunu görüntüle:**
```bash
ros2 topic echo /odom
```

**RViz ile görselleştirme:**
```bash
ros2 run rviz2 rviz2
```

RViz'de:
- Fixed Frame: `base_link` veya `odom` olarak ayarlayın
- Add > By topic > `/scan` (LIDAR için)
- Add > By topic > `/odom` (Odometry için)

## Proje Yapısı

```
robot_ws/
├── src/
│   ├── robot_interfaces/      # Özel mesaj (LineError - şerit takibi)
│   │   ├── msg/
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   ├── robot_description/     # Robot URDF/Xacro + Gazebo plugin
│   │   ├── urdf/robot.urdf.xacro
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   ├── robot_control/         # keyboard_teleop, line_follower
│   │   ├── src/
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   └── robot_gazebo/          # Launch, dünya, config, sahte sensörler
│       ├── launch/            # robot_gazebo, fake_sensors, localization, slam_mapping
│       ├── config/            # EKF, SLAM Toolbox
│       ├── scripts/           # fake_sensors.py
│       ├── worlds/            # robot_course.world
│       ├── rviz/              # robot_dashboard.rviz
│       ├── package.xml
│       └── CMakeLists.txt
└── README.md
```

## Robot Özellikleri

- **Boyutlar**: 0.6m x 0.4m x 0.2m (uzunluk x genişlik x yükseklik)
- **Tekerlekler**: 4 adet, 0.2m çapında
- **GPS**: Robot üzerinde, base_link'ten 0.25m yukarıda
- **LIDAR**: Robot önünde, 360° tarama, 30m menzil
- **Kontrol**: Differential drive (4 tekerlek)

## Sorun Giderme

### Gazebo açılmıyor
- Gazebo'nun düzgün kurulduğundan emin olun: `gazebo --version`
- ROS 2 environment'ını source ettiğinizden emin olun

### Robot görünmüyor
- URDF dosyasının doğru yüklendiğini kontrol edin
- Gazebo loglarını kontrol edin

### Keyboard kontrol çalışmıyor
- Terminal penceresinin focus'ta olduğundan emin olun
- Keyboard teleop node'unun çalıştığını kontrol edin: `ros2 node list`

### Sensör verileri gelmiyor
- Gazebo plugin'lerinin doğru yüklendiğini kontrol edin
- Topic'leri listeleyin: `ros2 topic list`

