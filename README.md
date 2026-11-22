# op3-krsbi-sim

Simulasi Webots untuk robot humanoid **ROBOTIS OP3** bermain sepak bola, dibuat untuk
menguji behavior [GoNasional](https://github.com/afifidf/gonasional-sim) tanpa robot fisik.
Package ROS 2: `aroc_sim_bridge` (ROS 2 Jazzy, Webots R2025a).

Isinya:

- **Lapangan 9 × 6 m** dengan dua gawang dan bola RoboCup (proto resmi Webots).
- **Framework ROBOTIS asli** (`op3_manager` mode simulasi). Topic `/robotis/...` yang dipakai
  (walking, action, head control) sama persis dengan di robot.
- **`imu_yaw_bridge`**: mengisi `/robotis/open_cr/imu` dan `/robotis/open_cr/yaw` (derajat)
  dari IMU Webots. Di simulasi, `open_cr_module` bawaan hanya mengirim nol dan tidak punya
  topic yaw.
- **`ball_tracker`** (supervisor Webots): posisi bola ground-truth dalam format yang dibaca
  GoNasional, jadi YOLO dan trigonocak tidak perlu dijalankan:
  - `/obj_detect` (`String "cx,cy"`, piksel di frame 450×337, ikut pan/tilt kepala)
  - `/vision/hasil_servo` (`Float32`, jarak horizontal robot–bola dalam cm)
  - `/vision/bola_posisi` (`String`: `kiri` / `tengah` / `kanan`)
  - `/aroc_sim/ball_position` (`PointStamped`, posisi bola di lapangan)
- **`place_ball`**: menaruh bola untuk skenario tes.

## Instalasi

```bash
sudo apt install ros-jazzy-webots-ros2-driver ros-jazzy-dynamixel-sdk
mkdir -p ~/aroc_sim_ws/src && cd ~/aroc_sim_ws/src
for r in ROBOTIS-OP3 ROBOTIS-OP3-msgs ROBOTIS-OP3-Simulations ROBOTIS-Framework ROBOTIS-Framework-msgs ROBOTIS-Math; do
  git clone --depth 1 -b jazzy-devel https://github.com/ROBOTIS-GIT/$r.git
done
git clone https://github.com/afifidf/op3-krsbi-sim.git aroc_sim_bridge
cd ~/aroc_sim_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
export WEBOTS_HOME=/usr/local/webots
```

Webots R2025a dipasang terpisah. Proto lapangan dan bola diunduh otomatis saat world dibuka
pertama kali, jadi butuh internet.

## Menjalankan

```bash
ros2 launch aroc_sim_bridge bringup.launch.py              # lapangan + bola
ros2 launch aroc_sim_bridge bringup.launch.py gui:=false   # tanpa window 3D
ros2 launch aroc_sim_bridge bringup.launch.py world:=empty # world kosong bawaan ROBOTIS
```

Robot otomatis berdiri ke init pose. Setelah itu, di terminal lain:

```bash
ros2 run GoNasional state
```

Tombol OpenCR disimulasikan lewat topic:

```bash
ros2 topic pub --once -w 2 /robotis/open_cr/button std_msgs/msg/String "data: user"
```

Urutannya `user` (berdiri, SCAN), lalu `mode` (simpan yaw, scan kepala, reset index bola),
lalu `start` (jalan follow-head). `start` bersifat toggle. `-w 2` menunggu `op3_manager`
dan GoNasional sama-sama terhubung, supaya pesan tidak hilang.

## Striker baru dan viewer lapangan

Stack baru di `GoNasional` (`aroc_soccer.launch.py`: motion manager, lokalisasi MCL,
behavior tree striker) mencari bola, mendekat, menyejajarkan badan ke arah gawang, lalu
menendang. Tidak perlu tombol virtual.

```bash
ros2 launch aroc_sim_bridge bringup.launch.py gui:=false
ros2 launch GoNasional aroc_soccer.launch.py sim:=true referee:=false
ros2 run aroc_sim_bridge field_viewer
```

`field_viewer` adalah jendela 2D ringan (bukan RViz) yang menampilkan robot sebenarnya
(hijau), estimasi lokalisasi (merah) dengan partikel MCL, bola sebenarnya dan bola estimasi,
state behavior, serta error lokalisasi. Di simulasi, gawang dan garis "terlihat" dari pose
sebenarnya ditambah noise (`sim_landmarks`), jadi lokalisasinya lebih akurat daripada di robot.

Dengan `vision:=gt`, kamera otomatis kecil (64×36) supaya Webots ringan. `ball_tracker`
mencatat kecepatan simulasi setiap 10 detik. Di bawah sekitar 0.7x, `op3_manager` (jam
nyata) dan Webots tidak sinkron, sehingga gerakan dan tendangan bisa tidak sampai.

## Skenario bola

```bash
ros2 run aroc_sim_bridge place_ball --bearing -15 --distance 1.0   # relatif robot, kiri +
ros2 run aroc_sim_bridge place_ball --x 1.5 --y 0.5                # koordinat lapangan
```

GoNasional memutar badan ke `saved_yaw + BALL_YAW_OFFSETS[index]` sebelum menendang, dan
`follow_head` memutar badan ke arah bola. Kalau bola tidak searah offset itu, keduanya saling
tarik dan robot tidak pernah menendang. Untuk tes tendangan, taruh bola dengan
`--bearing` = offset bola tersebut, **setelah** `mode` dan **sebelum** `start`.

## Vision (YOLO) di simulasi

Kamera simulasi dibuat 16:9 (default 640×360, `cam_width`/`cam_height`) seperti C920 robot
(1280×720), supaya `main_vision` memampatkannya ke 450×337 dengan distorsi yang sama.

```bash
ros2 launch aroc_sim_bridge bringup.launch.py vision:=yolo model:=<folder_model_openvino>
```

Dengan `vision:=yolo`, `/obj_detect` berasal dari `GoNasional vision`. Jarak dan sisi bola
tetap dari ground truth. `imgsz` dibaca dari `metadata.yaml` model, karena model OpenVINO
punya ukuran input tetap. Ground truth piksel bola ada di `/aroc_sim/ball_pixel`
(`x`, `y` dalam frame 450×337, `z` jarak cm).

Mengukur detektor (robot diam, kepala diarahkan otomatis):

```bash
ros2 run aroc_sim_bridge vision_eval --distances 0.5 1 2 3 4 --bearings -20 0 20 --label nama_model
```

Jika NumPy 2.x terpasang di `~/.local`, `cv_bridge` dan `torch` dari sistem bisa crash.
Launch di atas mendahulukan `/usr/lib/python3/dist-packages` untuk node vision. Kalau
menjalankan `GoNasional vision` sendiri, set `PYTHONPATH` yang sama.

## Dataset bola berlabel otomatis

Model YOLO yang dilatih dengan foto asli hampir tidak mendeteksi bola render Webots.
`dataset_gen` membuat dataset YOLO dari simulasi dengan label dari ground truth, untuk
digabung dengan dataset asli saat melatih ulang:

```bash
ros2 run aroc_sim_bridge dataset_gen --out ~/datasets/bola_sim --count 2000 --seed 1
```

`dataset_gen` menjalankan Webots sendiri (world `dataset_field.wbt`, tanpa window, sekitar
50 gambar/detik) dan tidak butuh `bringup`. Tiap gambar mengacak posisi, tinggi, arah,
dan tilt kamera, posisi dan rotasi bola, arah dan intensitas cahaya, serta noise kamera.
Latar diganti tiap `--relight-every` gambar (default 500; lambat karena teksturnya diunduh
lalu di-cache). Sekitar 10% gambar tanpa bola (`--neg-ratio`).

- Gambar JPG 450×337, dimampatkan dari kamera 16:9 seperti `main_vision` di robot
  (`--no-squash` untuk 640×360).
- Kotak dari siluet bola yang diproyeksikan dan dipotong di tepi gambar. Bola yang
  terlihat kurang dari 40% (`--min-visible`) atau lebih kecil dari 4 px (`--min-px`)
  tidak dipakai.
- `--class-id` mengikuti indeks kelas bola di dataset tim, misalnya `2` untuk dataset
  dengan kelas `GK, Robot, bola`. Kalau digabung, `data.yaml` diambil dari dataset tim.
- `--seed` berbeda menghasilkan gambar berbeda dengan nama file berbeda, jadi beberapa
  run bisa digabung dalam satu folder.

Belum ada robot lain atau penghalang di gambar, jadi dataset ini melengkapi dataset asli,
bukan menggantikannya.

## Catatan

- **Page tendang.** Page 155/156 (tendang kiri/kanan tim) kosong di motion file bawaan ROBOTIS.
  Launch membuat motion file khusus simulasi yang mengisi slot itu dengan `l_kick` (84) dan
  `r_kick` (83) bawaan. Page 15, 80, 142, dan 143 sudah sama dengan file bawaan.
- **Arah yaw.** Bridge memakai konvensi ENU (berlawanan jarum jam positif). Konvensi
  firmware OpenCR tim belum dipastikan. Kalau terbalik, ubah parameter `yaw_sign`.
- **Oklusi.** `ball_tracker` menganggap bola terlihat selama ada di dalam FOV kamera.
- **trigonocak.** Jangan jalankan bersamaan dengan sumber `/vision/bola_cx_cy`, karena
  keduanya menulis ke `/vision/hasil_servo`.

Lisensi Apache-2.0. Proto robot dan framework milik ROBOTIS, proto lapangan dan bola milik
Cyberbotics, diunduh dari repo masing-masing dan tidak disertakan di sini.
