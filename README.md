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
for r in ROBOTIS-OP3 ROBOTIS-OP3-msgs ROBOTIS-OP3-Simulations ROBOTIS-OP3-Common ROBOTIS-Framework ROBOTIS-Framework-msgs ROBOTIS-Math; do
  git clone --depth 1 -b jazzy-devel https://github.com/ROBOTIS-GIT/$r.git
done
git clone https://github.com/afifidf/op3-krsbi-sim.git aroc_sim_bridge
# Opsional, untuk team.launch.py: kamera OP3 bisa dimatikan (3 robot jauh lebih cepat)
git -C ROBOTIS-OP3-Simulations apply ../aroc_sim_bridge/patches/op3_extern_controller_camera_period.patch
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

## Dataset YOLO berlabel otomatis (bola + landmark)

Model YOLO yang dilatih dengan foto asli hampir tidak mendeteksi bola render Webots.
`dataset_gen` membuat dataset YOLO dari simulasi dengan label dari ground truth, untuk
digabung dengan dataset asli saat melatih ulang:

```bash
ros2 run aroc_sim_bridge dataset_gen --out ~/datasets/sim_multi --count 5000 --seed 1
```

Kelas (urutannya tetap; 1–4 sama dengan jenis landmark lokalisasi GoNasional):

| id | kelas | objek di simulasi |
|----|-------|-------------------|
| 0 | `bola` | bola |
| 1 | `tiang_gawang` | tiang gawang (silinder, tanpa mistar) |
| 2 | `titik_penalti` | tanda penalti, 1.3 m dari garis gawang |
| 3 | `titik_tengah` | tanda tengah lapangan |
| 4 | `sudut_lapangan` | pertemuan garis di sudut lapangan |
| 5 | `robot` | robot OP3 lain, berdiri atau jatuh |

`main_vision` hanya memakai deteksi bernama `bola`, jadi model 5 kelas bisa langsung dipakai.
`--no-landmarks` untuk label bola saja.

`dataset_gen` menjalankan Webots sendiri (world `dataset_field.wbt.in`, tanpa window, sekitar
3–4 gambar/detik karena fisika robot) dan tidak butuh `bringup`.

Empat robot OP3 ikut diacak. Tiap robot digerakkan controller kecil (`distractor_pose`)
yang memasang pose acak di sekitar pose siap jalan. Di sekitar 50% gambar ada robot lain
(`--robot-ratio`), berdiri atau jatuh, dan dilabel `robot`. Di sekitar 30% gambar
(`--self-ratio`), kamera dipasang tepat di posisi kamera kepala sebuah robot, sehingga saat
menunduk terlihat bahu, lengan, dan kaki robot sendiri (tidak dilabel). Tanpa contoh ini,
model lama menganggap kaki dan lengan robot sebagai bola dengan confidence sampai 0.95. Tiap gambar mengacak posisi, tinggi, arah,
dan tilt kamera, posisi dan rotasi bola, arah dan intensitas cahaya, serta noise kamera.
Latar diganti tiap `--relight-every` gambar (default 500; lambat karena teksturnya diunduh
lalu di-cache). Sekitar 10% gambar tanpa bola (`--neg-ratio`).

- Gambar JPG 450×337, dimampatkan dari kamera 16:9 seperti `main_vision` di robot
  (`--no-squash` untuk 640×360).
- Kotak = titik 3D objek yang diproyeksikan (permukaan bola, silinder tiang, kotak tetap di
  lantai untuk tanda garis), dipotong di tepi gambar. Objek yang terlihat kurang dari 40%
  (`--min-visible`) atau lebih kecil dari 4 px (`--min-px`) tidak dilabel.
- Oklusi tidak dihitung: tanda di belakang bola atau tiang di balik jaring tetap dilabel.
- Kamera diarahkan ke sekitar bola, jadi landmark lebih jarang muncul (sampel 300 gambar:
  bola 277, tiang 84, penalti 38, tengah 30, sudut 28).
- `--seed` berbeda menghasilkan gambar berbeda dengan nama file berbeda, jadi beberapa
  run bisa digabung dalam satu folder.

Dataset ini melengkapi foto asli dari kamera robot, bukan menggantikannya.

### Melatih

`training/merge_datasets.py` menggabungkan dataset simulasi dan dataset tim sambil
memetakan indeks kelas (kelas yang tidak dipetakan dibuang):

```bash
python3 training/merge_datasets.py --out ~/datasets/gabungan \
    --src ~/datasets/sim_multi --src ~/datasets/gkeep_sementara:2=0
```

`training/train_yolo.ipynb` untuk Colab atau Kaggle (GPU): gabung dataset, latih
`yolo11n` di `imgsz` 480, tampilkan mAP per kelas, lalu ekspor OpenVINO. Folder
`*_openvino_model` disalin ke `GoNasional/model/`, dan `main_vision` dijalankan dengan
`-p imgsz:=480`. Di simulasi: `bringup.launch.py vision:=yolo model:=<folder>`, lalu ukur
dengan `vision_eval`.

## Lokalisasi dengan kamera

Dengan model YOLO yang mengenali landmark, lokalisasi GoNasional memakai tiang gawang,
titik penalti, titik tengah, dan sudut lapangan dari kamera, bukan ground truth:

```bash
ros2 launch aroc_sim_bridge bringup.launch.py gui:=false vision:=yolo model:=<folder_model>
ros2 launch GoNasional aroc_soccer.launch.py sim:=true referee:=false landmarks:=vision
```

`vision_fps` (default 8) membatasi FPS YOLO supaya Webots tidak ikut melambat.

`loc_bench` mengukur error lokalisasi terhadap ground truth. Robot dipindah ke 6 pose
(`/aroc_sim/place_robot`, robot tetap berdiri), lalu kepala scan:

```bash
ros2 launch GoNasional aroc_soccer.launch.py sim:=true referee:=false landmarks:=vision behavior:=false
ros2 run aroc_sim_bridge loc_bench --mode offset   # estimasi awal digeser 0.6 m
ros2 run aroc_sim_bridge loc_bench --mode kidnap   # robot dipindah diam-diam
```

Hasil di laptop:
- Model `sim_multi`: offset median error 4 cm; kidnap, 5 dari 6 pose pulih dalam 9–40 s.
- Model `sim_robot` (6 kelas): offset median 9 cm; kidnap `--dwell 120`, 6 dari 6 pulih
  dalam 0–45 s, error median 9 cm. Sim berjalan 0.25x karena YOLO, jadi dwell diperpanjang.
  Model ini juga tidak lagi melihat kaki robot sendiri sebagai bola.

 Pose yang tidak melihat landmark sama sekali tidak bisa pulih, dan statusnya
turun ke UNCERTAIN setelah 15 s tanpa landmark.

## Tiga robot satu tim (team play)

```bash
ros2 launch aroc_sim_bridge team.launch.py gui:=false     # 3 OP3 + stack GoNasional masing-masing
ros2 run aroc_sim_bridge team_monitor --duration 360 --csv team.csv
ROS_DOMAIN_ID=12 ros2 topic echo /behavior/role           # lihat satu robot
```

Tiap robot jalan di `ROS_DOMAIN_ID` sendiri (11, 12, 13): op3_manager, bridge, dan stack
GoNasional robot itu tidak saling lihat di ROS. Mereka hanya berbagi info lewat UDP
`team_comm`, sama seperti robot asli di WiFi. Pemain 1 kiper (`goalie:=0` tanpa kiper),
bola mulai di (-0.6, 0.4) (`ball:=x,y`), jumlah robot `robots:=1..4`.

Vision memakai ground truth, dan kamera tidak dirender (`camera_period_ms: 0`, perlu patch
di atas). Tanpa patch, render kamera 3 robot tiap 8 ms membuat simulasi turun ke 0.15x
waktu nyata. Robot jatuh karena op3_manager memakai jam nyata. Dengan patch: 0.6–0.75x.

Hasil 6 menit di laptop:
- Peran: pemain 1 kiper 100%, pemain 2 striker 97%, pemain 3 defender 96%
  (sisanya striker sementara pemain 2 bangun).
- Dua striker sekaligus 0% dari sampel, tanpa striker 1%.
- Bola masuk gawang lawan setelah 206 s.
- Error lokalisasi median 1–6 cm, p90 maksimum 15 cm.

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
