# Bench MAVROS: koneksi dan telemetry

Untuk merekam bukti koneksi, telemetry, disconnect dan konfigurasi aktual,
gunakan profil bench pada [panduan rekaman sesi](recording.md).

Adaptasi gagasan 2 dari [riset](research/openmantaclaus-review.md), sesuai
keputusan operator: versi firmware, endpoint dan mode kendali belum final.
`helm_mavros_monitor` menyediakan pembacaan telemetry dan kontrak command yang
terkunci secara bawaan. `bench.launch.py` hanya menjalankan MAVROS dan monitor
`helm`. Gunakan [arming.launch.py](arming.md) untuk mengaktifkan permintaan
arming/disarming dengan konfirmasi state; gerakan tetap belum diaktifkan.

## Dependency dan launch

Gunakan paket ROS dari satu set repository/ABI yang konsisten. MAVROS membutuhkan
dataset geoid saat startup meskipun pengujian hanya memakai IMU/pressure:

```bash
sudo apt-get update
sudo apt-get install ros-lyrical-mavros ros-lyrical-mavros-msgs
sudo geographiclib-get-geoids egm96-5
source /opt/ros/lyrical/setup.bash
source scripts/ros_env.sh
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
```

Image Lyrical lama dapat memuat interface/type-support dengan ABI yang berbeda
dari paket MAVROS terbaru. Pada container validasi, penambahan MAVROS 2.16.0
memerlukan pembaruan paket pesan/interface ROS, generator dan Fast DDS
type-support secara bersama, lalu build ulang workspace. Error seperti
`undefined symbol: has_buffer_fields_...` menandakan underlay tidak konsisten,
bukan kegagalan koneksi Pixhawk.

Endpoint serial bawaan disimpan di `src/bootstrap/config/bench.yaml` pada
`/mavros_node.ros__parameters.fcu_url`: `serial:///dev/ttyACM0:115200`.
Port dan baud rate tersebut belum diverifikasi pada Pixhawk nyata. Sesuaikan
nilainya sekali di YAML jika perangkat berbeda, lalu build ulang `bootstrap`.
Untuk penggunaan sehari-hari, cukup jalankan:

```bash
ros2 launch bootstrap bench.launch.py
```

Jika memakai Docker, port serial harus tersedia dan dapat diakses oleh user di
dalam container. Konfigurasi `/home/lain/ros2-docker/compose.yaml` meneruskan
`/dev/ttyACM0` dan menambahkan grup akses dari `SERIAL_GID` pada `.env`.
Nilainya harus sesuai `stat -c %g /dev/ttyACM0` pada host. Sambungkan Pixhawk
sebelum menjalankan Compose. Setelah perubahan Compose, buat ulang container
dengan `docker compose up -d --force-recreate ros2` dari direktori tersebut;
image tidak perlu dibangun ulang. Jika USB dicabut dan disambungkan kembali,
buat ulang container apabila mapping perangkat lama tidak lagi bekerja.

Konfigurasi kendaraan lain atau endpoint UDP tetap dapat dipilih saat diperlukan:

```bash
# Gunakan endpoint serial dari file konfigurasi kendaraan.
ros2 launch bootstrap bench.launch.py \
  config:=/absolute/path/bench.yaml

# UDP bind lokal dan remote harus mengikuti konfigurasi FCU/SITL.
ros2 launch bootstrap bench.launch.py \
  config:=/absolute/path/bench.yaml fcu_url:=udp://127.0.0.1:14550@127.0.0.1:14555
```

Tanpa argumen endpoint, launch membaca `/mavros_node.ros__parameters.fcu_url`
dari YAML bawaan atau file yang dipilih melalui `config`. Argumen `fcu_url`
menggantikan nilai YAML bila diberikan; nilai kosong pada YAML tanpa pengganti
menggagalkan launch. Target system/component juga
parameter MAVROS; ID 1 pada template harus diverifikasi untuk kendaraan.
`helm.firmware_family`, `firmware_version`, dan `control_mode` menyimpan
konfigurasi yang dipilih operator, bukan hasil pendeteksian firmware. Mode
aktual datang dari `/vehicle/state`. Parameter read-only selama node hidup;
ubah YAML dan relaunch untuk mengganti konfigurasi.

Launch membatasi plugin ke `sys_status`, `sys_time`, `imu` dan menonaktifkan
forwarding GCS serta plugin command/RC/setpoint. MAVROS tetap mengirim heartbeat,
sinkronisasi waktu dan permintaan informasi protokol; ini bukan silent serial
sniffer. Service administratif bawaan `sys_status`, termasuk `/mavros/set_mode`,
masih bagian dari MAVROS. Jangan memanggilnya pada tahap ini. Pembatasan launch
adalah batas aplikasi, bukan access control ROS terhadap node lain yang sengaja
mengirim MAVLink. Hentinya salah satu proses menutup launch bench.

Pada MAVROS 2.16.0, penonaktifan plugin `command` juga membuat permintaan
capabilities/version otomatis oleh `sys_status` melaporkan timeout/service
unavailable. Ini tidak menghentikan telemetry; jangan membuka plugin command
untuk menghilangkan log tersebut. Versi firmware belum diverifikasi otomatis.

## Data yang tersedia

| Input default (dapat diubah via parameter) | Output | Semantik |
| --- | --- | --- |
| `/mavros/state` | `/vehicle/state`, `/vehicle/health` | Koneksi, armed, mode, manual_input yang diamati; bukan izin menjalankan misi |
| `/mavros/imu/data` | `/vehicle/imu`, `/vehicle/heading` | Quaternion ENU/body FLU sudah dikonversi MAVROS; heading yaw radian, nol timur, positif ke utara |
| `/mavros/imu/static_pressure` | `/vehicle/pressure` | Pressure absolut Pa, header sumber dipertahankan |
| Pressure yang telah dikonfirmasi operator | `/vehicle/depth` | Opsional, meter positif turun terhadap permukaan terukur |

Tidak diterbitkan odometri atau applied command. Orientasi Pixhawk tidak
menghasilkan displacement horizontal. Heading/pressure membutuhkan koneksi
fresh, timestamp yang maju dan masuk batas umur, frame `imu_frame`, serta
angka finite; quaternion harus bernorma satu. Timeout menggunakan waktu sumber
dan waktu penerimaan monotonic. Pengulangan pesan lama tidak menjaga freshness.
State memakai reliable QoS; IMU/pressure sensor best effort. Field Float64
depth/heading mengikuti kontrak lama dan hanya diterbitkan saat input valid;
gunakan telemetry stamped dan health untuk memeriksa freshness.

`/vehicle/health` selalu WARN/ERROR pada bench. Pesan `READ_ONLY` berarti koneksi
dan telemetry yang diwajibkan tersedia. `TELEMETRY_STALE` menunjukkan input
terlambat/invalid; `DISCONNECTED` menunjukkan heartbeat hilang/terputus. Field
diagnostic mencatat `observed_armed`, `observed_mode`, `manual_input`,
`imu_fresh`, `pressure_fresh`, dan `depth_available`. State `ARMED` yang biasa
mengizinkan misi tidak pernah dikeluarkan monitor, walaupun FCU armed dari luar.
`manual_input` hanya observasi flag MAVLink, bukan bukti manual takeover bekerja.

### Identitas depth sensor harus diperiksa

Plugin IMU MAVROS menerjemahkan `SCALED_PRESSURE`/`HIGHRES_IMU` ke static pressure.
Itu bisa merupakan barometer internal Pixhawk. Sensor underwater tertentu
mungkin dikirim melalui stream lain; jangan menganggap adanya pressure berarti
sensor depth yang benar sudah terbaca. Bila topic pressure yang benar tersedia,
pilih `pressure_topic` dan pastikan frame cocok. Sampai itu dibuktikan,
`pressure_is_depth_sensor: false` dan tidak ada output depth.

Untuk sumber pressure yang terverifikasi, isi `surface_pressure_pa` hasil ukur
di permukaan, `water_density_kg_m3` sesuai air pengujian, lalu aktifkan
`pressure_is_depth_sensor`. Kedua nilai harus positif dan finite. Depth dihitung
`(pressure - surface_pressure) / (density * gravity)`; nilai negatif kecil tidak
dipotong agar bias permukaan terlihat. Tidak ada asumsi datum dari altitude MSL
atau depth konstan mock. Kalibrasi tersebut belum tersedia untuk kendaraan nyata.

## Interface pada profil bench read-only

- `/vehicle/arm` (`SetBool`): baik arm maupun disarm mengembalikan `success=false`.
- `/vehicle/set_mode` (`Trigger`): menolak perubahan mode; `control_mode` belum dieksekusi.
- `/cmd_vel` (`Twist`): diterima untuk melaporkan penolakan, tidak diteruskan.
- `/safety/abort` (`Trigger`): `success=false`; tidak mengaku berhasil menghentikan
  atau men-disarm perangkat yang belum dapat dikendalikan adapter ini.

Pada profil bench bawaan, monitor tidak membuat client/publisher MAVROS command.
`control_enabled: true` menggagalkan startup; parameter tidak dapat diubah saat
runtime. Mengisi mode atau
versi firmware tidak membuka kendali. `auv.launch.py mode:=hardware` tetap ditolak.
Untuk perangkat yang armed oleh sistem lain, penghentian tetap melalui sarana
independen kendaraan yang sudah tersedia.

## Verifikasi dan langkah commissioning berikutnya

`tests/test_bench.py` menguji normalisasi telemetry, default depth nonaktif,
data invalid/stale/replay, disconnect, serta seluruh interface command yang
terkunci. Tes UDP menjalankan MAVROS sebenarnya dengan peer MAVLink loopback
berbasis header MAVLink resmi (hanya dibangun saat `BUILD_TESTING`). Peer tidak
mensimulasikan firmware/dinamika ArduSub. Tes membuktikan arah NED→ENU sekali,
konversi hPa→Pa, hilangnya koneksi, serta tidak adanya paket aktuator saat
interface command dipanggil. Log ada di `artifacts/bench-*/`.

```bash
source scripts/ros_env.sh
ROS_DOMAIN_ID=77 python3 -m pytest tests/test_bench.py -v -s
```

Sebelum kendali diaktifkan pada perubahan berikutnya: pin firmware dan MAVROS,
verifikasi endpoint/sensor/mode pada SITL atau bench thruster nonaktif, tetapkan
setpoint serta neutral/hold, uji acknowledgement arming/disarming, watchdog,
manual override dan failsafe FCU. Peer loopback tidak menutup acceptance hardware
M1/M3, dan depth tetap tidak membuktikan gate passage.

Referensi implementasi bridge: [MAVROS IMU](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/imu.cpp),
[state](https://github.com/mavlink/mavros/blob/ros2/mavros_msgs/msg/State.msg), dan
[system status](https://github.com/mavlink/mavros/blob/ros2/mavros/src/plugins/sys_status.cpp).
