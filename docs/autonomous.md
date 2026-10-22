# Menjalankan misi autonomous onboard

Program misi berjalan pada komputer di dalam AUV. Ia menentukan gerak dari
sensor dan mengirim command melalui `wayfinder → helm → MAVROS → ArduSub`.
Profil awal menjalankan `DIVE_TO_DEPTH → TURN`; qualification hardware belum
tersedia. [Progres verifikasi](autonomous-verification.md) mencatat tahap yang
sudah lulus.

Urutan `DIVE_TO_DEPTH → APPROACH_GATE` juga tersedia setelah kamera dan
kalibrasinya dikonfigurasi. `PASS_GATE` hardware tetap belum tersedia.

## Konfigurasi kendaraan

Endpoint serial tetap dibaca dari `src/bootstrap/config/bench.yaml`, dengan
bawaan `serial:///dev/ttyACM0:115200`. Pengaturan autonomous berada pada
`src/bootstrap/config/autonomous.yaml`; endpoint tidak perlu ditulis di command.

Isi konfigurasi hardware sebelum memakainya:

- `firmware_version`: versi ArduSub yang benar-benar terpasang.
- `pressure_topic` bila berbeda dari bawaan bench: sumber pressure yang sudah
  dibuktikan berasal dari sensor tekanan air.
- `pressure_is_depth_sensor: true`, `surface_pressure_pa`, dan
  `water_density_kg_m3`: hasil verifikasi sensor dan kalibrasi permukaan.
- Target depth, toleransi, gain, batas input dan timeout: hasil uji kendaraan.
  Angka awal pada file merupakan konfigurasi pengujian software.
- `start_delay_s`: waktu persiapan setelah program misi berjalan; bawaan 30 detik.

Adapter memeriksa parameter FCU tanpa mengubahnya: frame Vectored-6DoF,
ID sender yang sesuai, pilot-input failsafe disarm, timeout >0 dan <=2 detik,
serta RC throttle failsafe tidak bertentangan. Gain joystick, deadzone, wiring,
arah motor dan laju telemetry tetap perlu diuji secara fisik.

## Launch

Setelah build dan source workspace, jalankan:

```bash
ros2 launch bootstrap auv.launch.py mode:=hardware
```

Konfigurasi bawaan masih menandai firmware dan sensor air sebagai belum
terverifikasi. Launch menolak konfigurasi tersebut sebelum menjalankan node.
Setelah data konfigurasi dilengkapi, launch ini **memulai countdown otomatis**.
Jangan menjalankan launch bench bersamaan dengan profil autonomous.

Alurnya:

1. Program onboard mulai dalam keadaan disarmed dan menghitung waktu persiapan.
2. Operator memasukkan AUV ke air selama countdown.
3. Setelah timer selesai, program memeriksa kesiapan guidance dan telemetry,
   memilih `ALT_HOLD`, lalu meminta arming dan menunggu konfirmasi.
4. Program menjalankan tugas sesuai urutan pada YAML.
5. Saat selesai atau gagal, sesi command ditutup dan disarm diminta. Status akhir
   membedakan disarm terkonfirmasi dari hasil yang belum dapat dipastikan.

Countdown memakai waktu monotonic dan dimulai sesudah konfigurasi program misi
dimuat. Waktu boot OS/container tidak mengurangi 30 detik persiapan. Timer tidak
mendeteksi apakah AUV sudah berada di air. Bila preflight gagal saat timer habis,
start dibatalkan dan tidak dimulai lagi ketika sensor pulih.

`/mission/start` tidak dapat melewati countdown hardware. `/mission/abort` dan
`/safety/abort` tersedia untuk pengujian/penghentian; misi normal tidak bergantung
pada komputer luar. Setelah abort, kegagalan, atau misi selesai, program tidak
mengulang countdown sendiri.

## Mengaktifkan pendekatan visual

Untuk misi kamera, ubah `orchestrator.tasks` menjadi
`[DIVE_TO_DEPTH, APPROACH_GATE]`. Lengkapi `spectra.calibration_file`, pilih
`spectra.input_geometry` sesuai driver (`raw` atau `rectified`), dan pertahankan
`wayfinder.use_camera_info: true`. Isi `bootstrap.camera_launch_file` dengan
path launch Python driver agar driver ikut dijalankan oleh entry point yang sama.
Driver harus menerbitkan `/camera/image_raw` dalam frame yang dikonfigurasi.

Preflight memerlukan deteksi dan CameraInfo segar yang mempunyai timestamp sama.
Tanpa data kamera terkalibrasi, program membatalkan start. Saat berjalan, guidance
mencari gate, mengoreksi yaw dari bearing, lalu maju ketika sejajar dan depth
sesuai. Surge berhenti saat ambang pendekatan visual tercapai; completion
memerlukan beberapa frame baru pada rentang waktu stabil.

Ambang lebar gate masih harus dinilai pada rekaman kendaraan. Pendekatan tidak
membuktikan crossing. Kamera hilang atau data berulang mengakhiri task dengan
gagal dan misi meminta disarm. [Recording](recording.md) menyediakan profil
`autonomous` untuk merekam seluruh jalur tersebut.

## Startup ketika daya dinyalakan

[Override Compose](../deploy/compose.autonomous.yaml) menjadikan launch autonomous
sebagai proses utama container. Ia memakai service `ros2` dan volume workspace
dari Compose yang sudah tersedia pada `ros2-docker`. Konfigurasi ini memakai
`restart: "no"`; kematian proses tidak boleh mengulang countdown.

Pada companion yang memakai systemd, [contoh service](../deploy/aterkia-autonomous.service)
memulai container `ros2-lyrical` sekali saat boot. Container harus dibuat terlebih
dahulu memakai Compose dasar dan override tersebut, dengan workspace yang sudah
dibangun serta konfigurasi hardware lengkap. Penghentian service menghentikan
container melalui SIGINT; failsafe FCU tetap diperlukan jika seluruh proses mati.

File deployment disediakan untuk konfigurasi companion. Service belum dipasang
atau diaktifkan pada host pengembangan, dan uji power-on fisik belum dilakukan.

## Build dan bukti pengujian

```bash
source /opt/ros/lyrical/setup.bash
bash scripts/verify.sh
source install/setup.bash
```

Regresi diverifikasi dengan build terpisah di `/tmp/aterkia-autonomous` dalam
container. Setelah lulus, `build/` dan `install/` workspace biasa juga dibangun
ulang. Gunakan `source install/setup.bash` untuk hasil build workspace tersebut.

SITL merupakan pengujian terpisah dengan firmware yang hash-nya dipatok;
lihat [cara menjalankan dan batas buktinya](autonomous-verification.md).
