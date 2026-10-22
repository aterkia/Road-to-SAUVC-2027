# Arming dan disarming Pixhawk

`helm` sekarang dapat meminta arm/disarm melalui MAVROS. Program hanya melapor
berhasil setelah Pixhawk menerima perintah dan telemetry menunjukkan status yang
diminta. Tidak ada arming otomatis saat launch atau saat koneksi tersambung lagi.

## Cara menjalankan

Setelah workspace dibangun dan environment ROS disiapkan, gunakan satu launch:

```bash
ros2 launch bootstrap arming.launch.py
```

Launch ini menggantikan `bench.launch.py`; jangan jalankan keduanya bersamaan.
Endpoint tetap dibaca dari `src/bootstrap/config/bench.yaml` dan secara bawaan
menggunakan `serial:///dev/ttyACM0:115200`. Argumen `config` dan `fcu_url` tetap
tersedia untuk konfigurasi kendaraan atau pengujian UDP.

Di terminal container lain yang sudah menyiapkan environment ROS:

```bash
# Mengizinkan motor bekerja.
ros2 service call /vehicle/arm std_srvs/srv/SetBool '{data: true}'

# Mencabut izin motor bekerja.
ros2 service call /vehicle/arm std_srvs/srv/SetBool '{data: false}'

# Melihat status aktual dari Pixhawk.
ros2 topic echo /vehicle/state --once
```

Arming dapat membuat motor bekerja. Pengujian bench pertama harus dilakukan
dengan propulsi dinonaktifkan dan sarana penghentian independen tersedia.
Implementasi tidak menggunakan force-arm dan tidak melewati pemeriksaan arming FCU.

## Perilaku yang diuji

- Launch telemetry biasa tetap read-only. `arming.launch.py` menambahkan plugin
  MAVROS `command` dan mengaktifkan parameter `helm.arming_enabled`.
- Arming memerlukan koneksi, state dan IMU yang masih baru. Disarming tetap
  dapat diminta saat IMU terlambat, selama koneksi dan state masih tersedia.
- Respons MAVROS harus menyatakan diterima, lalu state FCU harus mengonfirmasi
  `armed` sesuai permintaan. Respons diterima tanpa konfirmasi state tidak cukup.
- Permintaan untuk status yang sudah teramati tidak mengirim perintah tambahan.
- Penolakan FCU, service MAVROS tidak tersedia, koneksi putus dan timeout
  menghasilkan `success: false`. Timeout/disconnect berarti hasil belum dapat
  dipastikan; periksa telemetry atau gunakan kendali independen kendaraan.
- Batas tunggu bawaan adalah 6 detik (`helm.arming_timeout`), menggunakan waktu
  monotonic. Program tetap memproses telemetry selama menunggu respons.
- Hanya satu permintaan diproses pada satu waktu; permintaan bersamaan ditolak.
  Tidak ada retry otomatis. Respons terlambat tidak mengubah hasil yang sudah
  dikembalikan. Timeout tidak membatalkan perintah yang sudah dikirim ke FCU.

Pada launch ini, status health `ARMING_ONLY` berarti hanya arming/disarming yang
diaktifkan. Health tetap WARN/ERROR dan tidak mengeluarkan izin misi `ARMED`.
`/safety/abort` pada profil bench ini belum menghentikan kendaraan; disarm harus
diminta secara eksplisit. Adapter autonomous diuji terpisah; lihat
[progres dan bukti pengujian](autonomous-verification.md).

Plugin MAVROS `command` juga menyediakan service command umum. Pembatasan adapter
ini bukan access control ROS: node lain dapat memanggil service MAVROS secara
langsung. Gunakan `/vehicle/arm` untuk alur yang memeriksa konfirmasi state.

Pengujian memakai service ROS tiruan dan MAVROS nyata dengan peer MAVLink UDP di
loopback. Peer mengirim ACK dan heartbeat; ia bukan simulator dinamika ArduSub.
Uji arming pada Pixhawk nyata belum dilakukan oleh implementasi ini.

Referensi bridge: [MAVROS command plugin 2.16.0](https://github.com/mavlink/mavros/blob/2.16.0/mavros/src/plugins/command.cpp).
