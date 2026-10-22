# Verifikasi implementasi autonomous

Tanggal: 23 Oktober 2026.

Acuan: [proposal yang disetujui](proposals/autonomous-implementation.md).
PRD tetap menjadi sumber kebutuhan. Verifikasi hardware fisik belum dilakukan.

| Tahap | Status | Bukti |
| --- | --- | --- |
| 0 — Kontrak dan konfigurasi uji | Selesai untuk pengujian software; firmware uji dipisahkan dari hardware | [Kontrak interface](interfaces.md#onboard-ardusub-adapter-autonomous-proposal-stages-01), fixture SITL pada `tests/sitl/ardusub.parm` |
| 1 — Adapter dan pengaman | Lulus build, regresi lengkap, dan SITL | `artifacts/autonomous-stage1-verification.log`: 56 tes sistem lulus, satu tes SITL dilewati pada regresi biasa; SITL lulus terpisah pada `artifacts/autonomous-sitl-tests.log` |
| 2 — Countdown, depth, turn onboard | Lulus build, regresi lengkap dan SITL; termasuk satu entry point hardware | `artifacts/autonomous-stage2-verification.log`: 63 tes sistem lulus, 3 tes SITL dilewati pada regresi biasa; `artifacts/autonomous-stage2-launch-sitl.log` dan `artifacts/autonomous-stage2-stop.log` lulus terpisah |
| 3 — Kamera search/align/approach | Lulus build, regresi lengkap dan SITL; uji fisik belum dilakukan | `artifacts/autonomous-stage3-verification.log`, `artifacts/autonomous-stage3-guidance.log`, `artifacts/autonomous-final-sitl.log` |
| 4 — Bukti passage | Menunggu eksperimen kamera/geometri dan referensi independen | `PASS_GATE` hardware belum tersedia |
| 5 — Qualification hardware | Menunggu tahap 4 dan uji fisik | — |

## Lingkungan uji

Build pengembangan disimpan di `/tmp/aterkia-autonomous` dalam container
`ros2-lyrical`, terpisah dari `build/` dan `install/` milik pengguna. Tes memakai
domain ROS tersendiri dan hanya endpoint UDP localhost, tanpa serial Pixhawk.

Setelah regresi lulus, build/install workspace biasa juga diperbarui dan keenam
package berhasil dibangun (`artifacts/autonomous-workspace-build.log`). Interface
`VehicleCommand` dan argumen entry point diverifikasi dari install tersebut;
pemeriksaan ini tidak menjalankan kendaraan.

Firmware uji: binary resmi ArduSub 4.7.2, commit
`fb773567896014da007bca58358704d4a0d8516f`, SHA-256
`cc1ea9446f7a34388588c2398ffa17c914f43cd4d302cf187211a7bfc4204f2e`.
Ini pilihan firmware **SITL**, bukan penetapan versi firmware kendaraan fisik.
Sumber: [server firmware resmi](https://firmware.ardupilot.org/Sub/stable/SITL_x86_64_linux_gnu/).
Tes memeriksa hash sebelum mengeksekusi binary. Versi pada direktori `stable`
dapat berubah; binary yang berbeda ditolak sampai baseline pengujian diperbarui.

Uji firmware membuktikan perubahan depth lebih dari 0,3 m saat diberi input turun,
perubahan heading CCW lebih dari 0,15 rad saat diberi yaw positif, dan disarm FCU
setelah `helm` dihentikan sementara MAVROS masih mengirim heartbeat. Deadzone
vertikal di fixture dibuat lebih kecil daripada input uji. Nilainya harus
dikalibrasi kembali untuk kendaraan; batas input kecil saja tidak menjamin
kendaraan bergerak atau membatasi seluruh output stabilisasi FCU.

Jalankan setelah build dan source workspace, di environment ROS yang terisolasi:

```bash
export ROS_DOMAIN_ID=79
export ATERKIA_SITL_BINARY=/tmp/aterkia-sitl/ardusub
python3 -m pytest tests/test_sitl.py -q -s
```

Binary dapat diunduh dari tautan `ardusub` pada direktori firmware resmi di atas,
lalu diberi izin executable. Tes membutuhkan binary dengan hash yang dicatat.
Tanpa `ATERKIA_SITL_BINARY`, tes firmware dilewati secara eksplisit; tes ROS biasa
tetap berjalan. Kelulusan ROS tanpa tes firmware bukan kelulusan tahap 1.

Fixture `tests/sitl/ardusub.parm` hanya untuk simulator. Ia menonaktifkan pre-arm
checks untuk sensor ideal simulator; file ini tidak boleh diunggah ke Pixhawk
fisik. Adapter runtime tidak mengubah parameter FCU dan tidak memakai force-arm.
Versi firmware, sumber sensor tekanan air, datum, densitas, kamera, wiring, dan arah
motor kendaraan tetap perlu diverifikasi sebelum uji fisik.

## Skenario tahap 1

| Skenario | Kriteria lulus | Pengujian |
| --- | --- | --- |
| Startup atau FCU di-arm dari luar | Tidak ada command yang diteruskan, sesi tetap tertutup | `test_depth_preserves_source_stamp_and_startup_never_arms` |
| Command sah | Tanda surge/yaw/heave benar; jumlah absolut input dibatasi; tombol dan ekstensi nol | `test_axis_mapping_and_combined_input_limit` |
| Command pertama tidak datang atau stream berhenti | Netral satu kali, sesi terkunci, disarm; publisher yang kembali tidak melanjutkan sesi | `test_first_command_and_stream_timeout_disarm_without_auto_resume` |
| Command invalid, replay, timestamp masa depan, atau dua publisher | Sesi dikunci dan disarm | `test_invalid_replayed_commands_and_multiple_publishers_disarm` |
| Mode hanya dinyatakan terkirim, atau arm selesai setelah abort | Tanpa state mode yang sesuai, persiapan gagal; respons arm terlambat tidak membuka sesi | `test_mode_needs_state_confirmation_and_abort_cancels_late_arm` |
| Sensor, mode, atau koneksi berubah | Gerakan dihentikan; reconnect tidak membuka sesi | `test_sensor_loss_mode_change_and_disconnect_latch_the_session` |
| Konfigurasi failsafe, sender, atau frame salah | Persiapan dan arming ditolak | `test_fcu_configuration_must_match_failsafe_sender_and_frame` |
| Bridge mati tetapi heartbeat MAVROS tetap ada | Firmware melakukan disarm berdasarkan kehilangan pilot input | `test_alt_hold_depth_yaw_feedback_and_firmware_input_failsafe` pada SITL |

Tujuh skenario ROS berada di [tes adapter](../tests/test_vehicle_control.py).
Uji dinamika firmware berada di [tes SITL](../tests/test_sitl.py).

## Tahap 2: misi onboard

Misi SITL menyelesaikan `DIVE_TO_DEPTH → TURN` melalui countdown, preflight,
konfirmasi `ALT_HOLD`, arming, Action, dan konfirmasi disarm. Satu-satunya
publisher `/vehicle/command` adalah `wayfinder`; tidak ada publisher command
pengganti operator pada tes ini. Rekaman status ada di
`artifacts/sitl-1eqkysqj/mission.txt`.

SITL meminta SCALED_PRESSURE 10 Hz dan ATTITUDE 20 Hz melalui service MAVROS
sebelum pengujian. Laju ini diperlukan agar batas freshness 0,5 detik terpenuhi;
stream bawaan simulator tidak dijadikan jaminan laju telemetry kendaraan.
Pada kendaraan fisik, laju stream dan timeout kedua node harus disesuaikan
berdasarkan pengukuran saat commissioning.

Tes tambahan mencakup countdown gagal/abort tanpa auto-start ketika pulih,
depth yang tidak pernah tercapai, kedalaman stabil dari beberapa sampel,
heading melewati batas ±π, depth dengan timestamp beku, serta matinya
`wayfinder` saat heartbeat MAVROS tetap hidup.

Regresi startup menunggu readiness dari `wayfinder` sebelum mengirim goal.
Pemeriksaan cadence command pada bag memakai timestamp pengiriman DDS, bukan
waktu penerimaan recorder. Pada rekaman `recording-mock-e5puy4sp`, gap penerimaan
mencapai 0,661 detik sedangkan gap pengiriman maksimum 0,050 detik. Batas watchdog
mock tetap 0,4 detik; timestamp penerimaan masih disimpan untuk analisis recorder.

Override Compose lulus `docker compose config --quiet`; contoh service lulus
`systemd-analyze verify`. File deployment belum diaktifkan pada host atau AUV.

## Tahap 3: kamera sampai pendekatan gate

Misi `DIVE_TO_DEPTH → APPROACH_GATE` lulus terhadap firmware SITL dengan gambar
sintetis melalui `spectra` dan kalibrasi uji. Search menghasilkan yaw positif,
gate di kanan menghasilkan yaw negatif, dan gate sejajar menghasilkan surge.
Pengujian memeriksa feedback IMU dan komponen output FCU untuk surge. Pose
horizontal tidak tersedia; pengujian tidak mengasumsikan perpindahan atau
crossing dari command maupun PWM.

Saat ambang pendekatan tercapai, surge dinetralkan dan hasil menunggu beberapa
frame baru pada rentang waktu stabil. Frame berulang dan kamera hilang diuji
sebagai kegagalan. Hasil task adalah pendekatan visual, bukan `PASS_GATE`.
Normal completion mengakhiri misi dengan konfirmasi disarm.

Bukti awal: `artifacts/sitl-2bwr84td/mission.txt` dan rekaman MCAP
`artifacts/recording-autonomous-opm8uexz/session/`, termasuk gambar, deteksi,
command bertimestamp, depth, IMU, output FCU dan status misi.

Tahap 4–5 menunggu kamera fisik terkalibrasi, geometri gate terukur, referensi
independen untuk crossing, serta commissioning kendaraan. Tidak ditambahkan
odometri palsu, SLAM, sensor baru, atau payload untuk meloloskan prasyarat ini.

## Hasil akhir software tahap 0–3

Seluruh enam package berhasil dibangun. Regresi melalui CTest lulus:
66 tes sistem, 4 tes C++, dan 6 tes package Python. Empat skenario firmware
SITL yang dilewati pada regresi biasa juga dijalankan terpisah dan semuanya
lulus (`artifacts/autonomous-final-sitl.log`). Total 80 kasus aktual lulus;
angka agregat `colcon test-result` juga menghitung wrapper CTest.

Setiap file tes ROS dijalankan dalam proses Python/DDS tersendiri melalui CTest,
secara serial pada domain pengujian. Fixture FCU mengirim telemetry melalui
executor terpisah dari client service. Pemisahan ini menjaga sensor tiruan tetap
berjalan selama pengujian request/ACK, tanpa melonggarkan batas watchdog runtime.

Hasil monolitik lama pada build pengembangan dipindahkan ke
`artifacts/autonomous-legacy-system-test.xml` karena nama tes CTest-nya sudah
berubah. Hasil tersebut bukan gerbang kelulusan akhir.
