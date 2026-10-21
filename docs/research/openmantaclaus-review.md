# Riset OpenMantaClaus untuk Aterkia

Tanggal: 22 Oktober 2026. Status: **catatan riset dan usulan**, belum menjadi
keputusan implementasi atau perubahan PRD.

Tindak lanjut terpisah: operator kemudian mengizinkan adaptasi gagasan 1 dan 2.
Implementasinya dijelaskan di [kalibrasi kamera](../camera-calibration.md) dan
[bench MAVROS](../mavros-bench.md), dengan hasil uji pada
[milestone ledger](../milestones.md). Operator kemudian juga mengizinkan gagasan 3,
yang diadaptasi sebagai [rekaman sesi dan replay](../recording.md).
Gagasan 4–7 tetap ditunda. Catatan riset di
bawah mempertahankan lingkup dan temuan snapshot awal.

Referensi: [kushagra77/OpenMantaClaus](https://github.com/kushagra77/OpenMantaClaus),
branch `main`, commit
[`f05f646aade941b8c35b3bb166d9c236fb181daf`](https://github.com/kushagra77/OpenMantaClaus/commit/f05f646aade941b8c35b3bb166d9c236fb181daf)
(11 Agustus 2026). Semua tautan kode di bawah dipatok pada snapshot ini.
Pembanding Aterkia: commit `6354a9912f58e791119c3d18e0b46234fc18e708`,
[PRD](../../PRD.md), [kontrak runtime](../interfaces.md), serta
[batas hardware dan M4](../hardware-and-finals.md).

## Kesimpulan

Ada gagasan yang layak diadaptasi, terutama untuk menjembatani mock ke kendaraan
Pixhawk/ArduSub dan meningkatkan persepsi pada rekaman kolam. Prioritasnya ialah
kalibrasi kamera, validasi bridge kendaraan secara terpisah, dan rekaman yang
mencakup seluruh jalur sensor–keputusan–aktuator. Semuanya dapat masuk ke package
yang sudah ada. Belum ada alasan untuk menambahkan package runtime baru sekarang.

OpenMantaClaus juga memakai pemetaan dan kontrol yang sangat bergantung pada
kendaraan serta arena asalnya. Bagian tersebut menjadi bahan pembanding,
bukan fondasi pengganti arsitektur Aterkia. PRD tetap sumber utama kebutuhan.

## Cara dan batas riset

Dilakukan pembacaan statis atas README, arsitektur, konfigurasi, bridge MAVROS,
persepsi, kontrol gate/gripper, odometri, bagian inisialisasi/pembaruan EKF,
skrip kalibrasi/rekaman, CI, serta metadata model dan lisensi. Keterangan dari
dokumentasi dibandingkan dengan kode terkait; angka pada komentar tidak selalu
sama dengan kode atau YAML.

Tidak ada kode upstream yang dijalankan, model yang dibenchmark, pengujian
hardware, atau dataset eksternal yang diaudit. Klaim hasil kompetisi/performa
pada README adalah laporan penulis proyek, bukan hasil verifikasi riset ini.
Tidak ada source code, dependency, konfigurasi runtime, atau PRD Aterkia yang
diubah dalam pekerjaan ini.

## Pemetaan tanggung jawab

Sumber alur upstream: [arsitektur](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/docs/architecture.md).

| Bagian upstream | Hubungan dengan Aterkia | Batas adaptasi |
| --- | --- | --- |
| `brain` | `orchestrator` | Urutan misi dan penanganan hasil; tetap gunakan Action Aterkia untuk identitas/cancel/result. |
| `cv` | `spectra` | Kalibrasi, inference, filtering, label; pemilihan/forwarding tugas tetap milik `orchestrator`. |
| `tasks` | `wayfinder` dan `helm` | Guidance masuk `wayfinder`; RC/servo, mode, arming, dan konversi kendaraan masuk `helm`. |
| `mavros_control` | Adapter hardware di `helm`; wiring di `bootstrap` | MAVROS menjadi bridge eksternal, bukan protokol MAVLink buatan sendiri. |
| `manta_bringup` | `bootstrap` | Pisahkan skenario uji melalui launch/config tanpa membuat node bisnis baru. |
| `ekfslam` | Kandidat estimasi keadaan pada tahap lanjut | Di luar MVP menurut PRD §2.2/FR-15; membutuhkan keputusan sensor dan bukti kebutuhan. |
| `interfaces` | `aterkia_interfaces` | Jangan mengganti kontrak standar/Action hanya karena upstream memakai message/service sendiri. |

## Gagasan yang layak diadaptasi

### 1. Kalibrasi kamera dan bearing yang memiliki arti fisik — prioritas awal

**Bukti upstream.** Skrip kalibrasi menghitung intrinsics/distortion dan
reprojection error per gambar. Preprocessing memakai peta undistorsi yang
disiapkan terlebih dahulu. Konversi posisi horizontal objek ke bearing memakai
focal length dan principal point kamera hasil koreksi. Sumber:
[kalibrasi](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/scripts/chessboard_calibration.py),
[camera control](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/cv/cv/utils/camera_control.py),
[konversi bearing](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/cv/cv/utils/filter_detections.py#L106-L117).

**Kecocokan.** Guidance Aterkia saat ini memakai error horizontal yang
dinormalisasi terhadap lebar gambar. Itu cukup untuk fixture mock, tetapi belum
memakai intrinsics kamera nyata. Kalibrasi membantu menghubungkan error visual
dengan sudut target tanpa menganggap principal point tepat di tengah gambar.

**Usulan.** Simpan alat kalibrasi offline di `scripts/`; gunakan `spectra` untuk
preprocessing yang sesuai. Data kalibrasi dapat mengikuti `sensor_msgs/CameraInfo`.
Jika `wayfinder` kemudian membutuhkan bearing, hitung dari deteksi dan intrinsics
yang konsisten. Pertahankan `Detection2DArray`; custom observation baru hanya
dibutuhkan bila ada konsumen dengan kebutuhan tambahan yang terukur.

**Syarat uji.** Gunakan kamera, resolusi, fokus, dan housing yang benar-benar
dipakai. Evaluasi reprojection error dan bearing pada beberapa posisi; uji sign
kanan/kiri, perubahan resolusi, serta timestamp. Hindari koreksi dua kali jika
driver sudah menyediakan gambar rectified. Jangan menyalin matriks kalibrasi
atau nilai exposure kendaraan referensi.

### 2. Uji bridge MAVROS tanpa menjalankan misi — prioritas awal

**Bukti upstream.** Ada launch khusus MAVROS dan utility yang membaca state/IMU,
meminta mode/arming, serta mengirim RC override. Dengan demikian jalur FCU bisa
diamati terpisah dari persepsi dan misi. Sumber:
[MAVROS-only launch](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/manta_bringup/launch/only_mavros.launch.py),
[controller utility](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/mavros_control/mavros_control/controller.py).

**Usulan.** Tambahkan skenario launch bench di `bootstrap` ketika adapter hardware
mulai dikerjakan. `helm` menjadi satu-satunya pemilik perintah kendaraan dan
normalisasi telemetry. Awali dari koneksi serta telemetry; verifikasi arming,
mode, frame, neutral/hold, dan watchdog sebelum menghubungkan guidance.

**Syarat uji.** Tetapkan versi firmware/bridge dan mode kendali terlebih dahulu.
Uji dalam SITL atau bench dengan thruster tidak aktif. Cocokkan tanda sumbu,
satuan, acknowledgement layanan, data stale, putus koneksi, dan manual override.
Jangan menyalin channel PWM, endpoint UDP, konfigurasi frame, atau urutan
auto-arm upstream. Periksa konversi yang sudah dilakukan bridge agar ENU/NED
tidak dikonversi dua kali. Ini belum menentukan bahwa RC override adalah
antarmuka akhir yang tepat untuk Aterkia.

### 3. Rekaman dari input sampai keluaran kendaraan — prioritas awal

**Bukti upstream.** Skrip rekaman mengumpulkan kamera, observasi, state MAVROS,
IMU, RC input/output, TF dan debug; kode aktual membatasi durasi menjadi 900 detik
serta menghentikan recorder dengan SIGINT. Sumber:
[record_bag.sh](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/scripts/record_bag.sh).

**Kecocokan/usulan.** Aterkia sudah memiliki rosbag qualification dan replay.
Manfaat tambahan ialah memperluas bukti saat integrasi hardware: rekam command
sebelum adapter, feedback/output yang tersedia, health, mode/arming, serta versi
konfigurasi dan firmware. Tempatnya `scripts/` dan konfigurasi `bootstrap`;
tidak perlu package recorder baru.

**Syarat uji.** Pastikan rekaman bisa dibuka setelah penghentian, timestamp
dapat dibandingkan, storage cukup, dan recorder tidak membuat kontrol kehilangan
deadline. Adaptasikan daftar topik, QoS dan sintaks CLI ke Lyrical. Replay persepsi
tetap hanya memasukkan sumber sensor yang dipilih; output kendaraan terekam
tidak boleh menjadi input kendali aktif.

### 4. Filtering geometri dan klasifikasi warna di dalam ROI — setelah dataset kolam ada

**Bukti upstream.** Bounding box disaring memakai confidence, ukuran, rasio,
posisi, dan konsistensi antarobjek. Warna flare/bucket diperiksa pada crop objek,
dengan hasil unknown bila dukungan warnanya tidak cukup. Sumber:
[filter_detections.py](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/cv/cv/utils/filter_detections.py).

**Usulan.** Perluas fungsi di `spectra` bila rekaman menunjukkan false positive
atau kegagalan warna. Nilai kualitas geometri yang kita miliki tetap dibedakan
dari probabilitas terkalibrasi. Untuk target final, deteksi bentuk/objek dapat
mengusulkan ROI, kemudian warna diuji hanya di ROI. Pakai unknown/reject untuk
hasil ambigu. Jangan menyamakan label `bucket` upstream dengan `drum` PRD tanpa
memeriksa objek, anotasi, dan kebutuhan misi.

**Syarat uji.** Bandingkan terhadap baseline saat ini pada rekaman berlabel yang
tidak dipakai untuk tuning: target parsial, pantulan, objek sewarna, gelap/keruh,
dan beberapa target sekaligus. Ukur false positive serta ketepatan label/posisi.
Jika menambah konfirmasi beberapa frame, hasil lama tetap harus kedaluwarsa;
filter temporal tidak boleh mempertahankan target sebagai data segar.

### 5. Detektor terkuantisasi sebagai eksperimen di `spectra` — bersyarat

**Bukti upstream.** Inference TFLite memakai model path eksplisit, jumlah thread,
letterbox, konversi koordinat kembali ke gambar asal, dan parameter kuantisasi.
Metadata model generalist menyebut input 256×256, empat kelas, dan INT8. Sumber:
[helper inference](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/cv/cv/utils/yolo.py),
[pipeline](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/cv/cv/cv_node.py),
[metadata model](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/scripts/yolo/runs/detect/pretrain_generalist_v2/weights/best_saved_model/metadata.yaml).

**Usulan.** Pertimbangkan hanya bila evaluasi menunjukkan HSV/bentuk belum
memadai. Lakukan eksperimen offline dahulu; jika unggul, pertahankan kontrak
deteksi dan boundary `spectra`. Tidak perlu framework plugin/factory atau package
per-model hanya untuk membandingkan dua metode.

**Syarat uji.** Periksa format tensor, label, preprocessing dan dekuantisasi;
uji pemetaan box pada aspect ratio berbeda. Ukur kualitas deteksi, latensi dari
timestamp gambar ke output, beban CPU/memori, dan dampaknya pada timeout di
companion computer kita. Angka FPS dan speed scale kendaraan referensi tidak
menjadi target atau parameter Aterkia. Dataset dan bobot belum dinilai layak pakai.

### 6. Bukti passage terhadap geometri gate — kandidat setelah estimasi pose tervalidasi

**Bukti upstream.** Controller gate/qualification membandingkan posisi robot
dengan posisi gate beserta exit margin; pergantian heading juga memakai
orientasi. Sumber:
[gate](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/tasks/src/controllers/gate.cpp),
[qualification](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/tasks/src/controllers/qual_gate.cpp).

**Usulan.** Prinsip membuktikan posisi sebelum/sesudah gate relevan untuk
`wayfinder`. Jika kelak pose gate dan kendaraan memiliki ketelitian yang
terukur, gunakan crossing terhadap bidang gate dan batas koridor yang sesuai
arah gate, lalu gabungkan dengan bukti visual. Jangan bergantung pada sumbu X
arena yang di-hardcode.

**Batas.** Aterkia saat ini membuktikan passage hanya terhadap ground truth mock.
Depth belum membuktikan perpindahan horizontal. Timeout dan hilangnya gate dari
gambar tetap bukan bukti sukses. Jangan mengganti mock odometry dengan estimasi
PWM lalu menganggap acceptance hardware sudah terpenuhi.

### 7. Mengurangi lonjakan command — evaluasi setelah karakteristik kendaraan diketahui

**Bukti upstream.** Task runner membatasi yaw, mengurangi forward command pada
belokan besar, dan melakukan smoothing perubahan RC. Sumber:
[task_runner.cpp](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/tasks/src/task_runner.cpp).

**Usulan.** Bila bench/pool menunjukkan overshoot akibat perubahan setpoint,
evaluasi pembatas perubahan command di `helm` serta hubungan surge–yaw pada
guidance `wayfinder`. Jangan menduplikasi stabilisasi autopilot atau langsung
mengadopsi PD/PWM upstream. Failsafe zero/neutral yang ditetapkan harus melewati
smoothing supaya penghentian tidak tertunda. Nilai gain/rate memerlukan uji
kendaraan sendiri.

## Hal yang tidak diadopsi dan alasannya

| Bagian | Temuan pada sumber | Implikasi untuk Aterkia |
| --- | --- | --- |
| Alur `brain → cv → tasks` | Persepsi meneruskan perintah tugas; task runner mengirim RC override. | PRD §5 memisahkan misi, persepsi, guidance dan hardware. Pertahankan alur Action `orchestrator → wayfinder → helm`. |
| Odometri dari PWM | Memakai `/mavros/rc/out`, IMU, massa/drag/koefisien thrust; ada pembalikan tanda ESC khusus dan kecepatan linear dibatasi tidak negatif. | Tidak portabel ke susunan thruster Aterkia. Kalibrasi dan ukur error terhadap referensi independen sebelum dipakai. |
| Bearing-only EKF SLAM | Inisialisasi memakai prior posisi landmark dan constraint lebar gate/jarak bucket, selain odometri dan bearing. | Bukan solusi localization tanpa asumsi geometri/gerak. SLAM masih di luar MVP; jangan memindahkan arena/priors mereka. |
| Timeout gripper | `execute()` mengembalikan `kTaskComplete` ketika timestamp melewati timeout setelah observasi tersedia. | Bertentangan dengan FR-09 jika dianggap sukses. Di Aterkia timeout harus gagal; drop/pickup butuh bukti hasil aktuator. |
| Payload dalam callback kontrol | Task runner melakukan servo command dan sleep pada jalur penyelesaian gripper. | Bila payload dibuat, gunakan langkah yang tidak memblokir abort/watchdog, dengan deadline dan acknowledgement yang jelas. |
| Lookup TF target | Sebagian lookup gate/flag berada di luar guard awal dan mengasumsikan frame tersedia. | Jika TF ditambahkan, periksa semua transform yang dibutuhkan, timestamp/age dan exception; frame hilang bukan alasan bergerak buta. |
| Parameter/auto-arm upstream | Ada urutan mode, depth, delay dan arming spesifik kendaraan. | Preflight dan otorisasi eksplisit kita dipertahankan. Konfigurasi frame, compass, versi BlueOS/ArduSub dan depth API perlu verifikasi tersendiri. |
| CI Humble | Workflow memakai Humble dan mengabaikan exit code `colcon test-result` dengan `|| true`. | Pipeline kita kelak harus cocok dengan baseline ROS yang dipilih dan gagal bila hasil tes gagal; jangan menyalin workflow apa adanya. |

Sumber rinci untuk batas odometri:
[node](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/ekfslam/src/odometry_node.cpp),
[model](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/ekfslam/src/utils/odometry.cpp).
Untuk prior/constraint:
[inisialisasi EKF](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/ekfslam/src/ekfslam.cpp#L43-L75),
[YAML](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/manta_bringup/launch/params.yaml#L154-L208).
Untuk timeout dan blocking:
[gripper](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/tasks/src/controllers/gripper.cpp#L16-L28),
[task runner](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/src/tasks/src/task_runner.cpp).
Untuk pipeline CI:
[build.yml](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/.github/workflows/build.yml).

## Kapan package baru masuk akal?

| Kandidat masa depan | Syarat konkret sebelum ditambahkan | Batas kepemilikan |
| --- | --- | --- |
| `state_estimation` (nama usulan) | Ada sumber motion/pose nyata, kebutuhan beberapa konsumen dan bukti bahwa guidance sederhana tidak cukup; revisi scope FR-15 disepakati. | Estimasi pose/ketidakpastian/TF; tidak memilih misi atau mengirim command kendaraan. Evaluasi komponen ROS yang tersedia sebelum membuat EKF sendiri. |
| Driver payload terpisah | Hardware sudah dipilih dan drivernya cukup mandiri/reusable untuk memiliki lifecycle dan tes sendiri. | Tetap diakses melalui `helm` sebagai pemilik otorisasi aktuator. Untuk satu mekanisme sederhana, adapter di `helm` sudah cukup. |
| Driver kamera eksternal | Kamera memerlukan driver ROS dan penyedia `CameraInfo`. | Utamakan driver yang tersedia; capture tidak dipindahkan ke `spectra` hanya karena upstream menggabungkannya. Tidak harus menjadi package buatan tim. |

Tidak diusulkan package `tasks`, SLAM, recorder, atau GUI baru untuk MVP sekarang.
Tool tuning, replay dan kalibrasi cukup ditempatkan di `scripts/` atau pengujian.

## Asal aset dan lisensi yang perlu diperiksa sebelum reuse

[LICENSE repo](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/LICENSE)
memuat Apache-2.0, sedangkan
[metadata model generalist](https://github.com/kushagra77/OpenMantaClaus/blob/f05f646aade941b8c35b3bb166d9c236fb181daf/scripts/yolo/runs/detect/pretrain_generalist_v2/weights/best_saved_model/metadata.yaml#L1-L5)
mencantumkan AGPL-3.0. Maka lisensi repo tidak boleh diasumsikan otomatis sama
untuk setiap bobot, dataset, atau dependency. Asal/izin dataset eksternal dan
ketentuan pemakaian model belum diverifikasi. Catatan ini tidak menyalin kode
atau memasukkan aset tersebut ke Aterkia.

## Urutan tindak lanjut yang disarankan

1. Kumpulkan rekaman kamera/housing dan telemetry Pixhawk yang benar-benar tersedia;
   definisikan kebutuhan kalibrasi serta format bukti bench.
2. Rancang adapter `helm` dan skenario bench `bootstrap` berdasarkan mode kendali
   ArduSub yang dipilih, lalu buktikan jalur telemetry/stop sebelum misi.
3. Evaluasi detektor yang ada pada rekaman berlabel; tambahkan filtering/ROI atau
   eksperimen learned detector hanya jika hasil ukur membenarkannya.
4. Putuskan sumber bukti passage hardware. Estimasi keadaan/pemetaan baru
   dipertimbangkan setelah kebutuhan dan ketidakpastiannya terukur.
5. Setelah hardware payload dan aturan final ditetapkan, definisikan bukti sukses
   drop/pickup sebelum implementasi M4 dilanjutkan.

Daftar ini bukan jadwal atau izin untuk mulai menulis kode. Keputusan implementasi
berikutnya tetap memerlukan cakupan yang sesuai PRD dan pengujian milestone.
