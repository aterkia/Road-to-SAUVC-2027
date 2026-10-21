# Kalibrasi kamera dan guidance bearing

Adaptasi gagasan 1 dari [riset](research/openmantaclaus-review.md). Implementasi
monocular `plumb_bob` menggunakan OpenCV yang sudah menjadi dependency `spectra`.
Tidak ada parameter optik kendaraan referensi yang digunakan.

## Menghasilkan kalibrasi

Gunakan kamera, housing, fokus, resolusi, dan medium yang akan dipakai. Ambil
setidaknya 10 gambar chessboard dengan variasi kemiringan, jarak dan posisi
yang mencakup tepi gambar. `columns`/`rows` menghitung **inner corners**;
ukuran kotak dalam meter. Dari workspace dengan dependency ROS tersumber:

```bash
python3 scripts/calibrate_camera.py artifacts/calibration/*.png \
  --columns 7 --rows 6 --square-size 0.03 --camera-name front_underwater \
  --output artifacts/front_camera.yaml
```

Output adalah YAML format kalibrasi ROS (`K`, `D`, `R`, `P` sebagai
`camera_matrix`, `distortion_coefficients`, `rectification_matrix`,
`projection_matrix`), ditambah RMS reprojection keseluruhan/per gambar dan
daftar gambar tanpa chessboard. File lama tidak ditimpa. Resolusi campuran,
gambar tidak terbaca, dan kurang dari 10 view yang berhasil ditolak.
Tidak ada eliminasi outlier otomatis atau klaim bahwa RMS kecil cukup untuk
menyatakan kalibrasi akurat; periksa error tiap view dan bearing pada gambar
terpisah yang tidak dipakai untuk fitting. Kalibrasi nyata belum tersedia.

Metode mengacu pada [panduan kalibrasi OpenCV](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
dan [kontrak CameraInfo ROS](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/CameraInfo.msg).

## Mengaktifkan

Salin konfigurasi stack yang ada, lalu ubah parameter berikut:

```yaml
spectra:
  ros__parameters:
    image_width: 640
    image_height: 480
    camera_frame: camera_optical_frame
    input_geometry: raw
    calibration_file: /absolute/path/front_camera.yaml
wayfinder:
  ros__parameters:
    image_width: 640.0
    image_height: 480.0
    camera_frame: camera_optical_frame
    use_camera_info: true
    bearing_tolerance: 0.08
    bearing_gain: 1.0
```

`raw` mengoreksi gambar melalui map yang dihitung sekali saat startup. Jika
driver sudah menghasilkan gambar rectified memakai **P yang sama**, gunakan
`rectified` dan remap input `/camera/image_raw` ke topik rectified driver.
Dalam mode tersebut tidak ada koreksi ulang. Jangan melakukan resize/crop
sesudah kalibrasi; implementasi sekarang hanya menerima full resolution,
monocular dengan R identity dan tanpa baseline stereo. Fisheye/equidistant
ditolak, bukan diperlakukan sebagai pinhole.

`spectra` tetap menerbitkan `Detection2DArray`. Koordinat bounding box pada mode
kalibrasi adalah koordinat gambar rectified, bukan gambar raw. Untuk setiap
hasil, `/perception/camera_info` membawa header akuisisi yang sama dan geometri
hasil (K = bagian kiri P, D = 0). Timestamp sumber tidak diganti waktu proses.
Tidak ada kamera driver atau node baru untuk tahap ini.

`wayfinder` memasangkan kedua topik dengan timestamp **persis sama**, buffer
maksimal 5 pasangan; QoS sensor best effort. Pasangan harus cocok frame,
resolusi dan model pinhole. Pasangan lama/invalid tidak memperbarui freshness.
Kehilangan pasangan saat PASS_GATE menyebabkan timeout kamera dan command nol,
seperti hilangnya kamera. Empty detections hanya menjadi bukti clear view
bila pasangan kalibrasinya valid.

Sudut horizontal kanan positif adalah `atan2(u - cx, fx)` radian menggunakan P,
lalu yaw FLU bernilai negatif untuk target di kanan. `bearing_gain` dalam
(rad/s)/rad dan `bearing_tolerance` dalam radian; batas yaw tetap berlaku.
Kamera diasumsikan terpasang menghadap depan dan selaras body. Extrinsics yang
berbeda belum didukung. Lebar gate tetap rasio piksel untuk heuristic approach;
bearing tidak menghasilkan estimasi jarak atau bukti passage hardware.

Default mock tetap `input_geometry: uncalibrated`, file kosong,
`use_camera_info: false`, dengan error piksel yang dinormalisasi dan
`align_tolerance` lama. Ini mode fixture, bukan fallback otomatis jika file
kalibrasi gagal. Saat memilih mode calibrated, file yang hilang/invalid
menggagalkan startup; saat `use_camera_info: true` input tanpa pasangan ditolak.

## Verifikasi

`tests/test_calibration.py` memeriksa recovery intrinsics dari proyeksi yang
diketahui, RMS, format/ukuran invalid, undistorsi sekali, principal point tidak
di tengah, tanda yaw kiri/kanan dan kehilangan CameraInfo. Jalankan:

```bash
source scripts/ros_env.sh
ROS_DOMAIN_ID=77 python3 -m pytest tests/test_calibration.py -v
```

Tes fixture ini tidak menggantikan kalibrasi kamera/housing di air dan
pengukuran bearing pada perangkat sebenarnya.
