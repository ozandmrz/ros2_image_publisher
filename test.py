from picamera2 import Picamera2
import time

# Picamera2 nesnesini başlat
picam2 = Picamera2()

# Kamera yapılandırmasını seç ve başlat
picam2.configure(picam2.create_still_configuration())

# Kamerayı başlat
picam2.start()

# Görüntünün netleşmesi için kısa bir bekleme süresi
time.sleep(2)

# Görüntüyü yakala ve PNG olarak kaydet
image = picam2.capture_file("image.png")

# Kamerayı durdur
picam2.stop()

print("Görüntü image.png olarak kaydedildi.")
