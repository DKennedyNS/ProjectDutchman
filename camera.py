from picamera import PiCamera
from time import sleep

numPhotos = 10
camera = PiCamera()

camera.start_preview()
sleep(5)
for i in range(10):
  camera.capture('/home/pi/Desktop/image%s.jpg' % i)
  print(i)
  sleep(5)

camera.stop_preview()

camera.rotation = 180
camera.start_preview(alpha=200)

