# **Arbak Savunma Ve Enerji Teknolojileri A.Ş.**

## Mayın Tarama Quadcopter:

Mayın tarama ve tespit amaçlı, uzaktan kumandalı veya otonom keşif amaçları için kullanılacak bir quadcopterdir. Prototipler üzerinde çalışılmaktadır. Proje testleri için Arduino Mega geliştirme kartı,
çeşitli sensörler(ultrasonik mesafe sensörleri, GPS, IMU, sıcaklık) batarya gibi komponentler kullanılmıştır.

### Kodların Açıklaması
#### ESP8266_IMU kodu:
Bu kod MPU6050 IMU sensöründen ESP8266 tarafından okumak için hazırda bir kütüphane ile (MPU6050_light.cpp) kullanılmaktadır.

#### IMU kodu:
Bu kod MPU6050 IMU sensöründen Arduino tarafından okumak için hazırda bir kütüphane ile (MPU6050_6Axis_MotionApps20.cpp) kullanılmaktadır.

#### lipoBattery_thermalcontroll kodu:
Bu kod Batarya kısmının aşırı ısınması veya basşka sorunlarına binayen bir arduino nano kartı ve lm35 sıcaklık sensörleri ile kontrolünü sağlıcak bir batarya koruma sistemi kurulmaya çalışıldı.

#### quadcopter_multi_driver_esp01 kodu:
Bu kod quadcopterdeki bazı parametreler ile (PID kontrolündeki error) api.thingspeak.com da bir grafik oluşturulması için Arduino Mega ile ESP8266 01 Wifi modülünün kullanımına yönelik yazılmıştır. 
Ar-Ge amaçlıdır başarılı olmamıştır.

#### quadcopter_multi_driver_esp8266 kodu:
Bu kod quadcopteri esp8266 kartı ile Arduino Mega daki gibi çalıştırma amacıyla yazıldı. Başarılı olmadı.

#### quadcopter_multi_driver_model... kodları:
Bu kod quadcopteri Arduino Mega, MPU6050, ultrasonik mesafe sensörleri ve uzaktan kumanda alıcı vericisi ile kullanımı için yazılmış bir kod versiyonlarıdır. 

