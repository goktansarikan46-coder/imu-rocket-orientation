# Roket-imu

STM32 + MPU-6050 ile roketin **roll** ve **pitch** açısını gerçek zamanlı hesaplayan
küçük bir öğrenci projesi. Açılar seri porttan CSV olarak PC'ye gönderiliyor, MATLAB
tarafında 3B roket modeli ve canlı grafiklerle çiziliyor.

Amaç mümkün olduğunca sade kod yazmak: filtre 20 satır, ana döngü tek sayfa.
Ama sadelik uğruna kararlılıktan ödün verilmedi — sabit örnekleme periyodu,
jiroskop bias kalibrasyonu ve ivmeölçer güvenilirlik kontrolü var.

## İçerik

```
roket-imu/
├── firmware/
│   └── main.c          STM32 tarafı (CubeMX projesine doğrudan girer)
├── matlab/
│   ├── roket_3b.m      3B roket + canlı roll/pitch grafikleri
│   └── csv_kaydet.m    Gelen veriyi dosyaya kaydeder (offline analiz için)
├── .gitignore
└── LICENSE
```

## Donanım

| Parça | Not |
|---|---|
| STM32 kartı | Projede STM32C092RC kullanıldı, HAL kullanan her STM32'de çalışır |
| MPU-6050 | I2C, 3 eksen ivmeölçer + jiroskop |
| USB-TTL / ST-Link VCP | UART üzerinden PC'ye veri |

Bağlantı: MPU-6050'nin **SCL/SDA** pinleri kartın I2C hattına, **VCC 3.3 V**, **GND ortak**.
Hangi pinlerin kullanıldığı `.ioc` dosyasında görünür (bu projede `I2C2` ve `USART2`).
I2C hattında dahili pull-up yoksa 4.7 kΩ direnç ekleyin.

## Kurulum

1. STM32CubeMX/CubeIDE'de yeni proje aç, **I2C2** ve **USART2**'yi (115200 baud) etkinleştir.
2. Üretilen `Core/Src/main.c` dosyasının yerine `firmware/main.c` dosyasını koy.
   (Kod tamamen `USER CODE` bloklarının içinde, CubeMX yeniden kod ürettiğinde silinmez.)
3. Derle, karta yükle.
4. MATLAB'de `matlab/roket_3b.m` içindeki `PORT` değişkenini kendi portunla değiştir
   (`serialportlist("available")` ile bulabilirsin) ve çalıştır.

## Seri port veri formatı

Kart saniyede 100 satır gönderir, her satır virgülle ayrılmış 9 tamsayı:

```
t_ms,ax,ay,az,gx,gy,gz,roll100,pitch100
1250,-120,340,16380,-5,12,3,-1543,872
```

| Alan | Anlamı |
|---|---|
| `t_ms` | Kartın açılışından beri geçen süre (ms) — zamanlama analizi için |
| `ax..gz` | Sensörden okunan **ham** değerler (LSB) |
| `roll100`, `pitch100` | Açı × 100 (derece). Yani `-1543` → `-15.43°` |

Açılar 100 ile çarpılıp tamsayı olarak gönderiliyor çünkü `sprintf` ile float basmak
küçük MCU'larda hem yavaş hem de ek kütüphane gerektiriyor. Ham veriler de gönderiliyor
ki aynı kayıt üzerinde farklı filtreler denenebilsin.

## Nasıl çalışıyor?

### 1. İki sensör, iki farklı zaaf

| Sensör | İyi tarafı | Kötü tarafı |
|---|---|---|
| İvmeölçer | Yer çekimini gördüğü için **mutlak** referans, kaymaz | Gürültülü, her titreşimden etkilenir |
| Jiroskop | Kısa vadede çok temiz ve hızlı | İntegral aldığımız için hatası **birikir** (drift) |

Tamamlayıcı filtre bu ikisini birleştirir: açıyı jiroskopla ilerletir, ivmeölçeri de
yavaşça "doğru yöne çeken" bir düzeltme olarak kullanır.

```c
aci += gyro_dps * DT;                          /* hizli ama kayan tahmin */
aci += (1 - ALFA) * (acc_aci - aci);           /* yavas ama kaymayan duzeltme */
```

`ALFA = 0.98` → düzeltmenin ağırlığı %2. Zaman sabiti yaklaşık
`DT * ALFA / (1 - ALFA) = 0.01 * 49 = 0.49 s`.
Yani ivmeölçer, yarım saniyeden uzun süreli hataları temizler; ondan kısa süreli
sarsıntıları ise jiroskop bastırır.

### 2. Sabit `dt` neden önemli?

`dt`'yi `HAL_GetTick()` farkıyla ölçmek ilk bakışta daha doğru görünür, ama ölçülen süreye
UART gönderiminin ve işlem süresinin gecikmesi de karışır. Jiroskopta bu doğrudan açı hatası
olarak birikir. Bunun yerine döngü **sabit 10 ms**'de çalıştırılıyor ve `dt` sabit `0.01f`
alınıyor:

```c
while ((int32_t)(HAL_GetTick() - sonraki) < 0) { }   /* periyodun basini bekle */
sonraki += ORNEK_PERIYOT;
```

`HAL_Delay(10)` yerine bunun kullanılmasının sebebi: `HAL_Delay` işlem süresinin **üstüne**
10 ms ekler, bu yöntemse periyodun başlangıcını sabitler.

### 3. Jiroskop bias kalibrasyonu

Jiroskop dururken bile sıfır göstermez, küçük bir ofseti vardır. Açılış anında kart 2 saniye
sabit tutulur, 200 örneğin ortalaması alınır ve bu değer her okumadan çıkarılır.
Bu ofset sıcaklıkla değiştiği için her açılışta yeniden ölçülür — sabit bir sayı olarak
koda gömülemez.

### 4. İvmeölçer güven kontrolü (basit ama kritik)

Roket motor ateşlerken ivmeölçer artık yer çekimini değil, itkiyi ölçer — o an ondan gelen
açı tamamen yanlıştır. Bu yüzden vektörün büyüklüğü 1 g'ye yakın değilse düzeltme adımı
atlanır ve o süre boyunca sadece jiroskopa güvenilir:

```c
if (buyukluk > 0.70f && buyukluk < 1.30f)  /* sadece bu aralikta duzelt */
```

Üç satırlık bu kontrol, yüksek ivmeli anlarda açının bozulmasını engeller.

## Bilinen sınırlamalar

- **Yaw (sapma) yok.** Tek başına IMU ile yaw'ı sabitlemek mümkün değil, manyetometre gerekir.
- **±90° pitch'te gimbal lock.** Euler açıları kullanıldığı için burun tam yukarı bakarken
  roll tanımsızlaşır. Kalıcı çözüm kuaterniyon tabanlı filtre.
- **UART bloklayıcı.** Her satır ~4 ms sürüyor, yani 10 ms'lik bütçenin epey bir kısmını
  yiyor. Veri hızını artırmak istersen kesme/DMA ile göndermek gerekir.
- **Sensör okuma yoklamalı (polling).** MPU-6050'nin INT pini kullanılmıyor.

## Yol haritası

- [ ] Kalman filtresi (jiroskop bias'ını çalışma anında tahmin eder)
- [ ] Kuaterniyon / Mahony filtresi — gimbal lock çözümü
- [ ] MPU INT pini + kesme ile örnekleme
- [ ] UART'ın DMA'ya taşınması
- [ ] SD karta uçuş kaydı

## Lisans

MIT — bkz. [LICENSE](LICENSE).
