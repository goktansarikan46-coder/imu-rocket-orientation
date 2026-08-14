"""
==========================================================================
 3D ROCKET VISUALIZATION
 STM32 + MPU-6050  ->  seri port  ->  Python
==========================================================================

Karttan gelen satir bicimi (CSV, saniyede ~100 satir):

    ax,ay,az,gx,gy,gz,roll100,pitch100

roll100 / pitch100 = aci * 100    (firmware tamsayi gonderdigi icin 100'e boluyoruz)


VERI AKISI
----------
    [Seri port]
         |   arka planda calisan okuma thread'i        (~100 Hz)
         v
    son_veri   <- her zaman "en guncel satir" tutulur
         |   ekran zamanlayicisi (QTimer)              (~60 FPS)
         v
    [Yumusatma] -> [3B roket] + [roll/pitch grafikleri]

Neden iki ayri hiz?
    Sensor 100 Hz uretiyor, ekran 60 FPS ciziyor. Okumayi ayri bir thread'e
    alirsak, cizim yavaslasa bile satirlar seri tamponda birikip gecikme
    yapmaz; ekran her zaman en son gelen degeri kullanir.


Kurulum   : pip install pyserial pyqtgraph PyQt5 numpy
Calistirma: python roket_3b_gorsel.py
"""

import sys
import math
import time
import threading

import numpy as np
import serial
import serial.tools.list_ports

import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtWidgets


# ==========================================================================
# 1) AYARLAR   -  once buraya bak
# ==========================================================================

PORT    = "COM3"      # None -> portu otomatik bul. Elle yazmak icin: "COM3" / "/dev/ttyACM0"
BAUD    = 115200
TAU     = 0.05      # yumusatma zaman sabiti [s]. Titrekse buyut, gecikirse kucult.
RSIGN   = +1        # roll  ters donuyorsa -1 yap
PSIGN   = +1        # pitch ters egiliyorsa -1 yap
ORNEK   = 400       # grafikte gorunen ornek sayisi (~4 saniye)
LOG_CSV = "deneme_kayıt.csv"      # Kayit icin dosya adi yaz: LOG_CSV = "kayit.csv"
                    
# Roket olculeri (govde ekseni +Z, merkez ~ orijin)
GOVDE_R  = 0.25
TABAN_Z  = -2.40
OMUZ_Z   = +1.30    # burun konisinin basladigi yer
BURUN_Z  = +2.60

# Renkler: 0..1 arasi (r, g, b)
BEYAZ       = (0.92, 0.93, 0.96)
KIRMIZI     = (0.90, 0.25, 0.25)
GRI         = (0.55, 0.55, 0.60)
MAVI        = (0.20, 0.52, 0.95)
ALEV_DIS    = (1.00, 0.42, 0.08)
ALEV_IC     = (1.00, 0.88, 0.45)


# ==========================================================================
# 2) SERI PORT OKUYUCU  (arka plan thread'i)
# ==========================================================================

class SeriOkuyucu(threading.Thread):
    """Seri porttan surekli satir okur, en son gecerli satiri saklar."""

    def __init__(self, port, baud, log_dosyasi=None):
        super().__init__(daemon=True)
        self.ser   = serial.Serial(port, baud, timeout=0.2)
        self.kilit = threading.Lock()   # iki thread ayni degiskene dokunuyor -> kilit sart
        self.son_veri     = None        # [ax, ay, az, gx, gy, gz, roll100, pitch100]
        self.satir_sayisi = 0
        self.calisiyor    = True
        self.log = None
        if log_dosyasi:
            # buffering=1 -> her satir aninda diske yazilir, program beklenmedik
            # sekilde kapansa bile o ana kadarki veri dosyada durur.
            self.log = open(log_dosyasi, "w", buffering=1)
            self.log.write("ax,ay,az,gx,gy,gz,roll100,pitch100\n")

    def run(self):
        """Thread basladiginda otomatik calisir. Ekran ciziminden bagimsizdir."""
        self.ser.reset_input_buffer()
        while self.calisiyor:
            try:
                ham = self.ser.readline()
            except Exception:
                break
            veri = self._satiri_coz(ham)
            if veri is None:
                continue
            with self.kilit:
                self.son_veri = veri
                self.satir_sayisi += 1

            # Dosyaya yazma kilidin DISINDA yapilir; disk islemi yavastir,
            # kilidi elde tutarken beklemek diger thread'i bosuna durdurur.
            if self.log:
                self.log.write(",".join(str(x) for x in veri) + "\n")

    @staticmethod
    def _satiri_coz(ham_bayt):
        """Bir satiri 8 tamsayiya cevirir. Bozuk/baslik satirlari icin None doner."""
        metin = ham_bayt.decode("ascii", "ignore").strip()
        parca = metin.split(",")
        if len(parca) != 8:
            return None                 # "WHO_AM_I=104" gibi satirlar burada elenir
        try:
            return [int(p) for p in parca]
        except ValueError:
            return None                 # "ax,ay,az,..." baslik satiri burada elenir

    def oku(self):
        """Ekran tarafi buradan okur: (son_veri, toplam_satir)."""
        with self.kilit:
            return self.son_veri, self.satir_sayisi

    def durdur(self):
        self.calisiyor = False
        time.sleep(0.25)
        try:
            self.ser.close()
        except Exception:
            pass
        if self.log:
            self.log.close()


# ==========================================================================
# 3) KUCUK YARDIMCILAR
# ==========================================================================

class Yumusatici:
    """
    Ustel yumusatma (low-pass):   deger += k * (hedef - deger)

    k sabit degil, gecen kare suresine gore hesaplanir: k = 1 - exp(-dt/tau)
    Boylece bilgisayar yavaslasa bile yumusatma hizi ayni kalir.

    aci_sarmali=True ise -180/+180 gecisi dogru yonden yapilir
    (179 -> -179 arasi 2 derecedir, 358 derece degil).
    """

    def __init__(self, tau, aci_sarmali=False):
        self.tau = tau
        self.aci_sarmali = aci_sarmali
        self.deger = 0.0

    def guncelle(self, hedef, dt):
        k = 1.0 - math.exp(-dt / self.tau)
        fark = hedef - self.deger
        if self.aci_sarmali:
            fark = (fark + 180.0) % 360.0 - 180.0
        self.deger += k * fark
        return self.deger


class Tampon:
    """Grafik icin kayan pencere: yeni ornek sagdan girer, en eski soldan cikar."""

    def __init__(self, n):
        self.veri = np.zeros(n)

    def ekle(self, x):
        self.veri[:-1] = self.veri[1:]   # yerinde kaydirma (yeni dizi olusturmaz)
        self.veri[-1] = x


def ivmeden_aci(ax, ay, az):
    """
    Sadece ivmeolcerden roll/pitch. Yercekimi vektorunun yonunden bulunur.
    Firmware'deki formulun aynisi; burada filtreli sonucla karsilastirmak icin var.
    """
    fax, fay, faz = ax / 16384.0, ay / 16384.0, az / 16384.0
    roll  = math.degrees(math.atan2(fay, faz))
    pitch = math.degrees(math.atan2(-fax, math.sqrt(fay * fay + faz * faz)))
    return roll, pitch


# ==========================================================================
# 4) 3B ROKET MODELI
# ==========================================================================

class Roket:
    """
    Roketin butun parcalarini tutar ve hepsini ayni donme matrisiyle dondurur.

    Her parca (item, temel_donusum) ikilisi olarak saklanir:
        temel_donusum = parcanin roket govdesi icindeki sabit yeri
        donme         = roketin o anki durusu
        son matris    = donme * temel_donusum

    Alev ayri tutulur, cunku boyu her karede degisir (titreme efekti).
    """

    def __init__(self, gorunum):
        self.gorunum  = gorunum
        self.parcalar = []      # (item, sabit temel donusum)
        self.alevler  = []      # (item, temel boy)

        self._govde_yap()
        self._kanatcik_yap()
        self._alev_yap()

    # ---- kurulum -------------------------------------------------------
    def _ekle(self, item, temel=None):
        self.parcalar.append((item, temel if temel is not None else pg.Transform3D()))
        self.gorunum.addItem(item)

    def _govde_yap(self):
        self._ekle(*self._silindir(GOVDE_R, GOVDE_R, OMUZ_Z - TABAN_Z, BEYAZ,   TABAN_Z))
        self._ekle(*self._silindir(GOVDE_R, 0.0,     BURUN_Z - OMUZ_Z, KIRMIZI, OMUZ_Z))
        self._ekle(*self._silindir(0.0,     GOVDE_R, 0.20,             GRI,     TABAN_Z))

    def _kanatcik_yap(self):
        for aci in (0, 120, 240):
            md = self._kanatcik_mesh(aci, r_kok=GOVDE_R, r_uc=0.80,
                                     z=TABAN_Z, h_kok=1.00, h_uc=0.45)
            self._ekle(gl.GLMeshItem(meshdata=md, smooth=False, shader="shaded",
                                     color=MAVI + (1.0,)))

    def _alev_yap(self):
        """
        Ic ice iki koni: genis turuncu dis alev + ince sari ic cekirdek.
        'additive' harmanlama ust uste binen yerleri parlatir, akkor gorunumu verir.
        """
        katmanlar = [
            # yaricap,        boy,  renk,     saydamlik
            (GOVDE_R * 0.95,  1.70, ALEV_DIS, 0.55),
            (GOVDE_R * 0.50,  0.95, ALEV_IC,  0.90),
        ]
        for yaricap, boy, renk, alfa in katmanlar:
            md = gl.MeshData.cylinder(rows=1, cols=20, radius=[yaricap, 0.0], length=1.0)
            item = gl.GLMeshItem(meshdata=md, smooth=True, shader="balloon",
                                 color=renk + (alfa,), glOptions="additive")
            self.gorunum.addItem(item)
            self.alevler.append((item, boy))

    # ---- geometri yardimcilari -----------------------------------------
    @staticmethod
    def _silindir(r_alt, r_ust, boy, renk, z_taban):
        """Silindir / koni uretir (r_ust=0 -> koni). Govde ekseni +Z."""
        md = gl.MeshData.cylinder(rows=1, cols=24, radius=[r_alt, r_ust], length=boy)
        item = gl.GLMeshItem(meshdata=md, smooth=True, shader="shaded", color=renk + (1.0,))
        temel = pg.Transform3D()
        temel.translate(0, 0, z_taban)
        return item, temel

    @staticmethod
    def _kanatcik_mesh(aci_derece, r_kok, r_uc, z, h_kok, h_uc):
        """Tek kanatcik: govdeden disari bakan yamuk, 2 ucgen."""
        a = math.radians(aci_derece)
        ux, uy = math.cos(a), math.sin(a)
        profil = [(r_kok, z), (r_kok, z + h_kok), (r_uc, z + h_uc), (r_uc, z)]
        kose = np.array([[r * ux, r * uy, zz] for (r, zz) in profil], dtype=float)
        yuz  = np.array([[0, 1, 2], [0, 2, 3]])
        return gl.MeshData(vertexes=kose, faces=yuz)

    @staticmethod
    def _alev_temel(boy):
        """
        Koni yukari bakar (0..1 arasi). Once boyuna olceklenir, sonra 180 derece
        cevrilip asagi bakar, en son kuyruga tasinir.
        (Matris carpimi sagdan sola okunur: T * R * S)
        """
        temel = pg.Transform3D()
        temel.translate(0, 0, TABAN_Z)
        temel.rotate(180, 1, 0, 0)
        temel.scale(1, 1, boy)
        return temel

    @staticmethod
    def _titreme(t):
        """Farkli hizda iki sinusun toplami -> tekrar etmiyormus gibi duran dogal titreme."""
        return 1.0 + 0.14 * math.sin(t * 23.0) + 0.07 * math.sin(t * 41.0)

    # ---- her karede cagrilir -------------------------------------------
    def yonlendir(self, roll_derece, pitch_derece, zaman):
        """
        Donme matrisi = Ry(pitch) * Rx(roll)
        (MATLAB kodundaki sira ile ayni: once roll, sonra pitch uygulanir.)
        """
        donme = pg.Transform3D()
        donme.rotate(pitch_derece, 0, 1, 0)
        donme.rotate(roll_derece,  1, 0, 0)

        for item, temel in self.parcalar:
            item.setTransform(donme * temel)

        k = self._titreme(zaman)
        for item, boy in self.alevler:
            item.setTransform(donme * self._alev_temel(boy * k))


# ==========================================================================
# 5) ACI GRAFIGI  (filtreli + ham ivmeolcer)
# ==========================================================================

class AciGrafigi:
    """Tek bir aci icin kayan grafik. Renkli cizgi = filtreli, gri = ham ivmeolcer."""

    def __init__(self, baslik, renk_255, y_sinir, n):
        self.widget = pg.PlotWidget(title=baslik)
        self.widget.setBackground("k")
        self.widget.setYRange(-y_sinir, y_sinir)
        self.widget.setXRange(0, n)
        self.widget.showGrid(x=True, y=True, alpha=0.25)
        self.widget.setMouseEnabled(x=False, y=False)
        self.widget.getPlotItem().hideAxis("bottom")
        self.widget.getPlotItem().titleLabel.setText(
            baslik, color=pg.mkColor(renk_255), size="11pt")

        self.cizgi_ham     = self.widget.plot(pen=pg.mkPen((130, 135, 145, 140), width=1))
        self.cizgi_filtre  = self.widget.plot(pen=pg.mkPen(renk_255, width=2))
        self.tampon_ham    = Tampon(n)
        self.tampon_filtre = Tampon(n)

    def guncelle(self, filtreli, ham):
        self.tampon_filtre.ekle(filtreli)
        self.tampon_ham.ekle(ham)
        self.cizgi_filtre.setData(self.tampon_filtre.veri)
        self.cizgi_ham.setData(self.tampon_ham.veri)


# ==========================================================================
# 6) ANA PENCERE
# ==========================================================================

class Pencere(QtWidgets.QWidget):

    def __init__(self, okuyucu):
        super().__init__()
        self.okuyucu = okuyucu

        self.setWindowTitle("Roket - Attitude")
        self.resize(1280, 760)
        self.setStyleSheet("background-color:#000;")

        self._sahne_kur()
        self._panel_kur()
        self._yerlesim_kur()

        # Yumusaticilar: roll acisi sarmalidir (-180/+180), pitch degildir.
        self.roll_suzgec  = Yumusatici(TAU, aci_sarmali=True)
        self.pitch_suzgec = Yumusatici(TAU, aci_sarmali=False)

        self.roll_hedef = self.pitch_hedef = 0.0
        self.roll_ham   = self.pitch_ham   = 0.0

        self.t_onceki    = time.perf_counter()
        self.t_baslangic = time.perf_counter()
        self.kare = 0

        self.zamanlayici = QtCore.QTimer()
        self.zamanlayici.timeout.connect(self.guncelle)
        self.zamanlayici.start(16)          # 16 ms -> ~60 FPS

    # ---- arayuz kurulumu ------------------------------------------------
    def _sahne_kur(self):
        self.gorunum = gl.GLViewWidget()
        self.gorunum.setBackgroundColor("k")
        self.gorunum.opts["distance"]  = 13
        self.gorunum.opts["elevation"] = 16
        self.gorunum.opts["azimuth"]   = -40

        zemin = gl.GLGridItem()
        zemin.setSize(12, 12)
        zemin.setSpacing(1, 1)
        zemin.translate(0, 0, -4.5)
        self.gorunum.addItem(zemin)

        self.roket = Roket(self.gorunum)

    def _panel_kur(self):
        self.sayi_etiketi = QtWidgets.QLabel("ROLL   ---\nPITCH  ---")
        self.sayi_etiketi.setStyleSheet(
            "color:#f2f4f8; font-family:Consolas,monospace; font-size:26px;")

        pg.setConfigOptions(antialias=True)
        self.grafik_roll  = AciGrafigi("ROLL",  (60, 205, 255), 180, ORNEK)
        self.grafik_pitch = AciGrafigi("PITCH", (255, 155, 50),  90, ORNEK)

        self.durum_etiketi = QtWidgets.QLabel("")
        self.durum_etiketi.setStyleSheet(
            "color:#737d8c; font-family:Consolas,monospace; font-size:12px;")

    def _yerlesim_kur(self):
        sol = QtWidgets.QVBoxLayout()
        sol.addWidget(self.sayi_etiketi)
        sol.addWidget(self.gorunum, 1)

        sag = QtWidgets.QVBoxLayout()
        sag.addWidget(self.durum_etiketi)
        sag.addWidget(self.grafik_roll.widget, 1)
        sag.addWidget(self.grafik_pitch.widget, 1)

        ana = QtWidgets.QHBoxLayout(self)
        ana.addLayout(sol, 62)
        ana.addLayout(sag, 38)

    # ---- her karede calisan dongu ---------------------------------------
    def guncelle(self):
        simdi = time.perf_counter()
        dt = simdi - self.t_onceki
        self.t_onceki = simdi
        gecen = simdi - self.t_baslangic

        # 1) En son gelen satiri al
        veri, satir = self.okuyucu.oku()
        if veri is not None:
            self.roll_hedef  = RSIGN * veri[6] / 100.0
            self.pitch_hedef = PSIGN * veri[7] / 100.0
            r_ham, p_ham = ivmeden_aci(veri[0], veri[1], veri[2])
            self.roll_ham, self.pitch_ham = RSIGN * r_ham, PSIGN * p_ham
        else:
            satir = 0

        # 2) Yumusat (sadece gorsel; filtrelemenin asil isi karttadir)
        roll  = self.roll_suzgec.guncelle(self.roll_hedef, dt)
        pitch = self.pitch_suzgec.guncelle(self.pitch_hedef, dt)

        # 3) Ciz
        self.roket.yonlendir(roll, pitch, gecen)
        self.grafik_roll.guncelle(roll, self.roll_ham)
        self.grafik_pitch.guncelle(pitch, self.pitch_ham)

        # 4) Sayisal gostergeler
        self.kare += 1
        fps = self.kare / gecen
        self.sayi_etiketi.setText("ROLL  %+7.2f\nPITCH %+7.2f" % (roll, pitch))
        self.durum_etiketi.setText(
            "%.0f fps | %d satir | gri cizgi: ham islenmemis veri" % (fps, satir))

    def closeEvent(self, olay):
        self.zamanlayici.stop()
        self.okuyucu.durdur()
        olay.accept()


# ==========================================================================
# 7) BASLANGIC
# ==========================================================================

def port_sec():
    if PORT:
        return PORT
    portlar = list(serial.tools.list_ports.comports())
    if not portlar:
        print("Seri port bulunamadi. Karti tak ya da PORT degiskenini elle yaz.")
        sys.exit(1)
    print("Bulunan portlar:")
    for p in portlar:
        print("   ", p.device, "-", p.description)
    print("Secilen:", portlar[0].device)
    return portlar[0].device


def main():
    okuyucu = SeriOkuyucu(port_sec(), BAUD, LOG_CSV)
    okuyucu.start()

    pg.mkQApp("Roket")
    pencere = Pencere(okuyucu)
    pencere.show()
    pg.exec()


if __name__ == "__main__":
    main()