# DroneTeslimat -Optimizasyonu 

Bu proje, **çok kısıtlı ortamlarda dinamik teslimat planlaması** için optimize edilmiş bir **drone filo yönetim sistemidir**. Projede, drone'ların belirli engelleri aşarak hedef noktalara en kısa sürede ve etkili biçimde teslimat yapması amaçlanmaktadır.

## 🔍 Amaç

- Drone'ların dar ve sınırlı alanlarda çarpışmadan teslimat yapabilmesini sağlamak  
- A*, Genetik Algoritma ve CSP (Constraint Satisfaction Problem) gibi algoritmalarla **dinamik rota optimizasyonu yapmak**  
- Teslimat sürelerini ve kaynak kullanımını minimize etmek

## 🛠️ Kullanılan Teknolojiler

- **Python** 🐍  
- **A\* Algoritması** – en kısa yol hesaplaması  
- **Genetik Algoritma** – global optimizasyon  
- **CSP (Constraint Satisfaction Problem)** – çatışmasız görev atama  
- **Tkinter veya Matplotlib** – (varsa) görselleştirme için

## 📁 Proje Yapısı
DroneTeslimat/
├── main.py # Uygulamanın giriş noktası
├── a.py # Alternatif senaryo/test dosyası
├── data_classes.py # Drone ve görev veri modelleri
├── data_genpy # Rastgele görev/drone verisi üretimi (Python script)
├── dataset.json # Örnek görev ve koordinat verileri
├── ga.py # Genetik algoritma ile optimizasyon modülü
├── utils.py # Yardımcı fonksiyonlar (mesafe, skor hesaplama, vb.)
├── grup7_rapor.pdf # Proje teknik raporu
├── README.md # Bu tanıtım dosyası


## 🚀 Başlangıç

Projeyi çalıştırmak için:

1. Depoyu klonlayın:
   ```bash
   git clone https://github.com/mehmetkrdn/DroneTeslimat.git
   cd DroneTeslimat
2. Uygulamayı başlatın:
python main.py

## Özellikler
✅ Engelleri otomatik algılama ve yol planlama

✅ Çoklu drone için görev dağılımı

✅ Gerçek zamanlı görselleştirme (opsiyonel)

✅ Kullanıcı tarafından tanımlanabilir haritalar

✅ Performans karşılaştırmaları ve analizler

