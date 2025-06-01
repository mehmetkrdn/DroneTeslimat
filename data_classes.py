from dataclasses import dataclass #dataclassda otomatik olarak init gibi metotlar oluşur fakat daha okunaklı olması için normal class yerine tanımlama yerine böyle tanımladık
from typing import Tuple, List

@dataclass
class Drone:
    id: int #dronekimlikno
    maksagırlık: float #maks taşıyacağı ağırlık dronun
    battery: int  #şarj
    speed: float #hız
    başlangıçpozisyonu: Tuple[float, float] #başlangıç noktası  x y - değiştirilemez tuple.

@dataclass
class DeliveryPoint:
    id: int #teslim edilecek yerin id no
    pos: Tuple[float, float] #teslim edilecek yerin koordinatı
    ağırlık: float #paketin ağırlık
    öncelik: int #teslimat önceliği
    saataralıgı: Tuple[str, str] #sat aralığı

@dataclass
class NoFlyZone:
    id: int #yasakbölge idsi
    koordinat: List[Tuple[float, float]] #köşegenlerin koordinatı xy
    aktifzaman: Tuple[str, str] #bölgenin aktif olduğu zaman
