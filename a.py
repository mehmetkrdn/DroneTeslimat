import heapq
import math
from typing import Dict, Tuple, List
from data_classes import DeliveryPoint, Drone, NoFlyZone

# hedef flyzoneda mı
def point_in_polygon(point: Tuple[float, float], polygon: List[Tuple[float, float]]) -> bool:
    x, y = point
    inside = False
    n = len(polygon)
    p1x, p1y = polygon[0]

    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y + 1e-9) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside

# mesafe fonksiyonu
def euclidean(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

# maliyet fonksiyonu
def cost_fn(distance, ağırlık, öncelik):
    return distance * ağırlık + (6 - öncelik) * 100  # 1. öncelik = 500 ceza, 5 = 100 ceza

enerjiharcama = 10  # uçuşta harcanan enerji metre başına

# tek paket kısıtlaması

# tek paket kısıtlaması + battery kontrolü
def filter_feasible_deliveries(drone: Drone, deliveries: List[DeliveryPoint], noflyzones: List[NoFlyZone]) -> List[DeliveryPoint]:
    feasible_deliveries = []
    
    for delivery in deliveries:
        # Kapasite kontrol
        if delivery.ağırlık > drone.maksagırlık:
            continue
        
        # No-Fly Zone kontrol
        in_nofly = False
        for nfz in noflyzones:
            if point_in_polygon(delivery.pos, nfz.koordinat):
                in_nofly = True
                break
        
        if in_nofly:
            continue

        # şarj kontrol (gidiş + dönüş için enerji hesabı!)
        distance = euclidean(drone.başlangıçpozisyonu, delivery.pos)
        total_energy_needed = distance * 2 * enerjiharcama  # gidiş ve dönüş enerjisi

        if total_energy_needed > drone.battery:
            continue  # yetersiz batarya → geç

        # Bu teslimat uygun → listeye ekle
        feasible_deliveries.append(delivery)
    
    return feasible_deliveries

# A* algoritması
def astar(drone: Drone, deliveries: List[DeliveryPoint], noflyzones: List[NoFlyZone]):
    start = drone.başlangıçpozisyonu
    goals = [d.pos for d in deliveries]
    open_set = []
    heapq.heappush(open_set, (0, start, []))  # (f_score, current_pos, path_so_far)

    visited = set()

    while open_set:
        f_score, current, path = heapq.heappop(open_set)

        if current in goals:
            return path + [current]

        if current in visited:
            continue
        visited.add(current)

        for delivery in deliveries:
            if delivery.pos in visited:
                continue

            # Mesafe hesapla
            dist = euclidean(current, delivery.pos)

            # Kapasite kontrol
            if delivery.ağırlık > drone.maksagırlık:
                continue

            # Normal cost + heuristic hesapla
            cost = cost_fn(dist, delivery.ağırlık, delivery.öncelik)
            heuristic = euclidean(delivery.pos, drone.başlangıçpozisyonu)

            # No-Fly Zone cezası
            nofly_penalty = 0
            for nfz in noflyzones:
                if point_in_polygon(delivery.pos, nfz.koordinat):
                    nofly_penalty += 1000  # ceza değeri

            # Total score
            total_score = cost + heuristic + nofly_penalty

            # Open set'e ekle
            heapq.heappush(open_set, (total_score, delivery.pos, path + [current]))

    # >>> DİKKAT: return None buraya gelmeli! (while döngüsü dışına)
    return None  # uygun rota bulunamadı

# Test için main kısmı
if __name__ == "__main__":
    # Örnek drone
    drone = Drone(id=1, maksagırlık=5.0, battery=5000, speed=10.0, başlangıçpozisyonu=(0, 0))

    # Örnek teslimatlar
    deliveries = [
        DeliveryPoint(id=1, pos=(10, 10), ağırlık=2.0, öncelik=3, saataralıgı=("09:00", "10:00")),
        DeliveryPoint(id=2, pos=(30, 5), ağırlık=1.0, öncelik=4, saataralıgı=("09:30", "11:00")),
    ]

    # Örnek no-fly zone (bir kare bölge)
    noflyzones = [
        NoFlyZone(id=1, koordinat=[(15, 0), (25, 0), (25, 10), (15, 10)], aktifzaman=("09:00", "11:00"))
    ]

    # Filtrele
    feasible = filter_feasible_deliveries(drone, deliveries, noflyzones)

    # A* çalıştır
    path = astar(drone, feasible, noflyzones)

    # Sonuç yazdır
    print("Bulunan rota:", path)
