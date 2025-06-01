import random
import math
from typing import List
from data_classes import Drone, DeliveryPoint, NoFlyZone

# Euclidean mesafe fonksiyonu
def euclidean(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

# Sabit enerji tüketimi
ENERGY_PER_METER = 10  # 1 metre uçuşta 10 mAh harcar (örnek)

# Popülasyon başlatma
def initialize_population(deliveries: List[DeliveryPoint], population_size: int) -> List[List[DeliveryPoint]]:
    population = []
    for _ in range(population_size):
        individual = deliveries.copy()
        random.shuffle(individual)
        population.append(individual)
    return population

# Fitness fonksiyonu mutasyon crossover ga fonkisiyonun evrim sürecine benzetmesinden gelir.
def fitness(drone: Drone, individual: List[DeliveryPoint], noflyzones: List[NoFlyZone]) -> float:
    total_energy = 0
    penalty = 0
    completed = 0
    current_pos = drone.başlangıçpozisyonu
    
    for delivery in individual:
        # Mesafe
        dist = euclidean(current_pos, delivery.pos)
        energy_needed = dist * ENERGY_PER_METER

        # Battery yetiyor mu?
        if (total_energy + energy_needed) > drone.battery:
            break  # daha fazla teslimat yapılamaz
        
        # No-Fly Zone kontrol
        in_nofly = False
        for nfz in noflyzones:
            if point_in_polygon(delivery.pos, nfz.koordinat):
                in_nofly = True
                break
        
        if in_nofly:
            penalty += 1000  # ceza ekle
        
        # Başarılı teslimat
        completed += 1
        total_energy += energy_needed
        current_pos = delivery.pos
    
    # Fitness skoru
    score = completed * 1000 - total_energy - penalty
    return score

# Crossover
def crossover(parent1: List[DeliveryPoint], parent2: List[DeliveryPoint]) -> List[DeliveryPoint]:
    size = len(parent1)
    start, end = sorted(random.sample(range(size), 2))
    child = [None] * size
    child[start:end] = parent1[start:end]
    pointer = 0
    for delivery in parent2:
        if delivery not in child:
            while child[pointer] is not None:
                pointer += 1
            child[pointer] = delivery
    return child

# Mutasyon 
def mutate(individual: List[DeliveryPoint], mutation_rate=0.1):
    for i in range(len(individual)):
        if random.random() < mutation_rate:
            j = random.randint(0, len(individual) - 1)
            individual[i], individual[j] = individual[j], individual[i]

# No-Fly Zone kontrol fonksiyonu:
def point_in_polygon(point: tuple, polygon: List[tuple]) -> bool:
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

def run_ga(drone: Drone, deliveries: List[DeliveryPoint], noflyzones: List[NoFlyZone],
           population_size=20, generations=50, mutation_rate=0.1):

    # Popülasyonu başlat
    population = initialize_population(deliveries, population_size)

    # Her jenerasyon
    for generation in range(generations):
        # Fitness hesapla
        scored_population = [(individual, fitness(drone, individual, noflyzones)) for individual in population]
        
        # Skora göre sırala (en iyi başa)
        scored_population.sort(key=lambda x: x[1], reverse=True)
        
        # En iyi bireyi yazdır (isteğe bağlı)
        best_score = scored_population[0][1]
        print(f"Generation {generation+1}, Best Fitness: {best_score}")
        
        # Elitizm → en iyi %20’yi koru
        retain_length = int(len(scored_population) * 0.2)
        new_population = [ind[0] for ind in scored_population[:retain_length]]
        
        # Yeni çocuklar oluştur
        while len(new_population) < population_size:
            parent1 = random.choice(new_population)
            parent2 = random.choice(new_population)
            child = crossover(parent1, parent2)
            mutate(child, mutation_rate)
            new_population.append(child)
        
        # Yeni popülasyonla devam et
        population = new_population

    # Son jenerasyondan en iyi bireyi döndür
    final_best = scored_population[0][0]
    return final_best


if __name__ == "__main__":
    # Örnek drone
    drone = Drone(id=1, maksagırlık=5.0, battery=5000, speed=10.0, başlangıçpozisyonu=(0, 0))

    # Örnek teslimatlar
    deliveries = [
        DeliveryPoint(id=1, pos=(10, 10), ağırlık=2.0, öncelik=3, saataralıgı=("09:00", "10:00")),
        DeliveryPoint(id=2, pos=(30, 5), ağırlık=1.0, öncelik=4, saataralıgı=("09:30", "11:00")),
        DeliveryPoint(id=3, pos=(20, 20), ağırlık=3.0, öncelik=2, saataralıgı=("09:00", "10:30")),
    ]

    # Örnek no-fly zone
    noflyzones = [
        NoFlyZone(id=1, koordinat=[(15, 0), (25, 0), (25, 10), (15, 10)], aktifzaman=("09:00", "11:00"))
    ]

    # GA çalıştır
    best_route = run_ga(drone, deliveries, noflyzones,
                       population_size=20, generations=50, mutation_rate=0.1)

    # Sonuç yazdır
    print("GA En İyi Rota:")
    for d in best_route:
        print(f"Teslimat {d.id} ({d.pos})")

