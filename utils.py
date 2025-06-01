import random
from data_classes import Drone, DeliveryPoint, NoFlyZone

def generate_random_deliveries(num_deliveries):
    deliveries = []
    for i in range(1, num_deliveries + 1):
        pos = (random.uniform(0, 100), random.uniform(0, 100))
        ağırlık = random.uniform(1.0, 5.0)
        öncelik = random.randint(1, 5)
        saataralıgı = ("09:00", "12:00")
        deliveries.append(DeliveryPoint(id=i, pos=pos, ağırlık=ağırlık, öncelik=öncelik, saataralıgı=saataralıgı))
    return deliveries

def generate_random_noflyzones(num_zones):
    zones = []
    for i in range(1, num_zones + 1):
        x = random.uniform(10, 90)
        y = random.uniform(10, 90)
        size = random.uniform(5, 15)
        coords = [
            (x, y),
            (x + size, y),
            (x + size, y + size),
            (x, y + size)
        ]
        zones.append(NoFlyZone(id=i, koordinat=coords, aktifzaman=("09:00", "12:00")))
    return zones

def generate_random_drones(num_drones):
    drones = []
    for i in range(1, num_drones + 1):
        start_pos = (random.uniform(0, 100), random.uniform(0, 100))
        drones.append(Drone(id=i, maksagırlık=5.0, battery=10000, speed=10.0, başlangıçpozisyonu=start_pos))
    return drones
