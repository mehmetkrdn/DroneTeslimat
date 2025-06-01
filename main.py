from utils import generate_random_deliveries, generate_random_noflyzones, generate_random_drones
import time
from ga import run_ga
from data_classes import Drone, DeliveryPoint, NoFlyZone
import matplotlib.pyplot as plt
from a import astar, filter_feasible_deliveries


harcananenerji = 10

# GÖRSELLEŞTİRME FONKSİYONU:
def plot_route(drone: Drone, route: list, noflyzones: list, title="Teslimat Rotası"):
    plt.figure(figsize=(8, 8))
    
    # Drone başlangıç noktası
    plt.scatter(drone.başlangıçpozisyonu[0], drone.başlangıçpozisyonu[1], c='blue', marker='s', s=100, label='Drone Başlangıç')
    
    # Teslimat noktaları
    xs = [delivery.pos[0] for delivery in route]
    ys = [delivery.pos[1] for delivery in route]
    plt.scatter(xs, ys, c='green', marker='o', s=80, label='Teslimat Noktası')

    # Çizilen rota
    full_route_x = [drone.başlangıçpozisyonu[0]] + xs
    full_route_y = [drone.başlangıçpozisyonu[1]] + ys
    plt.plot(full_route_x, full_route_y, c='black', linestyle='--', linewidth=2, label='Rota')

    # No-Fly Zone'ları çiz
    for nfz in noflyzones:
        coords = nfz.koordinat + [nfz.koordinat[0]]  # kapanması için
        xs_nfz = [p[0] for p in coords]
        ys_nfz = [p[1] for p in coords]
        plt.plot(xs_nfz, ys_nfz, c='red', linewidth=2, label='No-Fly Zone')

    plt.title(title)
    plt.xlabel('X Koordinat')
    plt.ylabel('Y Koordinat')
    plt.legend()
    plt.grid(True)
    plt.show()

# SENARYO FONKSİYONU:
def run_scenario(num_drones, num_deliveries, num_nfz, generations=50):
    print(f"\n=== TEST: {num_drones} Drone, {num_deliveries} Teslimat, {num_nfz} No-Fly Zone ===")

    drones = generate_random_drones(num_drones)
    deliveries = generate_random_deliveries(num_deliveries)
    noflyzones = generate_random_noflyzones(num_nfz)

    total_completed = 0
    total_energy = 0
    total_deliveries = len(deliveries)

    start_time = time.time()

    for drone in drones:
        best_route = run_ga(drone, deliveries, noflyzones, generations=generations)
        
        completed = len(best_route)
        energy = 0
        current_pos = drone.başlangıçpozisyonu
        for delivery in best_route:
            dist = ((current_pos[0] - delivery.pos[0]) ** 2 + (current_pos[1] - delivery.pos[1]) ** 2) ** 0.5
            energy += dist * harcananenerji
            current_pos = delivery.pos
        
        total_completed += completed
        total_energy += energy

    end_time = time.time()

    tamamlanan_yuzde = (total_completed / (total_deliveries * len(drones))) * 100
    ortalama_enerji = total_energy / total_completed if total_completed > 0 else 0
    sure = end_time - start_time

    print(f"Tamamlanan Teslimat Yüzdesi: {tamamlanan_yuzde:.2f}%")
    print(f"Ortalama Enerji Tüketimi: {ortalama_enerji:.2f} mAh")
    print(f"Algoritma Çalışma Süresi: {sure:.2f} saniye")

    # Örnek → ilk drone için en iyi rotayı çizelim:
    best_route = run_ga(drones[0], deliveries, noflyzones, generations=generations)
    plot_route(drones[0], best_route, noflyzones, title=f"GA Rota - {num_drones} Drone, {num_deliveries} Teslimat")

from a import astar, filter_feasible_deliveries

def run_astar_scenario(num_drones, num_deliveries, num_nfz):
    print(f"\n=== A* TEST: {num_drones} Drone, {num_deliveries} Teslimat, {num_nfz} No-Fly Zone ===")

    drones = generate_random_drones(num_drones)
    deliveries = generate_random_deliveries(num_deliveries)
    noflyzones = generate_random_noflyzones(num_nfz)

    total_completed = 0
    total_energy = 0
    total_deliveries = len(deliveries)

    drone_routes = {}  # Her drone için rota listesi

    start_time = time.time()

    for drone in drones:
        drone_routes[drone.id] = []  # Bu drone’un rotası
        remaining_deliveries = deliveries.copy()
        current_pos = drone.başlangıçpozisyonu
        current_battery = drone.battery

        while remaining_deliveries:
            feasible = filter_feasible_deliveries(drone, remaining_deliveries, noflyzones)

            if not feasible:
                break

            best_path = astar(drone, feasible, noflyzones)
            if not best_path:
                break

            target_pos = best_path[-1]

            delivered = None
            for d in feasible:
                if d.pos == target_pos:
                    delivered = d
                    break

            if delivered:
                dist = ((current_pos[0] - delivered.pos[0]) ** 2 + (current_pos[1] - delivered.pos[1]) ** 2) ** 0.5
                energy_needed = dist * 2 * harcananenerji

                if current_battery < energy_needed:
                    break

                current_battery -= energy_needed
                total_energy += energy_needed
                total_completed += 1

                # ROTAYI KAYDET → base → hedef → base
                drone_routes[drone.id].append((current_pos, delivered.pos, drone.başlangıçpozisyonu))

                current_pos = drone.başlangıçpozisyonu

                remaining_deliveries.remove(delivered)
            else:
                break

    end_time = time.time()

    tamamlanan_yuzde = (total_completed / (total_deliveries * len(drones))) * 100
    ortalama_enerji = total_energy / total_completed if total_completed > 0 else 0
    sure = end_time - start_time

    print(f"Tamamlanan Teslimat Yüzdesi: {tamamlanan_yuzde:.2f}%")
    print(f"Ortalama Enerji Tüketimi: {ortalama_enerji:.2f} mAh")
    print(f"Algoritma Çalışma Süresi: {sure:.2f} saniye")

    # Örnek: İlk drone’un rotasını çizdir
    first_drone = drones[0]
    fig = plt.figure(figsize=(8, 8))
    plt.scatter(first_drone.başlangıçpozisyonu[0], first_drone.başlangıçpozisyonu[1], c='blue', marker='s', s=100, label='Drone Başlangıç')

    for segment in drone_routes[first_drone.id]:
        base_pos, delivery_pos, back_pos = segment
        # base → hedef
        plt.plot([base_pos[0], delivery_pos[0]], [base_pos[1], delivery_pos[1]], c='black', linestyle='--', linewidth=2)
        # hedef → base
        plt.plot([delivery_pos[0], back_pos[0]], [delivery_pos[1], back_pos[1]], c='gray', linestyle='--', linewidth=1)

    # No-Fly Zone'ları da çiz
    for nfz in noflyzones:
        coords = nfz.koordinat + [nfz.koordinat[0]]
        xs_nfz = [p[0] for p in coords]
        ys_nfz = [p[1] for p in coords]
        plt.plot(xs_nfz, ys_nfz, c='red', linewidth=2, label='No-Fly Zone')

    plt.title(f"A* Rota - {num_drones} Drone, {num_deliveries} Teslimat")
    plt.xlabel('X Koordinat')
    plt.ylabel('Y Koordinat')
    plt.legend()
    plt.grid(True)
    plt.show()


# === MAIN ===
if __name__ == "__main__":
    # SENARYO 1 → GA
    run_scenario(num_drones=5, num_deliveries=20, num_nfz=2, generations=50)

    # SENARYO 1 → A*
    run_astar_scenario(num_drones=5, num_deliveries=20, num_nfz=2)

    # SENARYO 2 → GA
    run_scenario(num_drones=10, num_deliveries=50, num_nfz=5, generations=50)

    # SENARYO 2 → A*
    run_astar_scenario(num_drones=10, num_deliveries=50, num_nfz=5)

