import math as m
import random
import numpy as np
import matplotlib.pyplot as plt

# Coordonnées radar (début de piste)
radar_x, radar_y = 0, 2
Radar = (radar_x, radar_y)

# Position de l'avion (modifiable ou aléatoire)
avion_x = 3.0
avion_y = 4.0
Avion = (avion_x, avion_y)

# Piste d'atterrissage
X = np.linspace(0, 5, 500)
piste_y = [2] * 500

dico = {}

def binomial(n, p):
    """Calcule le coefficient binomial avec mémoïsation."""
    if p == 0 or p == n:
        return 1
    elif (n, p) in dico:
        return dico[(n, p)]
    else:
        dico[(n, p)] = binomial(n - 1, p) + binomial(n - 1, p - 1)
    return dico[(n, p)]

def bernstein(x, n, k):
    """Évalue le polynôme de Bernstein."""
    return binomial(n, k) * (x**k) * ((1 - x)**(n - k))

def Bezier0(Ps, t):
    """Renvoie le point de la courbe de Bézier d'ordre quelconque pour t dans [0, 1]."""
    x, y = 0.0, 0.0
    n = len(Ps) - 1
    for k in range(n + 1):
        xk, yk = Ps[k]
        b = bernstein(t, n, k)
        x += b * xk
        y += b * yk
    return (x, y)

def coord(Ps):
    """Échantillonne les coordonnées successives de la courbe de Bézier."""
    Ps = [(float(x), float(y)) for x, y in Ps]
    return [Bezier0(Ps, t) for t in np.arange(0, 1.1, 0.1)]

def courbure(trajectoire):
    """Retourne les valeurs de courbure le long d'une trajectoire."""
    Ps1 = np.array(trajectoire)
    x_t = np.gradient(Ps1[:, 0])
    y_t = np.gradient(Ps1[:, 1])
    xx_t = np.gradient(x_t)
    yy_t = np.gradient(y_t)
    denom = (x_t**2 + y_t**2) ** 1.5
    denom = np.where(denom == 0, 1e-9, denom)
    return np.abs(xx_t * y_t - x_t * yy_t) / denom

def dist(trajectoire) -> float:
    """Distance euclidienne cumulée de la trajectoire."""
    x = [p[0] for p in trajectoire]
    y = [p[1] for p in trajectoire]
    s = sum((x[k] - x[k - 1])**2 + (y[k] - y[k - 1])**2 for k in range(1, len(trajectoire)))
    return float(np.sqrt(s))

def variationcourbure(trajectoire) -> float:
    """Mesure la régularité des variations de courbure."""
    courbures = courbure(trajectoire)
    return float(sum(abs(courbures[i] - courbures[i - 1]) for i in range(1, len(courbures))) / len(courbures))

def cout(Ps):
    """Fonction coût pénalisant les virages trop serrés et les trajectoires non physiques."""
    trajectoire = coord(Ps)
    max_c = max(courbure(trajectoire)) / 100.0
    rayon_min = 1.0 / max_c if max_c != 0 else float('inf')
    distance = dist(trajectoire) / 10.0
    variation = variationcourbure(trajectoire) / 10.0

    # Contrainte sur le rayon minimal de virage
    if rayon_min < 4:
        return 1e6

    # Contraintes topologiques de descente progressive
    x0, x1, x2 = Ps[0][0], Ps[1][0], Ps[2][0]
    if x1 > x0 or x2 > x1:
        return 1e6

    y0, y1, y2, y3 = Ps[0][1], Ps[1][1], Ps[2][1], Ps[3][1]
    if not (y0 > y1 > y2 > y3):
        return 1e6

    d = 0.01
    e = 1.0
    f = 1.0
    return d * max_c + e * distance + f * variation

def generation_trajectoire(Avion, Radar):
    """Génère une trajectoire candidate avec des points de contrôle aléatoires."""
    pcx1 = np.random.uniform(-15, -5)
    pcy1 = np.random.uniform(2.75, 3.25)
    pcx2 = np.random.uniform(-22, -12)
    pcy2 = np.random.uniform(2.25, 2.75)
    pcx3 = np.random.uniform(-20, -10)
    return [Avion, (pcx1, pcy1), (pcx2, pcy2), (pcx3, 2.0), Radar]

def crossover(ind1, ind2) -> list:
    """Croisement barycentrique continu entre deux trajectoires."""
    alpha = random.random()
    enfant = [(alpha * k1[0] + (1 - alpha) * k2[0], alpha * k1[1] + (1 - alpha) * k2[1]) for k1, k2 in zip(ind1, ind2)]
    if enfant[3][0] >= -2:
        enfant[3] = (-2.0 - abs(enfant[3][0] + 0.1), enfant[3][1])
    return enfant

def mutation(Ps, p_mut=0.2, sigma=0.5):
    """Mutation gaussienne appliquée aux points de contrôle."""
    if np.random.rand() < p_mut:
        p1 = (Ps[1][0] + np.random.normal(0, sigma), Ps[1][1] + np.random.normal(0, sigma))
        p2 = (Ps[2][0] + np.random.normal(0, sigma), Ps[2][1] + np.random.normal(0, sigma))
        mx3 = Ps[3][0] + np.random.normal(0, sigma)
        if mx3 >= -2:
            mx3 = -2.0 - abs(mx3 + 0.1)
        p3 = (mx3, Ps[3][1])
        return [Ps[0], p1, p2, p3, Ps[-1]]
    return Ps

def AG(Avion, Radar, n=300, population_size=40, p_mut=0.2, sigma=0.5):
    """Optimisation par Algorithme Génétique."""
    population = [generation_trajectoire(Avion, Radar) for _ in range(population_size)]
    for _ in range(n):
        population = sorted(population, key=cout)
        survivors = list(population[:4])
        enfants = []
        for i in range(len(survivors)):
            for j in range(i + 1, len(survivors)):
                m = crossover(survivors[i], survivors[j])
                enfants.append(mutation(m, p_mut=p_mut, sigma=sigma))
        population.extend(enfants)
        population = sorted(population, key=cout)[:population_size]
    return population[0]

if __name__ == "__main__":
    best_traj = AG(Avion, Radar, n=150)
    pts = coord(best_traj)
    print(f"Meilleur coût atteint : {cout(best_traj):.4f}")
