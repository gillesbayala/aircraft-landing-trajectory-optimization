# 🛩️ Aircraft Landing Trajectory Optimization using Bézier Curves & Genetic Algorithms

[![Python](https://img.shields.io/badge/Python-3.9%2B-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Mathematics](https://img.shields.io/badge/Theory-Differential%20Geometry-orange.svg)](#mathematical-modeling)

Engineering project modeling and optimizing 2D aircraft descent and landing profiles under aeronautical and kinodynamic constraints (runway alignment, curvature bounds, passenger comfort).

---

## 📌 Problem Formulation

Terminal maneuvering area (TMA) air traffic control requires continuous management of descent funnels while ensuring aircraft stability. Standard radar procedures impose strict approach paths:

* **Objective:** Find an optimal flight trajectory $C(t)$ connecting an arbitrary initial entry fix $\mathbf{P}_{\text{aircraft}}$ to the final approach radar fix $\mathbf{P}_{\text{radar}}$ aligned with runway threshold.
* **Challenges:** Discard non-flyable profiles (excessive centripetal acceleration, sharp turns, backward segments) while minimizing distance and fuel consumption.

---

## 📐 Mathematical Modeling

### 1. Parametric Bézier Trajectories
Trajectories are modeled as degree-$n$ parametric Bézier curves defined by $n+1$ control points $\mathbf{P}_k = (x_k, y_k)$:
$$\mathbf{C}(t) = \sum_{k=0}^{n} B_{k,n}(t) \mathbf{P}_k, \quad t \in [0, 1]$$

where $B_{k,n}(t) = \binom{n}{k} t^k (1 - t)^{n-k}$ denote Bernstein basis polynomials.

### 2. Differential Geometry & Kinematic Curvature
To ensure flyability and passenger comfort, the signed curvature $\kappa(t)$ is computed from first and second derivative vectors:
$$\kappa(t) = \frac{|\dot{x}(t)\ddot{y}(t) - \ddot{x}(t)\dot{y}(t)|}{\left(\dot{x}(t)^2 + \dot{y}(t)^2\right)^{3/2}}$$

The instantaneous radius of curvature $R(t) = \frac{1}{\kappa(t)}$ is lower-bounded by the aircraft load-factor and roll-angle limits ($\phi$):
$$R_{\text{min}} = \frac{v^2}{a_c \tan(\phi)}$$

### 3. Multi-Objective Cost Function
Candidate paths are evaluated by a weighted cost metric:
$$J(\mathbf{C}) = w_d \cdot D(\mathbf{C}) + w_\kappa \cdot \kappa_{\text{max}} + w_v \cdot \Delta \kappa + \text{Penalties}$$

* $D(\mathbf{C})$: Total path length.
* $\kappa_{\text{max}}$: Maximum curvature along the descent.
* $\Delta \kappa$: Curvature rate of change (jerk / roll-rate transition proxy).
* **Penalties:** Infinite cost ($10^6$) assigned to profiles with $R_{\text{min}} < 4$ or non-monotone descent paths.

---

## 🧬 Evolutionary Optimization Architecture

A continuous Genetic Algorithm (GA) optimizes intermediate control points $(\mathbf{PC}_1, \mathbf{PC}_2, \mathbf{PC}_3)$ while keeping endpoints $\mathbf{P}_{\text{aircraft}}$ and $\mathbf{P}_{\text{radar}}$ grounded to physical coordinates:

1. **Selection:** Elitist selection based on rank ranking.
2. **Crossover:** Linear barycentric recombination between control vectors:
   $$\mathbf{P}_{\text{child}} = \alpha \mathbf{P}_{\text{parent}_1} + (1 - \alpha) \mathbf{P}_{\text{parent}_2}, \quad \alpha \sim \mathcal{U}(0, 1)$$
3. **Mutation:** Gaussian perturbation $\mathcal{N}(0, \sigma^2)$ applied to control point coordinates.

---

## 📊 Benchmark & Results

Comparative convergence between Random Generation (Monte Carlo), Local Greedy Search, and Genetic Optimization over 2,000 iterations:

| Method | Convergence Stability | Flyability Guarantee | Final Cost Score |
| :--- | :---: | :---: | :---: |
| **Random Search (Monte Carlo)** | Unstable (High Variance) | Weak (Random $R_{\text{min}}$ violations) | $\approx 1.28$ |
| **Local Step Search** | Premature local minima traps | Moderate | $\approx 0.80$ |
| **Genetic Algorithm** | **Global convergence within 250 epochs** | **100% compliant with ICAO limits** | **$\approx 0.10 - 0.45$** |

---

## 🚀 Quickstart

```bash
# Clone the repository
git clone [https://github.com/gillesbayala/aircraft-landing-trajectory-optimization.git](https://github.com/gillesbayala/aircraft-landing-trajectory-optimization.git)
cd aircraft-landing-trajectory-optimization

# Install dependencies
pip install -r requirements.txt

# Run simulation
python trajectory_optimizer.py
