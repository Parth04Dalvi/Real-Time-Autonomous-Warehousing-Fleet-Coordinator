# NEXUS — Autonomous Fleet Coordinator v2.1

NEXUS is a high-performance, multi-agent autonomous warehousing simulator designed to model, visualize, and optimize AGV (Automated Guided Vehicle) fleet coordination in real time. Built with a modular architecture, NEXUS utilizes a **Time-Expanded Space-Time $A^*$ Pathfinder** to eliminate deadlocks, an **Auction-Based Task Broker** for efficient job allocation, and a localized **Spatial Hash Grid paired with Reynolds Steering Behaviors** for dynamic collision avoidance at 60 FPS.



---

## 🚀 Core Architecture & Features

NEXUS shifts away from monolithic simulation loops by isolating concerns into specialized, high-efficiency engines:

*   **Visualizer & Core Loop (`index.html`)**: A lightweight, responsive HTML5 Canvas renderer utilizing `requestAnimationFrame` (RAF). Styled with Tailwind CSS and themed around a high-tech telemetry dashboard featuring a crisp cyber-grid aesthetic.
*   **Space-Time Pathfinder (`Pathfinder.js`)**: Implements an $A^*$ algorithm expanded into the $3^{\text{rd}}$ dimension (Time). Agents reserve trajectories in a global `Reservation Table` at coordinates $(x, y, t)$, proactively routing around intersections before moving.
*   **Auction-Based Task Broker (`TaskBroker.js`)**: Replaces simple FIFO assignment with a Contract Net Protocol auction. Idle AGVs bid on incoming orders using a cost heuristic that factors in Manhattan distance, order age, and battery discharge overhead.
*   **Reactive Collision Proxy (`CollisionProxy.js`)**: A microsecond-level localized physics proxy. By querying a highly optimized **Spatial Hash Grid**, it calculates inverse-proximity separation vectors to handle minor real-time physical rounding errors seamlessly.

---

## 📂 Project Structure

```text
nexus-fleet-coordinator/
├── index.html          # Main dashboard UI, canvas controller, & telemetry view
└── src/
    ├── Pathfinder.js   # Time-Expanded A* & Space-Time Reservation Table
    ├── TaskBroker.js   # Auction-based combinatorial task allocator
    └── CollisionProxy.js # Local Reynolds steering & spatial-hash separation system
