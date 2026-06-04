/**
 * NEXUS Spatial-Temporal Pathfinding Engine
 * Implements Time-Expanded A* using a Space-Time Reservation Table
 */
export class SpatialTemporalPathfinder {
  constructor(cols, rows) {
    this.cols = cols;
    this.rows = rows;
    // Map key: "c,r,t" -> true (reserved)
    this.reservationTable = new Set();
  }

  /**
   * Generates a key for the space-time lookup table
   */
  _makeKey(c, r, t) {
    return `${c},${r},${t}`;
  }

  /**
   * Reserve a planned trajectory line in space-time
   */
  reservePath(path, startPosIdx = 0) {
    path.forEach((node, index) => {
      const t = startPosIdx + index;
      this.reservationTable.add(this._makeKey(node.c, node.r, t));
      // Also reserve immediate neighbors at time 't' to act as a safety buffer
      this._reserveBuffer(node.c, node.r, t);
    });
  }

  _reserveBuffer(c, r, t) {
    const dirs = [[1,0], [-1,0], [0,1], [0,-1]];
    dirs.forEach(([dc, dr]) => {
      const nc = c + dc;
      const nr = r + dr;
      if (nc >= 0 && nc < this.cols && nr >= 0 && nr < this.rows) {
        // Soft reservation
        this.reservationTable.add(this._makeKey(nc, nr, t) + "_buffer");
      }
    });
  }

  /**
   * Clear space-time allocations when a robot completes or aborts a task
   */
  clearReservations() {
    this.reservationTable.clear();
  }

  /**
   * Computes a path from (sc, sr) to (ec, er) starting at time tick 'startTime'
   * @returns {Array<{c, r, t}>|null}
   */
  findSpaceTimePath(sc, sr, ec, er, startTime = 0, staticGrid) {
    if (sc === ec && sr === er) return [];

    const key = (c, r, t) => `${c},${r},${t}`;
    const h = (c, r) => Math.abs(c - ec) + Math.abs(r - er);
    
    const openSet = [];
    const gScore = new Map(); // key -> cost
    const cameFrom = new Map(); // key -> parentKey

    const startKey = key(sc, sr, startTime);
    gScore.set(startKey, 0);
    openSet.push({ c: sc, r: r, t: startTime, f: h(sc, sr), g: 0 });

    // Allow cardinal directions and a "Wait" action (stay in place for 1 frame)
    const actions = [[1,0], [-1,0], [0,1], [0,-1], [0,0]]; 

    // Max search window depth to prevent infinite loops in locked states
    const maxTimeDepth = startTime + 150; 

    while (openSet.length > 0) {
      // Sort to simulate a Min-Heap extraction
      openSet.sort((a, b) => a.f - b.f);
      const current = openSet.shift();

      if (current.c === ec && current.r === er) {
        // Reconstruct trajectory path
        const path = [];
        let currKey = key(current.c, current.currentR ?? current.r, current.t);
        while (currKey) {
          const parts = currKey.split(',');
          path.push({ c: parseInt(parts[0]), r: parseInt(parts[1]), t: parseInt(parts[2]) });
          currKey = cameFrom.get(currKey);
        }
        return path.reverse();
      }

      if (current.t >= maxTimeDepth) continue;

      for (const [dc, dr] of actions) {
        const nc = current.c + dc;
        const nr = current.r + dr;
        const nt = current.t + 1;

        if (nc < 0 || nc >= this.cols || nr < 0 || nr >= this.rows) continue;
        
        // Static layout check
        if (staticGrid[nr * this.cols + nc] === 1) continue;

        // Dynamic reservation collision check
        if (this.reservationTable.has(this._makeKey(nc, nr, nt))) continue;

        const nextKey = key(nc, nr, nt);
        const edgeCost = (dc === 0 && dr === 0) ? 1.2 : 1.0; // Slices minor penalty for idling
        const tentativeG = gScore.get(key(current.c, current.r, current.t)) + edgeCost;

        if (!gScore.has(nextKey) || tentativeG < gScore.get(nextKey)) {
          cameFrom.set(nextKey, key(current.c, current.currentR ?? current.r, current.t));
          gScore.set(nextKey, tentativeG);
          
          openSet.push({
            c: nc,
            r: nr,
            t: nt,
            g: tentativeG,
            f: tentativeG + h(nc, nr)
          });
        }
      }
    }
    return null; // Route blocked dynamically
  }
}
