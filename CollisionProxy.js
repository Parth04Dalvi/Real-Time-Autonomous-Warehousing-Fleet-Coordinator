/**
 * NEXUS Reactive Collision & Local Steering Proxy
 * Implements low-overhead localized separation forces using the engine's Spatial Hash
 */
export class CollisionProxy {
  constructor(spatialHash, safetyRadius = 18) {
    this.spatialHash = spatialHash;
    this.safetyRadius = safetyRadius; // px distance thresholds
  }

  /**
   * Intercepts natural linear steps, blending a local avoidance vector
   * @param {Robot} subject 
   * @param {number} preferredVx Target X velocity step
   * @param {number} preferredVy Target Y velocity step
   * @returns {{vx: number, vy: number}} Modified velocity vector
   */
  computeSafeVelocity(subject, preferredVx, preferredVy) {
    const nearbyAgents = this.spatialHash.nearby(subject.px, subject.py, this.safetyRadius * 2);
    
    let separationX = 0;
    let separationY = 0;
    let overlappingCount = 0;

    nearbyAgents.forEach(other => {
      if (other.id === subject.id) return;

      const dx = subject.px - other.px;
      const dy = subject.py - other.py;
      const distance = Math.sqrt(dx * dx + dy * dy);

      if (distance < this.safetyRadius) {
        overlappingCount++;
        // Calculate force inversely proportional to proximity distance threshold
        const strength = (this.safetyRadius - distance) / this.safetyRadius;
        separationX += (dx / (distance || 1)) * strength * 45;
        separationY += (dy / (distance || 1)) * strength * 45;
      }
    });

    if (overlappingCount > 0) {
      // Blend normal vector paths with steering separation correction forces
      const blendedVx = preferredVx + separationX;
      const blendedVy = preferredVy + separationY;
      
      // Re-normalize speeds back within uniform structural limits
      const totalMag = Math.sqrt(blendedVx * blendedVx + blendedVy * blendedVy);
      const targetMag = Math.sqrt(preferredVx * preferredVx + preferredVy * preferredVy);
      
      if (totalMag > 0) {
        return {
          vx: (blendedVx / totalMag) * targetMag,
          vy: (blendedVy / totalMag) * targetMag,
          collided: true
        };
      }
    }

    return { vx: preferredVx, vy: preferredVy, collided: false };
  }
}
