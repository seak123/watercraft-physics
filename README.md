# Watercraft Physics & Multiplayer Sync

A reference design for **player-built, physically-simulated watercraft** in an open-world
multiplayer game: how a craft floats and reacts to waves, how it stays frame-rate
independent, and how multiple players standing and walking on a *moving, rotating*
platform stay perfectly in sync across the network.

> **About this repository.** This is an original *clean-room* write-up of techniques I
> designed and shipped as the owner of a watercraft system in a commercial title. It
> contains **no proprietary source** — the code below is illustrative reference code I
> wrote to explain the approach and the engineering trade-offs I made. The engine's ocean
> *rendering* and *water-height sampling* are assumed as given (`SampleWaterHeight(pos)`);
> everything else — framework, buoyancy, controls, and networking — is the subject here.

---

## 1. Problem

A watercraft in this game is not a scripted animation — it is a rigid body floating on a
wave field, that:

- must **feel good** (stable, weighty, responsive) rather than floaty or twitchy,
- must behave **identically regardless of frame rate** (mobile 30fps vs 120fps),
- carries **multiple players** who move around freely on the deck, and
- must stay **consistent for every client** with bandwidth kept under control.

The three hard sub-problems, and the design for each, follow.

---

## 2. Architecture

```
                +-----------------------------+
                |        AWatercraft          |   Pawn; owns rigid body + control state
                |  (base class, extensible)   |
                +--------------+--------------+
                               |
        +----------------------+----------------------+
        |                      |                      |
+-------v--------+   +---------v---------+   +---------v---------+
| BuoyancyComp   |   |  ControlComp      |   |  NetSyncComp      |
| substep physics|   |  rudder / gears / |   |  movement replic. |
| 2 buoyancy     |   |  drive modes      |   |  + moving-base    |
| strategies     |   |                   |   |  player sync      |
+----------------+   +-------------------+   +-------------------+
        |
        | consumes
        v
  SampleWaterHeight(worldPos)   <- provided by engine ocean
```

Design rules I held to:

- **One base craft, extended by composition.** A raft, a larger boat, or a new drive mode
  are *extensions* — they reuse the same components and never fork the framework
  (open/closed). New behaviour is a new component or a new strategy, not an edit to the core.
- **Physics, control, and networking are separate components** (single responsibility) so I
  can retune buoyancy without touching replication, and vice-versa.
- **Buoyancy is a strategy behind one interface** — see §4 — so the rest of the craft neither
  knows nor cares which of the two implementations is running.

---

## 3. Frame-rate-independent physics (fixed substep)

The single most important correctness decision. Applying buoyancy/damping once per rendered
frame makes the craft behave differently at 30fps and 120fps — it visibly bobs differently
and, worse, desyncs between clients. I run the craft's physics on a **fixed-timestep
accumulator**, decoupled from the render frame:

```cpp
// Frame-rate independent integration. Physics always advances in fixed dt slices,
// no matter how long the render frame was. Guarantees identical behaviour at any FPS.
void UBuoyancyComponent::TickPhysics(float DeltaTime)
{
    constexpr float FixedStep = 1.0f / 60.0f;   // simulation runs at a stable 60 Hz
    constexpr int   MaxSteps  = 5;              // clamp to avoid a spiral of death

    Accumulator += DeltaTime;

    int Steps = 0;
    while (Accumulator >= FixedStep && Steps < MaxSteps)
    {
        IntegrateSubstep(FixedStep);            // buoyancy + damping + drag, once per slice
        Accumulator -= FixedStep;
        ++Steps;
    }

    if (Steps == MaxSteps)                       // we fell behind: drop the backlog
        Accumulator = 0.0f;
}
```

**Why this mattered in practice:** before this, the raft literally rode the waves
differently on different devices and clients could not agree on its height. Substepping made
the simulation deterministic w.r.t. frame time, which is also a *precondition* for the
network sync in §5 to be stable.

---

## 4. Buoyancy — two strategies, one interface

I shipped **two** buoyancy implementations behind a common interface, so each craft can trade
**accuracy against cost**. Both consume the same `SampleWaterHeight()` and apply forces
inside the fixed substep from §3.

```cpp
// Common interface — the craft depends on this, not on a concrete strategy.
class IBuoyancyStrategy
{
public:
    virtual ~IBuoyancyStrategy() = default;
    // Accumulate buoyant force/torque for this substep onto the rigid body.
    virtual void ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const = 0;
};
```

### 4a. Sample-point strategy — cheap and stable

Place N probe points on the hull; each submerged probe pushes up proportional to how deep it
is. O(N), no mesh work, extremely stable — the right default for simple craft like rafts.

```cpp
void FSamplePointBuoyancy::ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const
{
    for (const FVector& LocalProbe : Probes)
    {
        const FVector WorldProbe = Body.TransformPosition(LocalProbe);
        const float   WaterZ     = SampleWaterHeight(WorldProbe);
        const float   Depth      = WaterZ - WorldProbe.Z;      // >0 == submerged

        if (Depth > 0.0f)
        {
            // Force grows with submersion depth, clamped so a probe can't launch the hull.
            const float Force = FMath::Min(Depth * PerProbeStiffness, MaxProbeForce);
            Body.AddForceAtPosition(FVector::UpVector * Force, WorldProbe);
        }
    }
    ApplyLinearAndAngularDamping(Body, SubstepDt);            // kills endless bobbing
}
```

### 4b. Submerged-volume strategy — physically accurate

For craft where realistic tilt/righting matters, I clip the hull's triangles against the
water plane, compute the **actual submerged volume** and its **true center of buoyancy**, and
apply Archimedes' force there. Heavier, but the craft leans into turns and self-rights
believably.

```cpp
void FVolumeBuoyancy::ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const
{
    float   SubmergedVolume = 0.0f;
    FVector VolumeCentroid  = FVector::ZeroVector;

    for (const FHullTriangle& Tri : Hull)
    {
        FVector v0 = Body.TransformPosition(Tri.A);
        FVector v1 = Body.TransformPosition(Tri.B);
        FVector v2 = Body.TransformPosition(Tri.C);

        // Signed depths of each vertex below the (locally planar) water surface.
        const float d0 = SampleWaterHeight(v0) - v0.Z;
        const float d1 = SampleWaterHeight(v1) - v1.Z;
        const float d2 = SampleWaterHeight(v2) - v2.Z;

        // Clip the triangle to the underwater region and accumulate its prism volume
        // and centroid contribution. Fully/partially submerged tris handled separately.
        ClipAndAccumulate(v0, v1, v2, d0, d1, d2, SubmergedVolume, VolumeCentroid);
    }

    if (SubmergedVolume > KINDA_SMALL_NUMBER)
    {
        VolumeCentroid /= SubmergedVolume;                   // true center of buoyancy
        const float Force = WaterDensity * Gravity * SubmergedVolume;   // Archimedes
        Body.AddForceAtPosition(FVector::UpVector * Force, VolumeCentroid);
        ApplyLinearAndAngularDamping(Body, SubstepDt);
    }
}
```

**The design decision that reads well:** having *both* behind `IBuoyancyStrategy` meant I was
never boxed in — designers could pick "cheap and stable" for the starter raft and "accurate"
for hero vessels, per craft, with zero change to control or networking code.

---

## 5. The hard one: syncing players on a moving platform

A boat is a base that is itself moving and rotating; players walk around **on** it. If each
player's world position is replicated naively, tiny disagreements about *where the boat is*
compound with disagreements about *where the player is on it*, and you get jitter, rubber-
banding, and players sliding off the deck on remote screens.

**Approach: replicate player motion in the platform's local frame, plus client prediction +
reconciliation.**

```cpp
// A rider's position is stored & replicated RELATIVE to the boat, not in world space.
// Every client reconstructs world position from (authoritative boat transform) * (local).
FVector UNetSyncComponent::ResolveRiderWorldPosition(const FRiderState& Rider) const
{
    const FTransform BoatXform = GetBoatTransformForFrame(Rider.BaseFrameId);
    return BoatXform.TransformPosition(Rider.LocalOffset);
}

// Client predicts locally every frame for responsiveness, then reconciles when the
// authoritative state for that input frame arrives — correcting only if it drifted.
void UNetSyncComponent::OnServerRiderState(const FRiderState& Server)
{
    const FVector Predicted = PredictedHistory.Get(Server.BaseFrameId);
    const float   Error     = FVector::Dist(Predicted, Server.LocalOffset);

    if (Error > ReconcileThreshold)
    {
        LocalOffset = Server.LocalOffset;          // snap the base
        ReplayPendingInputs(Server.BaseFrameId);   // re-apply unacked inputs on top
    }
    // else: prediction was good enough — smooth, no visible correction.
}
```

Bandwidth control that made it shippable on weak networks:

- Rider deltas are **quantized** and sent only when they exceed a movement threshold.
- Boat transform updates are **rate-limited and prioritized**; on detected packet loss the
  per-tick update budget shrinks so the channel degrades gracefully instead of collapsing.
- Because physics is deterministic per substep (§3), clients can *predict* the boat between
  updates, so I can send fewer of them without visible cost.

---

## 6. Results

- Craft behave **identically across frame rates and devices** — the substep rewrite fixed a
  whole class of "bobs differently on mobile" and cross-client height-disagreement bugs.
- **Two buoyancy models** let the team dial realism-vs-performance per craft instead of being
  stuck with one compromise.
- Multiple players **sail together on one boat with no jitter or rubber-banding**, and it
  holds up under packet loss — the part that usually breaks first in this genre.

## 7. What this demonstrates

Ownership of a **physics-and-network-heavy** gameplay system end to end; comfort with rigid-
body simulation, numerical stability, and multiplayer netcode; and the engineering judgment
to build *two* solutions behind one interface when a single one wouldn't serve every case.
