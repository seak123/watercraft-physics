// Watercraft Physics — illustrative clean-room reference (no proprietary source).
// Buoyancy strategy interface + the two implementations described in the README.
#pragma once

#include "CoreMinimal.h"

// Minimal rigid-body facade used by the strategies; maps onto the engine's physics body.
struct FRigidBodyState
{
    FTransform Xform;
    void  AddForceAtPosition(const FVector& Force, const FVector& WorldPos);
    FVector TransformPosition(const FVector& Local) const { return Xform.TransformPosition(Local); }
};

// Provided by the engine ocean; the whole physics layer consumes only this.
float SampleWaterHeight(const FVector& WorldPos);

// Strategy interface — the craft depends on this abstraction, never on a concrete class.
class IBuoyancyStrategy
{
public:
    virtual ~IBuoyancyStrategy() = default;
    virtual void ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const = 0;
};

// (a) Cheap, stable — force per submerged probe point. Good default for simple craft.
class FSamplePointBuoyancy final : public IBuoyancyStrategy
{
public:
    TArray<FVector> Probes;
    float PerProbeStiffness = 1.0f;
    float MaxProbeForce     = 1.0e5f;
    virtual void ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const override;
};

// (b) Physically accurate — submerged volume + true center of buoyancy (Archimedes).
class FVolumeBuoyancy final : public IBuoyancyStrategy
{
public:
    struct FHullTriangle { FVector A, B, C; };
    TArray<FHullTriangle> Hull;
    float WaterDensity = 1000.0f;
    float Gravity      = 9.81f;
    virtual void ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const override;
};
