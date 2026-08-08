// 浮力策略 / Buoyancy strategies
//
// 两种浮力实现藏在同一个接口后面，按船只类型选择：
// 采样点法（便宜、极稳）与三角形浮心体积法（物理精确）。
// Two implementations behind one interface, chosen per craft: a sample-point method
// (cheap, extremely stable) and a submerged-volume method (physically accurate).
//
// 参照实现，不含任何专有源码 / Reference implementation; no proprietary source.
#pragma once

#include "CoreMinimal.h"

// 极简刚体外观封装，实际项目里映射到引擎物理刚体
// Minimal rigid-body facade; maps onto the engine's physics body in practice
struct FRigidBodyState
{
	FTransform Xform;
	FVector    LinearVelocity  = FVector::ZeroVector;
	FVector    AngularVelocity = FVector::ZeroVector;
	FVector    WorldCenterOfMass = FVector::ZeroVector;

	void    AddForceAtPosition(const FVector& Force, const FVector& WorldPos);
	FVector TransformPosition(const FVector& Local) const { return Xform.TransformPosition(Local); }
};

// 由海洋波场提供 / Provided by the ocean wave field (see OceanWaveField.h)
float SampleWaterHeight(const FVector& WorldPos);

/**
 * 浮力策略接口 / Buoyancy strategy interface.
 * ！！！船只本体只依赖这个抽象，永远不知道底下跑的是哪种实现。
 * The craft depends only on this abstraction and never learns which implementation runs.
 */
class IBuoyancyStrategy
{
public:
	virtual ~IBuoyancyStrategy() = default;

	// 在一个固定物理子步内，把浮力与阻尼累加到刚体上
	// Accumulate buoyancy and damping onto the body within one fixed substep
	virtual void ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const = 0;

	// 当前没入比例 [0,1]，表现层复用（水花、镜头晃动、阻尼缩放）
	// Current submersion ratio [0,1], reused by VFX, camera shake and damping scaling
	virtual float GetBuoyancyRate() const = 0;
};

#pragma region 采样点浮力 / Sample-point buoyancy
/**
 * 采样点法 / Sample-point method.
 *
 * 在船体上布若干"探测点"，每个没入水面的点按没入深度施加向上的力。
 * Probe points are distributed over the hull; each submerged probe pushes up in
 * proportion to how deep it sits.
 *
 * ！！！为什么它足够好：木筏是个近似平板，四角探测点已经能正确表达俯仰与横滚——
 * Why it suffices: a raft is essentially a flat slab, and four corner probes already
 * express pitch and roll correctly —
 * 一角入水更深，那一角受力更大，船自然回正。
 * a corner sitting deeper receives more force, so the craft self-levels naturally.
 * 代价是 O(N) 且完全不碰网格，非常适合大量同时存在的简单船只。
 * The cost is O(N) with no mesh work at all, ideal for many simple craft at once.
 */
class FSamplePointBuoyancy final : public IBuoyancyStrategy
{
public:
	// 船体局部空间的探测点（木筏通常取四角）/ Probes in hull local space (typically the four corners)
	TArray<FVector> Probes;

	virtual void ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const override;
	virtual float GetBuoyancyRate() const override { return LastBuoyancyRate; }

#pragma region 调参 / Tuning
	// 主浮力大小 / Main buoyancy magnitude
	float Val_BuoyancyForce = 0.5f;

	// 没入多深算"完全没入"，用于归一化没入比例
	// Depth at which the hull counts as fully submerged; normalizes the buoyancy rate
	float Val_BuoyancyDistance = 100.f;

	// 单点浮力系数与上限：上限很关键，否则某个点瞬间入水会把船弹飞
	// Per-probe factor and clamp — the clamp matters: without it a probe plunging
	// underwater launches the whole craft
	float Val_PointBuoyancyFactor = 1.0f;
	float Val_PointBuoyancyLimit  = 1.0f;

	// 垂直阻尼：按速度平方衰减，压掉上下无止境的弹跳
	// Vertical damping, quadratic in velocity — kills endless bobbing
	float Val_BuoyancyDamping = 1.0f;

	// 角阻尼：压掉船的持续摇摆 / Angular damping — settles persistent rocking
	float Val_AngularDamping = 1.0f;
#pragma endregion

private:
	mutable float LastBuoyancyRate = 0.f;
};
#pragma endregion

#pragma region 三角形浮心浮力 / Submerged-volume buoyancy
/**
 * 浸没体积法 / Submerged-volume method.
 *
 * 把船体三角形按水面裁剪，算出真实浸没体积与真实浮心，在浮心处施加阿基米德浮力。
 * Hull triangles are clipped against the water surface to obtain the true submerged
 * volume and its true centre of buoyancy; Archimedes' force is applied there.
 *
 * ！！！与采样点法的本质差别：浮心会随姿态自己移动。
 * The essential difference from sample points: the centre of buoyancy MOVES on its own.
 * 船一侧倾，浸没体积的形心就偏向那一侧，产生真实的回正力矩——
 * As the craft heels, the submerged centroid shifts to that side, producing a genuine
 * righting moment —
 * 转弯自然侧倾、翻覆后能自行复正，这些都不需要额外写规则。
 * so banking in turns and self-righting emerge without any special-case code.
 */
class FVolumeBuoyancy final : public IBuoyancyStrategy
{
public:
	struct FHullTriangle { FVector A, B, C; };

	TArray<FHullTriangle> Hull;

	virtual void ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const override;
	virtual float GetBuoyancyRate() const override { return LastBuoyancyRate; }

	// 水密度与重力，浮力 = rho * g * V / Water density and gravity; force = rho * g * V
	float Val_WaterDensity = 1000.0f;
	float Val_Gravity      = 9.81f;

	float Val_BuoyancyDamping = 1.0f;
	float Val_AngularDamping  = 1.0f;

private:
	/**
	 * 把一个三角形按水面裁剪，累加浸没体积与体积加权质心。
	 * Clip one triangle against the surface, accumulating submerged volume and the
	 * volume-weighted centroid.
	 * 三种情况：全在水下 / 全在水上 / 部分没入（需沿水面求交点再切分）。
	 * Three cases: fully submerged, fully dry, or partially submerged (which requires
	 * intersecting the edges with the surface and subdividing).
	 */
	void ClipAndAccumulate(const FVector& V0, const FVector& V1, const FVector& V2,
	                       float D0, float D1, float D2,
	                       float& OutVolume, FVector& OutCentroid) const;

	mutable float LastBuoyancyRate = 0.f;
};
#pragma endregion
