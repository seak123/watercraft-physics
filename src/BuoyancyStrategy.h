// 浮力策略接口 + 两种实现（采样点 / 三角形浮心体积）
// 参照实现，用于说明设计思路，不含任何专有源码。
#pragma once

#include "CoreMinimal.h"

// 极简刚体外观封装，实际项目里映射到引擎物理刚体（PxRigidBody / Chaos）
struct FRigidBodyState
{
	FTransform Xform;

	void    AddForceAtPosition(const FVector& Force, const FVector& WorldPos);
	FVector TransformPosition(const FVector& Local) const { return Xform.TransformPosition(Local); }
};

// 由引擎海洋提供：给定世界坐标，返回该点水面高度。整个物理层只依赖这一个外部输入。
float SampleWaterHeight(const FVector& WorldPos);

/**
 * 浮力策略接口
 * ！！！船只本体只依赖这个抽象，不关心底层跑的是哪种浮力实现（开闭原则）
 */
class IBuoyancyStrategy
{
public:
	virtual ~IBuoyancyStrategy() = default;

	// 在一个物理子步内，把浮力/阻尼累加到刚体上
	virtual void ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const = 0;
};

#pragma region 采样点浮力（便宜、稳定）
/**
 * 采样点法：在船体上布若干探测点，每个没入水面的点按没入深度施加向上的力。
 * O(N)，无网格计算，极稳定 —— 简单船只（如木筏）的默认选择。
 */
class FSamplePointBuoyancy final : public IBuoyancyStrategy
{
public:
	// 船体局部空间的探测点
	TArray<FVector> Probes;

	// 单点浮力刚度：力随没入深度线性增长的系数
	float Val_PerProbeStiffness = 1.0f;
	// 单点浮力上限：避免某个点把船"弹飞"
	float Val_MaxProbeForce = 1.0e5f;

	virtual void ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const override;
};
#pragma endregion

#pragma region 三角形浮心浮力（物理精确）
/**
 * 浸没体积法：把船体三角形按水面裁剪，算出真实浸没体积和真实浮心，
 * 在浮心处施加阿基米德浮力。开销更大，但转弯会自然侧倾、能真实自复正。
 */
class FVolumeBuoyancy final : public IBuoyancyStrategy
{
public:
	struct FHullTriangle { FVector A, B, C; };

	TArray<FHullTriangle> Hull;

	// 水密度 * 重力，预乘好省一次乘法
	float Val_WaterDensity = 1000.0f;
	float Val_Gravity = 9.81f;

	virtual void ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const override;

private:
	// 把一个三角形按水面裁剪，累加其浸没部分的体积与体积加权质心
	void ClipAndAccumulate(const FVector& V0, const FVector& V1, const FVector& V2,
	                       float D0, float D1, float D2,
	                       float& OutVolume, FVector& OutCentroid) const;
};
#pragma endregion
