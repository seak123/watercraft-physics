// 两种浮力策略的实现。参照实现，不含任何专有源码。
#include "BuoyancyStrategy.h"

namespace
{
	// 线性 + 角阻尼：干掉船只没完没了的上下晃动
	void ApplyDamping(FRigidBodyState& /*Body*/, float /*SubstepDt*/)
	{
		// 实际项目里在这里对刚体的线速度/角速度施加阻尼，
		// 阻尼系数随没入比例（BuoyancyRate）缩放，越沉阻尼越大。
	}
}

#pragma region 采样点浮力
void FSamplePointBuoyancy::ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const
{
	for (const FVector& LocalProbe : Probes)
	{
		const FVector WorldProbe = Body.TransformPosition(LocalProbe);
		const float   WaterZ     = SampleWaterHeight(WorldProbe);
		const float   Depth      = WaterZ - WorldProbe.Z; // >0 表示没入水下

		if (Depth > 0.0f)
		{
			// 力随没入深度增长，并做上限钳制
			const float Force = FMath::Min(Depth * Val_PerProbeStiffness, Val_MaxProbeForce);
			Body.AddForceAtPosition(FVector::UpVector * Force, WorldProbe);
		}
	}

	ApplyDamping(Body, SubstepDt);
}
#pragma endregion

#pragma region 三角形浮心浮力
void FVolumeBuoyancy::ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const
{
	float   SubmergedVolume = 0.0f;
	FVector VolumeCentroid  = FVector::ZeroVector;

	for (const FHullTriangle& Tri : Hull)
	{
		const FVector V0 = Body.TransformPosition(Tri.A);
		const FVector V1 = Body.TransformPosition(Tri.B);
		const FVector V2 = Body.TransformPosition(Tri.C);

		// 三个顶点相对（局部近似平面的）水面的有符号深度
		const float D0 = SampleWaterHeight(V0) - V0.Z;
		const float D1 = SampleWaterHeight(V1) - V1.Z;
		const float D2 = SampleWaterHeight(V2) - V2.Z;

		ClipAndAccumulate(V0, V1, V2, D0, D1, D2, SubmergedVolume, VolumeCentroid);
	}

	if (SubmergedVolume > KINDA_SMALL_NUMBER)
	{
		VolumeCentroid /= SubmergedVolume; // 真实浮心
		const float Force = Val_WaterDensity * Val_Gravity * SubmergedVolume; // 阿基米德
		Body.AddForceAtPosition(FVector::UpVector * Force, VolumeCentroid);
		ApplyDamping(Body, SubstepDt);
	}
}

void FVolumeBuoyancy::ClipAndAccumulate(const FVector& V0, const FVector& V1, const FVector& V2,
                                        float D0, float D1, float D2,
                                        float& OutVolume, FVector& OutCentroid) const
{
	// 全部在水面之上：不贡献浮力
	if (D0 <= 0.f && D1 <= 0.f && D2 <= 0.f)
	{
		return;
	}

	// 说明版：三角形完全没入时，直接用四面体有符号体积法累加体积与体积加权质心。
	// 部分没入（一个/两个顶点在水下）时，先沿水面把三角形裁成水下多边形再累加。
	// 这里给出"完全没入"的核心累加，裁剪分支同理按裁出的子三角形处理。
	if (D0 > 0.f && D1 > 0.f && D2 > 0.f)
	{
		// 以原点为参考的四面体有符号体积
		const float Vol = FVector::DotProduct(V0, FVector::CrossProduct(V1, V2)) / 6.0f;
		OutVolume   += Vol;
		OutCentroid += Vol * (V0 + V1 + V2) * 0.25f; // 四面体质心（含原点顶点）
	}
	// else: 部分没入分支 —— 求水面与两条边的交点，按裁出的水下三角形调用同样的累加逻辑
}
#pragma endregion
