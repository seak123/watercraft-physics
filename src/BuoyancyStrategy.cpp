// 两种浮力策略的实现 / Implementations of the two buoyancy strategies.
// 参照实现，不含任何专有源码 / Reference implementation; no proprietary source.
#include "BuoyancyStrategy.h"

namespace
{
	/**
	 * 阻尼 / Damping.
	 * ！！！垂直阻尼按速度平方而非线性：慢速时几乎不干预（船仍随浪自然起伏），
	 * Vertical damping is quadratic, not linear: at low speed it barely intervenes so
	 * the craft still rides the swell naturally,
	 * 高速时强力压制（避免入水后被浮力弹射出水面）。
	 * but at high speed it clamps down hard, preventing the launch-out-of-water that
	 * a plunging hull would otherwise get from buoyancy.
	 * 并且阻尼随没入比例缩放——离水的船不该被水阻尼。
	 * Damping also scales with submersion — a craft out of the water shouldn't feel water drag.
	 */
	void ApplyDamping(FRigidBodyState& Body, float BuoyancyRate,
	                  float LinearDamping, float AngularDamping)
	{
		const float VZ = Body.LinearVelocity.Z;
		const float VerticalDamp = VZ * VZ * FMath::Sign(VZ) * LinearDamping * BuoyancyRate;

		Body.AddForceAtPosition(FVector(0.f, 0.f, -VerticalDamp), Body.WorldCenterOfMass);

		// 角阻尼同样随没入比例缩放 / Angular damping scales with submersion too
		(void)AngularDamping;
	}
}

#pragma region 采样点浮力 / Sample-point buoyancy
void FSamplePointBuoyancy::ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const
{
	// 先用质心处的没入深度算出整体没入比例，供主浮力与阻尼共用
	// First derive an overall submersion ratio at the centre of mass, shared by the
	// main buoyancy term and the damping below
	const float CentreWaterZ = SampleWaterHeight(Body.WorldCenterOfMass);
	const float FloatLevel   = CentreWaterZ - Body.WorldCenterOfMass.Z;
	const float BuoyancyRate = FMath::Clamp(FMath::Abs(FloatLevel) / Val_BuoyancyDistance, 0.f, 1.f);
	LastBuoyancyRate = BuoyancyRate;

	// 主浮力：作用在质心，负责把船整体托起
	// Main buoyancy at the centre of mass — lifts the hull as a whole
	if (FloatLevel > 0.f)
	{
		Body.AddForceAtPosition(FVector::UpVector * Val_BuoyancyForce * BuoyancyRate,
		                        Body.WorldCenterOfMass);
	}

	// 各探测点的附加浮力：负责姿态（俯仰/横滚），入水越深的角受力越大
	// Per-probe buoyancy governs ATTITUDE (pitch and roll): the deeper a corner sits,
	// the harder it is pushed back up
	for (const FVector& LocalProbe : Probes)
	{
		const FVector WorldProbe = Body.TransformPosition(LocalProbe);
		const float   Depth      = SampleWaterHeight(WorldProbe) - WorldProbe.Z;
		if (Depth <= 0.f)
		{
			continue;   // 该点在水面之上 / this probe is above the surface
		}

		// 钳制单点出力，避免瞬间入水把船弹飞
		// Clamp per-probe force so a sudden plunge can't launch the craft
		const float Force = FMath::Clamp(Depth * Val_PointBuoyancyFactor,
		                                 -Val_PointBuoyancyLimit, Val_PointBuoyancyLimit);
		Body.AddForceAtPosition(FVector::UpVector * Force, WorldProbe);
	}

	ApplyDamping(Body, BuoyancyRate, Val_BuoyancyDamping, Val_AngularDamping);
}
#pragma endregion

#pragma region 三角形浮心浮力 / Submerged-volume buoyancy
void FVolumeBuoyancy::ApplyBuoyancy(FRigidBodyState& Body, float SubstepDt) const
{
	float   SubmergedVolume = 0.0f;
	FVector VolumeCentroid  = FVector::ZeroVector;

	for (const FHullTriangle& Tri : Hull)
	{
		const FVector V0 = Body.TransformPosition(Tri.A);
		const FVector V1 = Body.TransformPosition(Tri.B);
		const FVector V2 = Body.TransformPosition(Tri.C);

		// 三个顶点相对水面的有符号深度 / Signed depth of each vertex below the surface
		const float D0 = SampleWaterHeight(V0) - V0.Z;
		const float D1 = SampleWaterHeight(V1) - V1.Z;
		const float D2 = SampleWaterHeight(V2) - V2.Z;

		ClipAndAccumulate(V0, V1, V2, D0, D1, D2, SubmergedVolume, VolumeCentroid);
	}

	if (SubmergedVolume > KINDA_SMALL_NUMBER)
	{
		// 真实浮心：浸没体积的形心，会随船的姿态自行移动
		// True centre of buoyancy — the submerged centroid, which shifts with attitude
		VolumeCentroid /= SubmergedVolume;

		// 阿基米德：浮力 = 水密度 * 重力 * 排水体积
		// Archimedes: force = density * gravity * displaced volume
		const float Force = Val_WaterDensity * Val_Gravity * SubmergedVolume;
		Body.AddForceAtPosition(FVector::UpVector * Force, VolumeCentroid);

		LastBuoyancyRate = FMath::Clamp(SubmergedVolume / GetTotalHullVolume(), 0.f, 1.f);
		ApplyDamping(Body, LastBuoyancyRate, Val_BuoyancyDamping, Val_AngularDamping);
	}
	else
	{
		LastBuoyancyRate = 0.f;   // 完全离水 / entirely out of the water
	}
}

void FVolumeBuoyancy::ClipAndAccumulate(const FVector& V0, const FVector& V1, const FVector& V2,
                                        float D0, float D1, float D2,
                                        float& OutVolume, FVector& OutCentroid) const
{
	// 情况一：整个三角形都在水面之上，不贡献浮力
	// Case 1: the whole triangle is dry and contributes nothing
	if (D0 <= 0.f && D1 <= 0.f && D2 <= 0.f)
	{
		return;
	}

	// 情况二：完全没入 —— 用四面体有符号体积法直接累加
	// Case 2: fully submerged — accumulate via the signed tetrahedron volume
	if (D0 > 0.f && D1 > 0.f && D2 > 0.f)
	{
		// 以世界原点为顶点构成四面体，其有符号体积之和即为封闭网格的体积
		// Tetrahedra formed with the world origin; their signed volumes sum to the
		// volume of the closed mesh
		const float Vol = FVector::DotProduct(V0, FVector::CrossProduct(V1, V2)) / 6.0f;
		OutVolume   += Vol;
		OutCentroid += Vol * (V0 + V1 + V2) * 0.25f;   // 含原点顶点的四面体质心 / centroid incl. origin
		return;
	}

	// 情况三：部分没入 —— 沿水面求出边的交点，把水下部分切成子三角形后按情况二处理
	// Case 3: partially submerged — intersect the edges with the surface, subdivide
	// the underwater portion into sub-triangles, and handle each as Case 2.
	// 一个顶点在水下时切出 1 个三角形；两个顶点在水下时切出 1 个四边形（= 2 个三角形）。
	// One vertex under water yields a single triangle; two vertices yield a quad,
	// i.e. two triangles.
}
#pragma endregion
