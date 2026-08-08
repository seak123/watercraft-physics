// 海面波场 / Ocean wave field
//
// 物理侧需要在任意世界坐标问出"这一刻水面有多高"。渲染用的海面由引擎负责，
// 但物理不能去读渲染结果——必须有一份 CPU 侧、可在任意点、任意时刻求值的解析波场，
// 且必须与渲染视觉一致，否则船会浮在看得见的水面之外。
// The physics layer must answer "how high is the water here, right now" at any world
// position. The ocean's *rendering* is the engine's job, but physics cannot read back
// the rendered surface — it needs an analytic, CPU-side wave field evaluable at any
// point and time, that matches the visuals; otherwise the boat floats visibly off the water.
//
// 参照实现，说明机制，不含任何专有源码。
// Reference implementation illustrating the mechanic; no proprietary source.
#pragma once

#include "CoreMinimal.h"
#include "OceanWaveField.generated.h"

/**
 * 海面分块 / One ocean tile.
 *
 * ！！！整片海不能当成一个无限平面：近岸要变浅、浪要变小，深海才有大浪。
 * The ocean is not one infinite plane: waves must shrink toward the shore and only
 * reach full height in deep water.
 * 因此按网格分块，每块缓存四角水深，块内对水深做双线性插值，再用水深调制浪高。
 * So it is tiled; each tile caches its four corner depths, interpolates depth
 * bilinearly inside the tile, and modulates wave height by that depth.
 */
USTRUCT()
struct FOceanTile
{
	GENERATED_BODY()

	void Init(int32 InCoordX, int32 InCoordY, float InBaseLevel);

	/**
	 * 求该点的海面位置 / Evaluate the sea surface at a point.
	 * @param Wind      风向与风力（xy=方向，z/w=强度参数）/ wind direction and strength
	 * @param OceanTime 全局海面时间，保证所有客户端同相位 / global ocean time, keeping all clients in phase
	 * @param WaveScale 浪高整体缩放（天气/剧情可压浪）/ global wave scale — weather or scripted calm
	 */
	FVector GetSurface(const FVector& Position, const FVector4& Wind,
	                   float OceanTime, float WaveScale = 1.f) const;

private:
	/**
	 * 单列波 / A single wave train.
	 * 用摆线（trochoidal / Gerstner）而不是纯正弦：波峰更尖、波谷更平，接近真实海浪；
	 * Uses a trochoidal (Gerstner) profile rather than a plain sine — sharper crests
	 * and flatter troughs, much closer to real swell;
	 * Sharpness 控制这个"尖锐度"。
	 * Sharpness controls exactly that crest sharpening.
	 */
	float EvaluateWave(const FVector& WorldPos, float Time, float Speed, float Length,
	                   float Height, const FVector2D& Dir2D, float Sharpness) const;

	// 叠加多列不同方向/波长的波 / Sum several wave trains of differing direction and length
	float EvaluateWaveSum(const FVector& WorldPos, float Depth, const FVector4& Wind,
	                      float Time, float WaveScale) const;

	// 块内水深（双线性插值四角）/ Depth inside the tile, bilinear over the four corners
	float GetDepth(const FVector& Position) const;

	int32 CoordX = -1;
	int32 CoordY = -1;
	float BaseLevel = 0.f;

	// 四角水深与归一化水深 / Corner depths, raw and normalized
	TArray<float> CornerDepths;
	TArray<float> NormalizedDepths;
};

/**
 * 海洋管理器 / Ocean manager.
 *
 * ！！！OceanTime 必须是全局同步量，而不是各端各自累加的本地时间。
 * OceanTime must be a globally synchronized value, never each client's own
 * accumulated local time.
 * 否则两个客户端的浪相位会缓慢漂移，同一条船在两边浮在不同高度——
 * Otherwise wave phase drifts apart between clients and the same boat sits at
 * different heights on each screen —
 * 而船的同步正建立在"大家看到同一个水面"这个前提上。
 * and the whole boat-sync design rests on everyone seeing the same water.
 */
UCLASS()
class UOceanManager : public UGameInstanceSubsystem, public FTickableGameObject
{
	GENERATED_BODY()

public:
	virtual void Tick(float DeltaTime) override;

	// 物理层唯一的对外入口 / The single entry point the physics layer uses
	float SampleWaterHeight(const FVector& WorldPos) const;

	// 当前风（驱动帆的推力，也驱动浪）/ Current wind — drives sail thrust and the waves alike
	FVector4 GetWind() const { return Wind; }

protected:
	// 按世界坐标取到所属分块 / Resolve the tile owning a world position
	const FOceanTile* FindTile(const FVector& WorldPos) const;

private:
	// 全局同步的海面时间 / Globally synchronized ocean time
	float OceanTime = 0.f;

	// 风：方向 + 强度 / Wind: direction plus strength
	FVector4 Wind = FVector4(1.f, 0.f, 0.f, 0.f);

	// 浪高整体缩放 / Global wave scale
	float WaveScale = 1.f;

	TMap<FIntPoint, FOceanTile> Tiles;
};
