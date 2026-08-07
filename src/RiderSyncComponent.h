// 移动平台上的乘员同步：把乘员运动放到"船的局部坐标系"里复制，
// 叠加客户端预测 + 校正回滚。参照实现，不含任何专有源码。
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RiderSyncComponent.generated.h"

/**
 * 一个乘员的同步状态。
 * ！！！关键：LocalOffset 存的是相对"船"的局部坐标，而不是世界坐标。
 * 世界坐标 = 权威船体变换 * LocalOffset，由每个客户端各自重建。
 * 这样"船在哪"和"人在船上哪"两处误差不会叠加放大。
 */
USTRUCT()
struct FRiderState
{
	GENERATED_BODY()

	// 相对船体的局部位置（复制）
	UPROPERTY()
	FVector_NetQuantize LocalOffset = FVector::ZeroVector;

	// 该状态对应的输入帧号，用于客户端预测对账
	UPROPERTY()
	uint32 BaseFrameId = 0;
};

UCLASS(ClassGroup = (Vehicle), meta = (BlueprintSpawnableComponent))
class URiderSyncComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	// 由权威船体变换 * 乘员局部偏移，重建世界坐标
	FVector ResolveRiderWorldPosition(const FRiderState& Rider) const;

	// 收到服务器权威状态：预测足够准就不动，漂移超阈值才回滚重放
	void OnServerRiderState(const FRiderState& Server);

protected:
	// 取指定输入帧对应的（插值/外推后）船体变换
	FTransform GetBoatTransformForFrame(uint32 FrameId) const;

	// 从某帧起重放尚未确认的本地输入，叠加到权威状态之上
	void ReplayPendingInputs(uint32 FromFrameId);

#pragma region 调参 / 带宽控制
	// 预测与权威的误差超过该阈值才做一次校正（cm）
	UPROPERTY(EditAnywhere, Category = "Rider|Sync")
	float Val_ReconcileThreshold = 8.0f;

	// 乘员局部位移超过该量才发送一次增量，进一步省带宽（cm）
	UPROPERTY(EditAnywhere, Category = "Rider|Sync")
	float Val_SendThreshold = 2.0f;
#pragma endregion

private:
	FVector LocalOffset = FVector::ZeroVector;
};
