// 船只物理组件：帧率无关的定步长（fixed substep）积分驱动。
// 参照实现，用于说明设计，不含任何专有源码。
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "BuoyancyStrategy.h"
#include "WatercraftPhysicsComponent.generated.h"

/**
 * 负责在固定步长下推进船只刚体的浮力/阻尼积分。
 *
 * ！！！核心决策：物理必须与渲染帧率解耦。
 * 若每渲染帧只施加一次浮力，30fps 和 120fps 下船的起伏行为不同，
 * 更糟的是不同客户端无法就船的高度达成一致，直接影响网络同步。
 */
UCLASS(ClassGroup = (Vehicle), meta = (BlueprintSpawnableComponent))
class UWatercraftPhysicsComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	// 每帧调用：把本帧时间切成若干固定子步推进
	void TickPhysics(float DeltaTime);

	void SetBuoyancyStrategy(TSharedPtr<IBuoyancyStrategy> InStrategy) { Strategy = InStrategy; }

	// 当前没入比例，供表现层（水花、镜头晃动、阻尼缩放）复用
	float GetBuoyancyRate() const { return Val_LastBuoyancyRate; }

protected:
	// 单个固定子步内的积分：浮力 + 阻尼 + 水阻
	void IntegrateSubstep(float FixedDt);

#pragma region 调参
	// 物理仿真频率：固定 60Hz，保证任意渲染帧率下行为一致
	UPROPERTY(EditAnywhere, Category = "Watercraft|Physics")
	float Val_FixedStep = 1.0f / 60.0f;

	// 单帧最多追多少个子步，避免卡顿后"死亡螺旋"
	UPROPERTY(EditAnywhere, Category = "Watercraft|Physics")
	int32 Val_MaxSubsteps = 5;
#pragma endregion

private:
	// 时间累加器：渲染帧时间攒够一个固定步就推进一步
	float Accumulator = 0.0f;
	float Val_LastBuoyancyRate = 0.0f;

	TSharedPtr<IBuoyancyStrategy> Strategy;
	FRigidBodyState               Body;
};
