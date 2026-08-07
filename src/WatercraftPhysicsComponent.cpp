// 帧率无关的定步长积分实现。参照实现，不含任何专有源码。
#include "WatercraftPhysicsComponent.h"

void UWatercraftPhysicsComponent::TickPhysics(float DeltaTime)
{
	Accumulator += DeltaTime;

	// 物理只在固定步长下推进，无论本帧渲染耗时多长 —— 保证任意帧率下行为一致
	int32 Steps = 0;
	while (Accumulator >= Val_FixedStep && Steps < Val_MaxSubsteps)
	{
		IntegrateSubstep(Val_FixedStep);
		Accumulator -= Val_FixedStep;
		++Steps;
	}

	// 落后太多（卡顿）时直接丢掉积压，避免"死亡螺旋"越追越卡
	if (Steps >= Val_MaxSubsteps)
	{
		Accumulator = 0.0f;
	}
}

void UWatercraftPhysicsComponent::IntegrateSubstep(float FixedDt)
{
	if (!Strategy.IsValid())
	{
		return;
	}

	// 浮力策略（采样点 / 三角形浮心）在这一步内把力累加到刚体
	Strategy->ApplyBuoyancy(Body, FixedDt);

	// 此处再叠加：水流阻力、前进推力、舵角产生的偏航力矩等。
	// 表现层需要的"没入比例"在策略内部算好后回填 Val_LastBuoyancyRate。
}
