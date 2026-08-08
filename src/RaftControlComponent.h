// 驾驶 gameplay：登船 / 掌舵 / 两种动力（手动划桨 vs 升帆借风）/ 舵向转向 / 档位。
// 参照实现，说明操控玩法，不含任何专有源码。
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "RaftControlComponent.generated.h"

class APlayerCharacter;

// 动力模式：手动划桨，或升帆（半帆/全帆）借风前进
UENUM(BlueprintType)
enum class EDriveMode : uint8
{
	Manual UMETA(DisplayName = "手动划桨"),
	HalfSail UMETA(DisplayName = "半帆"),
	FullSail UMETA(DisplayName = "全帆"),
};

// 档位：倒 / 停 / 慢 / 半 / 全
UENUM()
enum class ERaftSpeed : uint8 { Back, Stop, Slow, Half, Full };

/**
 * 木筏操控组件。玩家从"登船区"上船，进入"驾驶区"即可掌舵。
 * ！！！输入走 Client_ 预测、Server_ 权威，关键量（舵值/档位/帆）复制给所有客户端。
 */
UCLASS(ClassGroup = (Vehicle), meta = (BlueprintSpawnableComponent))
class URaftControlComponent : public UActorComponent
{
	GENERATED_BODY()

public:
	// 登船区 / 驾驶区进出
	void OnEnterOrLeaveBoardingRegion(APlayerCharacter* Player, bool bEnter);
	void OnEnterOrLeaveDriverRegion(APlayerCharacter* Player, bool bEnter);

	// 手动划桨：开/关，前进或倒退
	UFUNCTION(Client) void Client_SwitchManualPower(bool bTurnOn, bool bForward);
	void Server_SwitchManualPower(bool bTurnOn, bool bForward);

	// 切换动力模式（手动 / 半帆 / 全帆）
	void SwitchDriveMode(EDriveMode Mode);

	// 转舵：一格一格打，或连续打
	UFUNCTION(Client) void Client_ChangeRudder(float RudderDelta);

	// 每个物理子步：把当前动力与舵向转成推力/力矩累加到刚体
	void ApplyControlForces(struct FRigidBodyState& Body, float SubstepDt) const;

protected:
	// 升帆借风：帆受的推力 = 风在前进方向的分量 * 帆面积
	FVector ComputeSailForce(const FVector& Wind) const;

#pragma region 调参
	UPROPERTY(EditAnywhere, Category = "Raft|Control") float Val_SteerVelFactor    = 0.2f;   // 转向力随航速缩放
	UPROPERTY(EditAnywhere, Category = "Raft|Control") float Val_SteerManualFactor = 60.f;   // 划桨时的转向权重
	UPROPERTY(EditAnywhere, Category = "Raft|Control") float Val_SailForceFactor   = 1000.f; // 帆推力系数
	UPROPERTY(EditAnywhere, Category = "Raft|Control") float Val_HalfSailFactor    = 0.5f;   // 半帆的帆面积比例
#pragma endregion

private:
	// 复制态：舵值 + 动力模式 + 帆面积
	UPROPERTY(ReplicatedUsing = OnRep_Rudder) float RudderValue = 0.f;
	UPROPERTY(Replicated) EDriveMode ShipMode = EDriveMode::Manual;
	UPROPERTY(Replicated) float      SailSize = 0.f;

	UFUNCTION() void OnRep_Rudder();

	int32 BoardingNum = 0;
};
