// Copyright LoogLong. All Rights Reserved.
#pragma once
#include "CoreMinimal.h"
#include "BoneContainer.h"
#include "BonePose.h"
#include "BoxTypes.h"
#include "BoneControllers/AnimNode_SkeletalControlBase.h"
#include "PhysicsEngine/TaperedCapsuleElem.h"
#include "Math/IntVector.h"
#include "Curves/CurveFloat.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "Engine/DataAsset.h"
#include "AnimNode_LoogPhysics.generated.h"

LOOGPHYSICS_API DECLARE_LOG_CATEGORY_EXTERN(LogLoogPhysics, Log, All);

namespace PhysicsDefine
{
	constexpr float Epsilon = UE_SMALL_NUMBER;

	constexpr float TetherCompressionStiffness = 1.f;

	constexpr float TetherCompressionVelocityAttenuation = 0.7f;

	constexpr float TetherStretchStiffness = 1.f;

	constexpr float TetherStretchVelocityAttenuation = 0.7f;

	constexpr float TetherStiffnessWidth = 0.3f;

	constexpr float DistanceHorizontalStiffness = 0.5f;

	constexpr float DistanceVelocityAttenuation = 0.3f;

	constexpr int32 AngleLimitIteration = 3;

	constexpr float AngleLimitAttenuation = 0.9f;

	constexpr  float TetherStretchLimit = 0.03f;

	constexpr  float ColliderCollisionDynamicFrictionRatio = 1.f;

	constexpr  float ColliderCollisionStaticFrictionRatio = 1.f;

	constexpr  float FrictionDampingRate = 0.6f;

	constexpr float MaxDistanceRatioFuturePrediction = 1.3f;

	constexpr float TriangleBendingMaxAngle = 120.f;

	constexpr float VolumeScale = 1000.f;
};

UENUM()
enum class ELoogPhysicsTriangleSignFlag
{
	Negative,
	Positive,
	Volume,
	Max UMETA(Hidden)
};


USTRUCT()
struct FTrianglePair
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(VisibleAnywhere, Category = "LoogPhysics")
	int32 V0;

	UPROPERTY(VisibleAnywhere, Category = "LoogPhysics")
	int32 V1;

	UPROPERTY(VisibleAnywhere, Category = "LoogPhysics")
	int32 V2;

	UPROPERTY(VisibleAnywhere, Category = "LoogPhysics")
	int32 V3;

	UPROPERTY(VisibleAnywhere, Category = "LoogPhysics")
	float RestAngleOrVolume;

	UPROPERTY(VisibleAnywhere, Category = "LoogPhysics")
	ELoogPhysicsTriangleSignFlag SignFlag;
};


UENUM()
enum class ELoogPhysicsParticleType
{
	FixedBone,
	MovedBone,
	VirtualBone,
	Max UMETA(Hidden)
};

UENUM()
enum class ELoogPhysicsSimulationMethod
{
	ForceBase,
	VelocityBase,
	PBD,
	XPBD,
	Max UMETA(Hidden)
};

UENUM()
enum class ELoogPhysicsColliderCollisionMethod
{
	NoCollision,
	Point,
	Edge,
	Max UMETA(Hidden)
};

USTRUCT()
struct FLoogPhysicsParticle
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(VisibleAnywhere)
	FBoneReference BindBone;

	UPROPERTY(VisibleAnywhere)
	ELoogPhysicsParticleType ParticleType = ELoogPhysicsParticleType::FixedBone;

	UPROPERTY(VisibleAnywhere)
	FVector LocalRefPosition = FVector::Zero();

	UPROPERTY(VisibleAnywhere)
	FQuat LocalRefRotation = FQuat::Identity;

	UPROPERTY(VisibleAnywhere)
	float Depth = 0.f;

	UPROPERTY(VisibleAnywhere)
	int32 ParentParticleIndex = INDEX_NONE;

	UPROPERTY(VisibleAnywhere)
	int32 RootParticleIndex = INDEX_NONE;

	UPROPERTY(VisibleAnywhere)
	TArray<int32> ChildParticleIndices;

	UPROPERTY(VisibleAnywhere)
	TArray<int32> LinkedParticles;

	// BoneTransform
	FVector BonePosition;
	FQuat BoneRotation;

	FVector BoneLocalPosition;
	FQuat BoneLocalRotation;

	// virtual mesh 每帧通过ReadBones得到的位置和旋转,并在模拟完成后修改为插值后位置。修改前的数据保存在PrevBasePosition/Rotation中
	FVector VirtualMeshPosition;
	FQuat VirtualMeshRotation;

	// 上更新的基准位置和旋转。从virtual mesh得到
	FVector PrevBasePosition;
	FQuat PrevBaseRotation;

	// 每步更新的基准位置和旋转，根据当前步的time，插值基准位置得到
	FVector StepBasePosition;
	FQuat StepBaseRotation;

	// Particle data
	FVector NextPosition; // 每步更新中预测的位置

	FVector OldPosition; // 每步更新完成后得到的最终位置
	FQuat OldRotation; // 每步更新完成后得到的最终旋转

	FVector DisplayPosition;

	FVector Velocity;
	FVector RealVelocity;

	float Friction;
	float StaticFriction;
	FVector CollisionNormal;

	FVector VelocityPosition;

	FTransform BoneTransform;
};

UCLASS(BlueprintType)
class ULoogPhysicsAsset : public UDataAsset
{
	GENERATED_BODY()
public:
	/** Main*/
	UPROPERTY(EditAnywhere, Category = "Main")
	float RotationalInterpolation = 0.5f;

	UPROPERTY(EditAnywhere, Category = "Main")
	float RootRotation = 0.5f;

	UPROPERTY(EditAnywhere, Category = "Main")
	float AnimationPoseRatio = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Main")
	float BlendWeight = 1.0f;

	/** Force*/
	UPROPERTY(EditAnywhere, Category = "Force", meta = (ClampMin = "0"))
	float GravityMagnitude = 500.f;

	/** Force*/
	UPROPERTY(EditAnywhere, Category = "Force", meta = (ClampMin = "0", ClampMax = "1"))
	FVector GravityDirection = FVector::DownVector;

	/** Force*/
	UPROPERTY(EditAnywhere, Category = "Force")
	float GravityFalloff = 0.1f;

	/** Force*/
	UPROPERTY(EditAnywhere, Category = "Force", meta = (ClampMin = "0", ClampMax = "1"))
	float LinearDamping{ 0.1f };

	/** Force*/
	UPROPERTY(EditAnywhere, Category = "Force")
	FRuntimeFloatCurve LinearDampingRichCurve;

	/** Force*/
	UPROPERTY(EditAnywhere, Category = "Force")
	float StablizationTimeAfterReset = 0.1f;

	/** Angle Restoration*/
	UPROPERTY(EditAnywhere, Category = "Angle Restoration")
	bool bUseAngleRestoration = true;

	/** Angle Restoration*/
	UPROPERTY(EditAnywhere, Category = "Angle Restoration", meta = (ClampMin = "0", ClampMax = "1"))
	float AngleRestorationStiffness{ 0.2f };

	/** Angle Restoration*/
	UPROPERTY(EditAnywhere, Category = "Angle Restoration")
	FRuntimeFloatCurve AngleRestorationRichCurve;

	/** Angle Restoration*/
	UPROPERTY(EditAnywhere, Category = "Angle Restoration", meta = (ClampMin = "0", ClampMax = "1"))
	float RestorationGravityFalloff = 0.f;

	/** Angle Restoration*/
	UPROPERTY(EditAnywhere, Category = "Angle Restoration", meta = (ClampMin = "0", ClampMax = "1"))
	float RestorationVelocityAttenuation = 0.8f;

	/** Angle Limit*/
	UPROPERTY(EditAnywhere, Category = "Angle Limit")
	bool bUseAngleLimit = true;

	/** Angle Limit*/
	UPROPERTY(EditAnywhere, Category = "Angle Limit", meta = (ClampMin = "0", ClampMax = "180"))
	float AngleLimit{ 75.f };

	/** Angle Limit*/
	UPROPERTY(EditAnywhere, Category = "Angle Limit")
	FRuntimeFloatCurve AngleLimitRichCurve;

	/** Angle Limit*/
	UPROPERTY(EditAnywhere, Category = "Angle Limit", meta = (ClampMin = "0", ClampMax = "1"))
	float AngleLimitStiffness{ 0.9 };

	/** Shape Restoration*/
	UPROPERTY(EditAnywhere, Category = "Shape Restoration", meta = (ClampMin = "0", ClampMax = "1"))
	float StructureStiffness = 1.f;

	/** Shape Restoration*/
	UPROPERTY(EditAnywhere, Category = "Shape Restoration")
	FRuntimeFloatCurve StructureStiffnessRichCurve;

	/** Shape Restoration*/
	UPROPERTY(EditAnywhere, Category = "Shape Restoration", meta = (ClampMin = "0", ClampMax = "1"))
	float TetherConstraintCompressionLimit = 0.8f;

	UPROPERTY(EditAnywhere, Category = "Shape Restoration", meta = (ClampMin = "0", ClampMax = "1"))
	float TriangleBendingStiffness = 0.8f;
	
	/** Inertia*/
	UPROPERTY(EditAnywhere, Category = "Inertia", meta = (ClampMin = "0", ClampMax = "1"))
	float WorldInertia = 1.f;

	UPROPERTY(EditAnywhere, Category = "Inertia", meta = (ClampMin = "0", ClampMax = "1"))
	float WorldInertiaSmoothing = 0.4f;

	UPROPERTY(EditAnywhere, Category = "Inertia", meta = (ClampMin = "0", ClampMax = "1000"))
	float WorldTranslationSpeedLimit = 500.f;

	UPROPERTY(EditAnywhere, Category = "Inertia", meta = (ClampMin = "0", ClampMax = "1440"))
	float WorldRotationSpeedLimit = 720.0f;

	UPROPERTY(EditAnywhere, Category = "Inertia", meta = (ClampMin = "0", ClampMax = "1"))
	float LocalInertia = 1.f;

	UPROPERTY(EditAnywhere, Category = "Inertia", meta = (ClampMin = "0"))
	float LocalMovementSpeedLimit = 500;

	UPROPERTY(EditAnywhere, Category = "Inertia", meta = (ClampMin = "0"))
	float LocalRotationSpeedLimit = 720;

	UPROPERTY(EditAnywhere, Category = "Inertia", meta = (ClampMin = "0", ClampMax = "1"))
	float DepthInertia = 0.0f;

	UPROPERTY(EditAnywhere, Category = "Inertia")
	float CentrifualAcceleration = 0.f;

	UPROPERTY(EditAnywhere, Category = "Inertia")
	float ParticleSpeedLimit = 500.f;

	/** Collider Collision*/
	UPROPERTY(EditAnywhere, Category = "Collider Collision")
	ELoogPhysicsColliderCollisionMethod CollisionMethod = ELoogPhysicsColliderCollisionMethod::NoCollision;

	/** Collider Collision*/
	UPROPERTY(EditAnywhere, Category = "Collider Collision")
	float Radius = 3.f;

	/** Collider Collision*/
	UPROPERTY(EditAnywhere, Category = "Collider Collision")
	FRuntimeFloatCurve RadiusCurve;

	/** Collider Collision*/
	UPROPERTY(EditAnywhere, Category = "Collider Collision")
	float Friction = 0.05f;

	/** Collider Collision*/
	UPROPERTY(EditAnywhere, Category = "Collider Collision")
	TObjectPtr<UPhysicsAsset> PhysicsAsset;

	/** Movement Limit*/
	static float EvaluateCurve(const FRuntimeFloatCurve& RichCurve, const float& Base, const float& Time)
	{
		const FRichCurve* Curve = RichCurve.GetRichCurveConst();
		if (!Curve->IsEmpty())
		{
			return Curve->Eval(Time, 1.0f) * Base;
		}
		return Base;
	}

	float EvaluateStructureStiffness(const float& InTime) const
	{
		return EvaluateCurve(StructureStiffnessRichCurve, StructureStiffness, InTime);
	}

	float EvaluateLinearDamping(const float& InTime) const
	{
		return EvaluateCurve(LinearDampingRichCurve, LinearDamping, InTime);
	}

	float EvaluateAngleRestorationStiffness(const float& InTime) const
	{
		return EvaluateCurve(AngleRestorationRichCurve, AngleRestorationStiffness, InTime);
	}

	float EvaluateAngleLimit(const float& InTime) const
	{
		return EvaluateCurve(AngleLimitRichCurve, AngleLimit, InTime);
	}

	float EvaluateRadius(const float& InTime) const
	{
		return EvaluateCurve(RadiusCurve, Radius, InTime);
	}
};


struct FLoogPhysicsTeamData
{
	// last frame transform
	FVector OldFrameWorldPosition;
	FQuat OldFrameWorldRotation;

	// current frame transform
	FVector FrameWorldPosition;
	FQuat FrameWorldRotation;

	FVector PrevStepPosition;
	FQuat PrevStepRotation;

	FVector NowStepPosition;
	FQuat NowStepRotation;

	// local惯性
	FVector StepVector;
	FQuat StepRotation;

	float StepTranslationInertiaRatio;
	float StepRotationInertiaRatio;
	FVector InertiaVector;
	FQuat InertiaRotation;

	// 世界惯性
	FVector ComponentWorldPosition;
	FQuat ComponentWorldRotation;
	FVector OldComponentWorldPosition;
	FQuat OldComponentWorldRotation;
	bool bIsInertiaShift;
	FVector FrameComponentShiftVector;
	FQuat FrameComponentShiftRotation;

	// 离心作用
	float AngularVelocity;
	FVector RotationAxis;

	// 速度稳定用
	float VelocityWeight;
	float GravityDot;
	float GravityRatio;

	float BlendWeight;
};

struct FLoogPhysicsCapsule
{
	FKTaperedCapsuleElem TaperedCapsuleElem;
	FBoneReference BindBone;

	float RadiusA;
	float RadiusB;

	// Local Center Position
	FVector CenterA;
	FVector CenterB;
	FTransform LocalTransform;


	// BindBoneTransform world space
	FTransform BoneTransform;

	// Frame Target Transform
	FVector FrameBasePosition;
	FQuat FrameBaseRotation;

	FVector OldFrameBasePosition;
	FQuat OldFrameBaseRotation;

	FVector NowPosition;
	FQuat NowRotation;

	FVector OldPosition;
	FQuat OldRotation;

	// 每步更新所需数据
	UE::Geometry::FAxisAlignedBox3d AABB;

	FVector NowStartPosition;
	FVector NowEndPosition;

	FVector OldStartPosition;
	FVector OldEndPosition;

	FQuat InverseOldRot;

	float PointCapsuleColliderDetection(FVector& InOutNextPos, float Radius, const UE::Geometry::FAxisAlignedBox3d& PointAABB, FVector& OutNormal) const;
	float EdgeCapsuleColliderDetection(FVector& InOutNextPosC0, FVector& InOutNextPosC1, const FVector2f& EdgeRadius, const UE::Geometry::FAxisAlignedBox3d& EdgeAABB, float CollisionFrictionRange, FVector& OutNormal) const;
};

struct FLoogPhysicsWindData
{
};

USTRUCT()
struct FLoogPhysicsClothTeam
{
	GENERATED_USTRUCT_BODY()

	UPROPERTY(EditAnywhere, Category = "LoogPhysics")
	FString SectionName;

	UPROPERTY(VisibleAnywhere, Category = "LoogPhysics")
	int32 StartIndex = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, Category = "LoogPhysics")
	int32 EndIndex = INDEX_NONE;

	UPROPERTY(VisibleAnywhere, Category = "LoogPhysics")
	TArray<FTrianglePair> TrianglePairs;

	UPROPERTY(VisibleAnywhere, Category = "LoogPhysics")
	TArray<FInt32Vector2> Edges;

	UPROPERTY(EditAnywhere, Category = "LoogPhysics")
	TObjectPtr<ULoogPhysicsAsset> ParameterAsset;

	TArray<FLoogPhysicsCapsule> CapsuleColliders;

	FLoogPhysicsTeamData TeamData;
	FLoogPhysicsWindData WindData;
};

USTRUCT(BlueprintType)
struct LOOGPHYSICS_API FAnimNode_LoogPhysics : public FAnimNode_SkeletalControlBase
{
	GENERATED_USTRUCT_BODY()

public:
	UPROPERTY(EditAnywhere, Category = "Setting")
	TArray<FLoogPhysicsParticle> Particles;

	UPROPERTY(EditAnywhere, Category = "Setting")
	TArray<FLoogPhysicsClothTeam> ClothTeams;

	UPROPERTY(EditAnywhere, Category = "Setting")
	int32 SimulationFrameRate = 90;

	UPROPERTY(EditAnywhere, Category = "Setting")
	int32 MinFrameRate = 30;

	UPROPERTY(EditAnywhere, Category = "Setting")
	int32 MaxFrameRate = 150;

	UPROPERTY(EditAnywhere, Category = "Setting")
	int32 MaxSimulationPerFrame = 3;
public:
	friend class FLoogPhysicsEditMode;
	FAnimNode_LoogPhysics();

	// FAnimNode_Base interface
	virtual void UpdateInternal(const FAnimationUpdateContext& Context) override;
	virtual void GatherDebugData(FNodeDebugData& DebugData) override;
	virtual void Initialize_AnyThread(const FAnimationInitializeContext& Context) override;
	virtual void CacheBones_AnyThread(const FAnimationCacheBonesContext& Context) override;
	virtual bool NeedsDynamicReset() const override { return true; }
	virtual void ResetDynamics(ETeleportType InTeleportType) override;
	// End of FAnimNode_Base interface

	// FAnimNode_SkeletalControlBase interface
	virtual void EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms) override;
	virtual bool IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones) override;
	// virtual bool HasPreUpdate() const override;
	// virtual void PreUpdate(const UAnimInstance* InAnimInstance) override;
	// End of FAnimNode_SkeletalControlBase interface


protected:
	// FAnimNode_SkeletalControlBase interface
	virtual void InitializeBoneReferences(const FBoneContainer& RequiredBones) override;
	// End of FAnimNode_SkeletalControlBase interface


private:
	void InitializeSimulationBuffer();

	void InitializeCollider(const FBoneContainer& RequiredBones);

	void FrameTimeUpdate(FComponentSpacePoseContext& Output);

	void AlwaysTeamUpdate(const FComponentSpacePoseContext& Output);

	void AlwaysWindUpdate(FComponentSpacePoseContext& Output);

	void ReadBoneTransform(FComponentSpacePoseContext& Output, const FBoneContainer& RequiredBones);

	void SimulationPreProxyMeshUpdate(FComponentSpacePoseContext& Output, const FBoneContainer& RequiredBones);

	void SimulationCalcCenterAndInertiaAndWind(FComponentSpacePoseContext& Output);

	void SimulationPreTeamUpdate();

	void ColliderSimulationPreUpdate();

	void SimulationStepTeamUpdate();

	void ColliderSimulationStartStep();

	void SimulationStepUpdateParticles();

	// TODO： 动画姿势与模拟姿势混合
	void SimulationStepUpdateBaseLinePose();

	// Root-to-Particle Tether Constraint
	void TetherConstraintSolverConstraint();

	// Particle To linked particle
	void DistanceConstraintSolverConstraint();

	// 角度约束+角度限制
	void AngleConstraintSolverConstraint();

	void SimulationClearTempBuffer();

	void TriangleBendingConstraintSolverConstraint();

	void TriangleBendingConstraintSumConstraint();

	void SolveColliderCollisionConstraint();

	void ColliderCollisionConstraintSolverPointConstraint(FLoogPhysicsClothTeam& InClothTeam);

	void ColliderCollisionConstraintSolverEdgeConstraint(FLoogPhysicsClothTeam& InClothTeam);

	void ColliderCollisionConstraintSumEdgeConstraint(const FLoogPhysicsClothTeam& InClothTeam);

	void MotionConstraintSolverConstraint();

	void SimulationStepPostTeam();

	void ColliderManagerSimulationEndStep();

	void SimulationCalcDisplayPosition();

	void VirtualMeshManagerSimulationPostProxyMeshUpdateLine();

	void VirtualMeshManagerSimulationPostProxyMeshUpdateTriangle();

	void VirtualMeshManagerSimulationPostProxyMeshUpdateTriangleSum();

	void VirtualMeshManagerSimulationPostProxyMeshUpdateWorldTransform();

	void VirtualMeshManagerSimulationPostProxyMeshUpdateLocalTransform();

	void ColliderManagerSimulationPostUpdate();

	void TeamManagerSimulationPostTeamUpdate();

	/**
	 * Applies the simulation results to the bone transforms.
	 *
	 * @param Output The pose context.
	 * @param OutBoneTransforms An array to store the resulting bone transforms.
	 */
	void WriteBackTransforms(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms);

	void CalculateCenterTransform(FLoogPhysicsClothTeam& InClothTeam, FVector& OutCenterPos, FQuat& OutCenterRot);

	ETeleportType PendingTeleportType = ETeleportType::None;
	bool bNeedResetSimulation = true;

	// Time Manager
	float FrameDeltaTime = 0.f;
	float SimulationDeltaTime = 0.f;

	float CurrentTickTime = 0;
	float PrevTickTime = 0;

	float PrevFrameUpdateTime = 0.f;
	float FrameUpdateTime = 0.f;

	float NowUpdateTime = 0;
	float OldUpdateTime = 0;

	float FrameInterpolation = 0.f;

	int32 SkipCount = 0;
	int32 SimStepCount = 0;
	FVector4 SimulationPower;

	// buffer 与动画混合时使用的base transform
	TArray<FVector> StepBasicPositionBuffer;
	TArray<FQuat> StepBasicRotationBuffer;

	// temp buffer
	TArray<float> LengthBufferArray;
	TArray<FVector> LocalPosBufferArray;
	TArray<FQuat> LocalRotBufferArray;
	TArray<FQuat> RotationBufferArray;
	TArray<FVector> RestorationVectorBufferArray;

	TArray<FVector> tempVectorBufferA;
	TArray<FQuat> tempRotationBufferA;
	TArray<FVector> tempVectorBufferB;
	TArray<int32> tempCountBuffer;
	TArray<float> tempFloatBufferA;
};
