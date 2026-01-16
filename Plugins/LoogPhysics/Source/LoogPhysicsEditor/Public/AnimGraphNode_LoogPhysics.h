// Copyright LoogLong. All Rights Reserved.
#pragma once
#include "AnimNode_LoogPhysics.h"
#include "AnimGraphNode_SkeletalControlBase.h"
#include "EdGraph/EdGraphNodeUtils.h"

#include "AnimGraphNode_LoogPhysics.generated.h"


USTRUCT()
struct FLoogPhysicsRootBone
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "BoneReference")
	FName BoneName;

	UPROPERTY(EditAnywhere, Category = "BoneReference")
	float VirtualBoneLength = 10.0f;

	UPROPERTY(EditAnywhere, Category = "BoneReference")
	int32 FixedDepth = 0;
};


USTRUCT()
struct FLoogPhysicsBoneSection
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, Category = "LoogPhysics")
	TArray<FLoogPhysicsRootBone> RootBones;

	UPROPERTY(EditAnywhere, Category = "LoogPhysics")
	bool bShowDebugDraw = true;

	UPROPERTY(EditAnywhere, Category = "LoogPhysics")
	bool bNeedConnectLines = false;

	UPROPERTY(EditAnywhere, Category = "LoogPhysics")
	bool bLoopConnection = false;

	UPROPERTY(EditAnywhere, Category = "LoogPhysics")
	bool bAutoConnection = false;
};

UCLASS()
class UAnimGraphNode_LoogPhysics : public UAnimGraphNode_SkeletalControlBase
{
	GENERATED_UCLASS_BODY()
	UPROPERTY(EditAnywhere, Category = Settings)
	FAnimNode_LoogPhysics Node;

	virtual FText GetNodeTitle(ENodeTitleType::Type TitleType) const override;

	// UObject interface
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;

protected:
	// UAnimGraphNode_Base interface
	virtual FEditorModeID GetEditorMode() const override;
	// virtual void ValidateAnimNodePostCompile(FCompilerResultsLog& MessageLog,
	//                                          UAnimBlueprintGeneratedClass* CompiledClass,
	//                                          int32 CompiledNodeIndex) override;
	virtual void CopyNodeDataToPreviewNode(FAnimNode_Base* AnimNode) override;
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
	// End of UAnimGraphNode_Base interface

	//virtual FText GetControllerDescription() const override;
	virtual FText GetControllerDescription() const override;
	virtual const FAnimNode_SkeletalControlBase* GetNode() const override { return &Node; }
	// End of UAnimGraphNode_SkeletalControlBase interface

	// UObject interface
	virtual void Serialize(FArchive& Ar) override;

	// End of UObject interface

public:
	UPROPERTY(EditAnywhere, Category = "Loog Physics Tools")
	TObjectPtr<USkeletalMesh> SkeletalMesh;

	UPROPERTY(EditAnywhere, Category = "Loog Physics Tools")
	TArray<FLoogPhysicsBoneSection> BoneSections;

	UPROPERTY(EditAnywhere, Category = "Loog Physics Tools")
	float ProxyMeshBoneClothTriangleAngle = 120.f;

	UPROPERTY(EditAnywhere, Category = "Loog Physics Tools")
	bool bDrawParticle = false;

	UPROPERTY(EditAnywhere, Category = "Loog Physics Tools")
	bool bDrawVirtualMesh = false;

	UPROPERTY(EditAnywhere, Category = "Loog Physics Tools")
	bool bDrawBasePose = false;

	UPROPERTY(EditAnywhere, Category = "Loog Physics Tools")
	bool bShowParticleDepth = false;

	UPROPERTY(EditAnywhere, Category = "Loog Physics Tools")
	bool bShowParticleIndex = false;

	UPROPERTY(EditAnywhere, Category = "Loog Physics Tools")
	bool bDrawBodyCollider = false;
private:
	/** Constructing FText strings can be costly, so we cache the node's title */
	FNodeTitleTextTable CachedNodeTitles;

	void CreateParticlesAndConstraints();

	void ModifyParticleProperty();
};
