// Copyright LoogLong. All Rights Reserved.
#include "LoogPhysicsEditMode.h"
#include "CanvasItem.h"
#include "CanvasTypes.h"
#include "EditorViewportClient.h"
#include "IPersonaPreviewScene.h"
#include "SceneManagement.h"
#include "Animation/DebugSkelMeshComponent.h"
#include "Materials/MaterialInstanceDynamic.h"

#define LOCTEXT_NAMESPACE "LoogPhysicsEditMode"

UE_DISABLE_OPTIMIZATION
FEditorModeID FLoogPhysicsEditMode::ModeName = "AnimGraph.SkeletalControl.LoogPhysics";
FLoogPhysicsEditMode::FLoogPhysicsEditMode()
	: RuntimeNode(nullptr)
	, GraphNode(nullptr)
{
}

void FLoogPhysicsEditMode::EnterMode(UAnimGraphNode_Base* InEditorNode, FAnimNode_Base* InRuntimeNode)
{
	RuntimeNode = static_cast<FAnimNode_LoogPhysics*>(InRuntimeNode);
	GraphNode = CastChecked<UAnimGraphNode_LoogPhysics>(InEditorNode);

	FAnimNodeEditMode::EnterMode(InEditorNode, InRuntimeNode);
}

void FLoogPhysicsEditMode::ExitMode()
{
	GraphNode = nullptr;
	RuntimeNode = nullptr;

	FAnimNodeEditMode::ExitMode();
}

void FLoogPhysicsEditMode::Render(const FSceneView* View, FViewport* Viewport, FPrimitiveDrawInterface* PDI)
{
	const USkeletalMeshComponent* SkelMeshComp = GetAnimPreviewScene().GetPreviewMeshComponent();
	if (RuntimeNode != nullptr)
	{
		if (RuntimeNode->bNeedResetSimulation)
		{
			return;
		}
		const auto& RuntimeParticles = RuntimeNode->Particles;
		if (RuntimeParticles.Num() == 0)
		{
			return;
		}
		const FMaterialRenderProxy* MaterialRenderProxy = GEngine->ConstraintLimitMaterialPrismatic->GetRenderProxy();
		int32 TeamIndex = 0;
		if (RuntimeNode->ClothTeams.Num() != GraphNode->BoneSections.Num())
		{
			return;
		}
		for (const auto& ClothTeam : RuntimeNode->ClothTeams)
		{
			const bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr;
			if (!bParameterValid)
			{
				return;
			}
			const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();

			const auto& Section = GraphNode->BoneSections[TeamIndex];
			TeamIndex++;
			if (!Section.bShowDebugDraw)
			{
				continue;
			}
			const FTransform ComponentTransform{ ClothTeam.TeamData.ComponentWorldRotation, ClothTeam.TeamData.ComponentWorldPosition };
			if (GraphNode->bDrawParticle)
			{
				FVector TeamPosition = ComponentTransform.InverseTransformPosition(ClothTeam.TeamData.NowStepPosition);
				FQuat TeamRotation = ComponentTransform.InverseTransformRotation(ClothTeam.TeamData.NowStepRotation);
				DrawWireSphere(PDI, TeamPosition, FLinearColor::Blue, 2.f, 16, SDPG_Foreground);
				DrawCoordinateSystem(PDI, TeamPosition, TeamRotation.Rotator(), 10.0f, SDPG_Foreground, 1.0f);
				for (int32 ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
				{
					const auto& RuntimeParticle = RuntimeParticles[ParticleIndex];
					float Radius = PhysicsParameter.EvaluateRadius(RuntimeParticle.Depth);
					const auto Color = RuntimeParticle.ParticleType == ELoogPhysicsParticleType::FixedBone ? FLinearColor::Red : FLinearColor::Yellow;
					FVector ParticleOldPos = ComponentTransform.InverseTransformPosition(RuntimeParticle.OldPosition);
					DrawWireSphere(PDI, ParticleOldPos, Color, Radius, 16, SDPG_Foreground);
				}
			}

			if (GraphNode->bDrawVirtualMesh)
			{
				for (auto& Edge : ClothTeam.Edges)
				{
					const auto& AParticle = RuntimeParticles[Edge.X];
					const auto& BParticle = RuntimeParticles[Edge.Y];
					FVector AParticleOldPos = ComponentTransform.InverseTransformPosition(AParticle.OldPosition);
					FVector BParticleOldPos = ComponentTransform.InverseTransformPosition(BParticle.OldPosition);
					PDI->DrawLine(AParticleOldPos, BParticleOldPos, FLinearColor::Yellow, SDPG_Foreground);
				}
			}
			if (GraphNode->bDrawBasePose)
			{
				for (int32 ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
				{
					const auto& RuntimeParticle = RuntimeParticles[ParticleIndex];
					FVector PrevBasePosition = ComponentTransform.InverseTransformPosition(RuntimeParticle.PrevBasePosition);
					DrawWireSphere(PDI, PrevBasePosition, FLinearColor::Red, 1.0f, 16, SDPG_Foreground);
				}
				for (auto& Edge : ClothTeam.Edges)
				{
					const auto& AParticle = RuntimeParticles[Edge.X];
					const auto& BParticle = RuntimeParticles[Edge.Y];
					FVector APrevBasePosition = ComponentTransform.InverseTransformPosition(AParticle.PrevBasePosition);
					FVector BPrevBasePosition = ComponentTransform.InverseTransformPosition(BParticle.PrevBasePosition);
					PDI->DrawLine(APrevBasePosition, BPrevBasePosition, FLinearColor::Red, SDPG_Foreground);
				}
			}
			if (GraphNode->bDrawBodyCollider)
			{
				int32 ColliderCount = ClothTeam.CapsuleColliders.Num();
				for (int32 Idx = 0; Idx < ColliderCount; ++Idx)
				{
					auto& RuntimeCollider = ClothTeam.CapsuleColliders[Idx];
					FQuat OldFrameBaseRotation = ComponentTransform.InverseTransformRotation(RuntimeCollider.OldFrameBaseRotation);
					FVector OldFrameBasePosition = ComponentTransform.InverseTransformPosition(RuntimeCollider.OldFrameBasePosition);
					FTransform ColliderTransform(OldFrameBaseRotation, OldFrameBasePosition);
					RuntimeCollider.TaperedCapsuleElem.DrawElemSolid(PDI, ColliderTransform, 1.f, MaterialRenderProxy);
					RuntimeCollider.TaperedCapsuleElem.DrawElemWire(PDI, ColliderTransform, 1.f, FColor::Black);
				}
			}
		}
	}
	FAnimNodeEditMode::Render(View, Viewport, PDI);
}

void FLoogPhysicsEditMode::DrawHUD(FEditorViewportClient* ViewportClient, FViewport* Viewport, const FSceneView* View, FCanvas* Canvas)
{
	const UDebugSkelMeshComponent* PreviewMeshComponent = GetAnimPreviewScene().GetPreviewMeshComponent();
	if (GraphNode->bShowParticleDepth || GraphNode->bShowParticleIndex)
	{
		if (PreviewMeshComponent != nullptr && PreviewMeshComponent->MeshObject != nullptr)
		{
			for (const auto& ClothTeam : RuntimeNode->ClothTeams)
			{
				const FTransform ComponentTransform{ ClothTeam.TeamData.ComponentWorldRotation, ClothTeam.TeamData.ComponentWorldPosition };
				for (int32 ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
				{
					auto& Particle = RuntimeNode->Particles[ParticleIndex];
					const FVector ParticlePos = ComponentTransform.InverseTransformPosition(Particle.OldPosition);
					if (GraphNode->bShowParticleDepth)
					{
						Draw3DTextItem(FText::AsNumber(Particle.Depth), Canvas, View, Viewport, ParticlePos);
					}
					if (GraphNode->bShowParticleIndex)
					{
						Draw3DTextItem(FText::AsNumber(ParticleIndex), Canvas, View, Viewport, ParticlePos);
					}
				}
			}
			
		}
	}

	FAnimNodeEditMode::DrawHUD(ViewportClient, Viewport, View, Canvas);
}

void FLoogPhysicsEditMode::DrawTextItem(const FText& Text, FCanvas* Canvas, float X, float& Y, float FontHeight)
{
	FCanvasTextItem TextItem(FVector2D::ZeroVector, Text, GEngine->GetSmallFont(), FLinearColor::White);
	TextItem.EnableShadow(FLinearColor::Black);
	Canvas->DrawItem(TextItem, X, Y);
	Y -= (3 + FontHeight);
}

void FLoogPhysicsEditMode::Draw3DTextItem(const FText& Text, FCanvas* Canvas, const FSceneView* View, const FViewport* Viewport, FVector Location)
{
	const int32 HalfX = Viewport->GetSizeXY().X / 2 / Canvas->GetDPIScale();
	const int32 HalfY = Viewport->GetSizeXY().Y / 2 / Canvas->GetDPIScale();

	const FPlane proj = View->Project(Location);
	if (proj.W > 0.f)
	{
		const int32 XPos = HalfX + (HalfX * proj.X);
		const int32 YPos = HalfY + (HalfY * (proj.Y * -1));
		FCanvasTextItem TextItem(FVector2D(XPos, YPos), Text, GEngine->GetSmallFont(), FLinearColor::White);
		TextItem.EnableShadow(FLinearColor::Black);
		Canvas->DrawItem(TextItem);
	}
}

#undef LOCTEXT_NAMESPACE
UE_ENABLE_OPTIMIZATION