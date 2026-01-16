// Copyright LoogLong. All Rights Reserved.
#include "AnimGraphNode_LoogPhysics.h"

#include "DetailCategoryBuilder.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "LoogPhysics.h"
#include "LoogPhysicsEditMode.h"
#include "Kismet2/CompilerResultsLog.h"
#include "Widgets/Layout/SUniformGridPanel.h"

#define LOCTEXT_NAMESPACE "LoogPhysics"
UE_DISABLE_OPTIMIZATION

namespace LoogPhysicsMath
{
	/**
	 * 计算二面角的夹角。
	 * 二面角夹角等同于两个面法线的夹角
	 */
	void CalculateDihedralAngle(const FVector& Pos0, const FVector& Pos1, const FVector& Pos2, const FVector& Pos3, float& RestAngle, ELoogPhysicsTriangleSignFlag& SignFlag)
	{
		FVector N1 = FVector::CrossProduct(Pos2 - Pos0, Pos3 - Pos0);
		FVector N2 = FVector::CrossProduct(Pos3 - Pos1, Pos2 - Pos1);
		N1.Normalize();
		N2.Normalize();
		float Dot = FVector::DotProduct(N1, N2);
		Dot = FMath::Clamp(Dot, -1.f, 1.f);

		RestAngle = FMath::Acos(Dot);

		// 计算方向性
		// 将恢复方向存储为正或负标志。
		FVector E = Pos3 - Pos2;
		float Dir = FVector::DotProduct(FVector::CrossProduct(N1, N2), E);
		SignFlag = Dir < 0.f ? ELoogPhysicsTriangleSignFlag::Negative : ELoogPhysicsTriangleSignFlag::Positive;
	}

	/**
	 * 计算四边形体积
	 */
	void CalculateVolume(const FVector& Pos0, const FVector& Pos1, const FVector& Pos2, const FVector& Pos3, float& VolumeRest, ELoogPhysicsTriangleSignFlag& SignFlag)
	{
		// 0/1 为对角点，2/3 为公共边
		VolumeRest = (1.0f / 6.0f) * FVector::DotProduct(FVector::CrossProduct(Pos1 - Pos0, Pos2 - Pos0), Pos3 - Pos0);
		VolumeRest *= PhysicsDefine::VolumeScale; //浮点数计算误差
		SignFlag = ELoogPhysicsTriangleSignFlag::Volume;
	}
}
// ----------------------------------------------------------------------------
UAnimGraphNode_LoogPhysics::UAnimGraphNode_LoogPhysics(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

FText UAnimGraphNode_LoogPhysics::GetControllerDescription() const
{
	return LOCTEXT("LoogPhysics", "Loog Physics");
}


// ----------------------------------------------------------------------------
FText UAnimGraphNode_LoogPhysics::GetNodeTitle(ENodeTitleType::Type TitleType) const
{
	return LOCTEXT("NodeTitle", "Loog Physics");
}

void UAnimGraphNode_LoogPhysics::PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);

}

FEditorModeID UAnimGraphNode_LoogPhysics::GetEditorMode() const
{
	return FLoogPhysicsEditMode::ModeName;
}

void UAnimGraphNode_LoogPhysics::CopyNodeDataToPreviewNode(FAnimNode_Base* AnimNode)
{
	// FAnimNode_LoogPhysics* LoogPhysics = reinterpret_cast<FAnimNode_LoogPhysics*>(AnimNode);
	//
	// Node.ClothTeams[0].Constraints;
	// for (int i = 0; i < Node.ClothTeams.Num(); ++i)
	// {
	// 	LoogPhysics->ClothTeams[i].Constraints = Node.ClothTeams[i].Constraints;
	// }
	Super::CopyNodeDataToPreviewNode(AnimNode);
}

void UAnimGraphNode_LoogPhysics::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
	IDetailCategoryBuilder& ViewportCategory = DetailBuilder.EditCategory(TEXT("Loog Physics Tools"));
	FDetailWidgetRow& WidgetRow = ViewportCategory.AddCustomRow(LOCTEXT("LoogPhysics", "LoogPhysicsTools"));

	WidgetRow[SNew(SUniformGridPanel).SlotPadding(FMargin(2, 0, 2, 0))
		+ SUniformGridPanel::Slot(0, 0)
		[
			SNew(SButton)
			.HAlign(HAlign_Center)
			.VAlign(VAlign_Center)
			.OnClicked_Lambda([this]()
			{
				this->CreateParticlesAndConstraints();
				return FReply::Handled();
			})
			.Content()
			[
				SNew(STextBlock).Text(FText::FromString(TEXT("CreateParticlesAndConstraints")))
			]
		]
		// + SUniformGridPanel::Slot(0, 0)
		// [
		// 	SNew(SButton)
		// 		.HAlign(HAlign_Center)
		// 		.VAlign(VAlign_Center)
		// 		.OnClicked_Lambda([this]()
		// 			{
		// 				this->CreateBendingConstraints();
		// 				return FReply::Handled();
		// 			})
		// 		.Content()
		// 		[
		// 			SNew(STextBlock)
		// 				.Text(FText::FromString(TEXT("CreateBendingConstraints")))
		// 		]
		// ]
		// + SUniformGridPanel::Slot(0, 1)
		// [
		// 	SNew(SButton)
		// 		.HAlign(HAlign_Center)
		// 		.VAlign(VAlign_Center)
		// 		.OnClicked_Lambda([this]()
		// 			{
		// 				this->CreateShearConstraints();
		// 				return FReply::Handled();
		// 			})
		// 		.Content()
		// 		[
		// 			SNew(STextBlock)
		// 				.Text(FText::FromString(TEXT("CreateShearingConstraints")))
		// 		]
		// ]
		// + SUniformGridPanel::Slot(0, 2)
		// [
		// 	SNew(SButton)
		// 		.HAlign(HAlign_Center)
		// 		.VAlign(VAlign_Center)
		// 		.OnClicked_Lambda([this]()
		// 			{
		// 				this->RemoveConstraints();
		// 				return FReply::Handled();
		// 			})
		// 		.Content()
		// 		[
		// 			SNew(STextBlock)
		// 				.Text(FText::FromString(TEXT("RemoveConstraints")))
		// 		]
		// ]
		// + SUniformGridPanel::Slot(0, 3)
		// [
		// 	SNew(SButton)
		// 		.HAlign(HAlign_Center)
		// 		.VAlign(VAlign_Center)
		// 		.OnClicked_Lambda([this]()
		// 			{
		// 				this->ModifyParticleProperty();
		// 				return FReply::Handled();
		// 			})
		// 		.Content()
		// 		[
		// 			SNew(STextBlock)
		// 				.Text(FText::FromString(TEXT("ModifyParticleProperty")))
		// 		]
		// ]
	];
	Super::CustomizeDetails(DetailBuilder);
}


void UAnimGraphNode_LoogPhysics::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);
}

void UAnimGraphNode_LoogPhysics::CreateParticlesAndConstraints()
{
	if (!SkeletalMesh)
	{
		return;
	}
	TArray<TObjectPtr<ULoogPhysicsAsset>> OldAssets;
	for (const FLoogPhysicsClothTeam& ClothTeam : Node.ClothTeams)
	{
		OldAssets.Add(ClothTeam.ParameterAsset);
	}
	Node.ClothTeams.Empty(BoneSections.Num());
	Node.Particles.Empty();
	TMap<FName, int32> BoneIndexMap;        // bone name to Particle Index
	TMap<FName, int32> VirtualBoneIndexMap; // Virtual Bone name to Particle Index

	auto AddParticleUnique = [&](const FName& InBoneName, ELoogPhysicsParticleType InParticleType, const FVector& LocalRefPosition, const FQuat& LocalRefRotation) mutable
	{
		TMap<FName, int32>* MapToUse = nullptr;
		if (InParticleType == ELoogPhysicsParticleType::VirtualBone)
		{
			MapToUse = &VirtualBoneIndexMap;
		}
		else
		{
			MapToUse = &BoneIndexMap;
		}

		auto ExistParticleIndexPtr = MapToUse->Find(InBoneName);
		if (ExistParticleIndexPtr != nullptr)
		{
			return *ExistParticleIndexPtr;
		}
		int32 ParticleIndex = Node.Particles.Num();
		MapToUse->Add(InBoneName, ParticleIndex);

		auto& Particle = Node.Particles.AddDefaulted_GetRef();
		Particle.BindBone.BoneName = InBoneName;
		Particle.ParticleType = InParticleType;
		Particle.LocalRefPosition = LocalRefPosition;
		Particle.LocalRefRotation = LocalRefRotation;
		return ParticleIndex;
	};

	auto& RefSkeleton = SkeletalMesh->GetRefSkeleton();
	TMap<int32, TArray<int32>> BoneChildrenMap; // bone index to child bone indices
	// build BoneChildrenMap
	const auto& BoneInfoArray = RefSkeleton.GetRawRefBoneInfo();
	const auto& RefBonePose = RefSkeleton.GetRefBonePose();
	for (int BoneIndex = 0; BoneIndex < BoneInfoArray.Num(); ++BoneIndex)
	{
		const auto& BoneInfo = BoneInfoArray[BoneIndex];
		if (BoneInfo.ParentIndex == INDEX_NONE)
		{
			// root bone
			continue;
		}
		auto& ParentChildren = BoneChildrenMap.FindOrAdd(BoneInfo.ParentIndex);
		ParentChildren.Add(BoneIndex);
	}

	int32 ParameterIndex = 0;
	for (FLoogPhysicsBoneSection& BoneSection : BoneSections)
	{
		auto& ClothSection = Node.ClothTeams.AddDefaulted_GetRef();
		ClothSection.ParameterAsset = OldAssets.IsValidIndex(ParameterIndex) ? OldAssets[ParameterIndex] : nullptr;
		ParameterIndex++;
		ClothSection.StartIndex = Node.Particles.Num();
		TArray<TArray<int32>> OrderToParticleIndex; // 相同深度的放到一起
		TArray<int32> OffsetBindBoneIndex; // bone索引
		TArray<int32> OffsetRootIndex;
		TArray<int32> ParticleRoots;

		for (int32 RootIndex = 0; RootIndex < BoneSection.RootBones.Num(); ++RootIndex)
		{
			FLoogPhysicsRootBone& RootBone = BoneSection.RootBones[RootIndex];
			int32 RootBoneIndex = RefSkeleton.FindBoneIndex(RootBone.BoneName);
			if (RootBoneIndex == INDEX_NONE)
			{
				continue;
			}

			// 先创建particle
			TArray<int32> ParticleBoneIndices;
			TArray<int32> ParticleParticleIndices;
			TArray<int32> BoneOrder;
			TArray<int32> BoneParents;
			ParticleBoneIndices.Add(RootBoneIndex);
			BoneOrder.Add(0);
			BoneParents.Add(INDEX_NONE);
			int32 ParticleBoneIndex = 0;
			int32 MaxOrder = 0;
			while (ParticleBoneIndex < ParticleBoneIndices.Num())
			{
				int32 BoneIndex = ParticleBoneIndices[ParticleBoneIndex];
				int32 ParentParticleIndex = ParticleBoneIndex;
				ParticleBoneIndex++;

				TArray<int32>* Children = BoneChildrenMap.Find(BoneIndex);
				if (Children == nullptr)
				{
					continue;
				}
				if (Children->Num() == 0)
				{
					continue;
				}
				int32 ChildDepth = BoneOrder[ParentParticleIndex] + 1;
				if (ChildDepth > MaxOrder)
				{
					MaxOrder = ChildDepth;
				}
				for (int32 ChildBoneIndex : *Children)
				{
					ParticleBoneIndices.Add(ChildBoneIndex);
					BoneOrder.Add(ChildDepth);
					BoneParents.Add(ParentParticleIndex);
				}
			}
			if (MaxOrder >= OrderToParticleIndex.Num())
			{
				int32 DepthToAdd = RootBone.VirtualBoneLength > 0.f ? MaxOrder - OrderToParticleIndex.Num() + 2 : MaxOrder - OrderToParticleIndex.Num() + 1;
				OrderToParticleIndex.AddDefaulted(DepthToAdd);
			}
			int32 RootParticleIndex = Node.Particles.Num();
			for (int i = 0; i < ParticleBoneIndices.Num(); ++i)
			{
				auto BoneIndex = ParticleBoneIndices[i];
				auto Type = BoneOrder[i] <= RootBone.FixedDepth ? ELoogPhysicsParticleType::FixedBone : ELoogPhysicsParticleType::MovedBone;
				const FTransform& BoneLocalTransform = RefBonePose[BoneIndex];
				int32 ParticleIndex = AddParticleUnique(RefSkeleton.GetBoneName(BoneIndex), Type, BoneLocalTransform.GetTranslation(), BoneLocalTransform.GetRotation());
				ParticleParticleIndices.Add(ParticleIndex);
				auto& ChildParticle = Node.Particles[ParticleIndex];
				ChildParticle.RootParticleIndex = RootParticleIndex;

				if (BoneParents[i] != INDEX_NONE)
				{
					int32 ParentParticleIndex = ParticleParticleIndices[BoneParents[i]];
					auto& ParentParticle = Node.Particles[ParentParticleIndex];
					ParentParticle.ChildParticleIndices.Add(ParticleIndex);
					ChildParticle.ParentParticleIndex = ParentParticleIndex;
				}
				else
				{
					ParticleRoots.Add(RootIndex);
				}
				OrderToParticleIndex[BoneOrder[i]].Add(ParticleIndex);
				OffsetRootIndex.Add(RootIndex);
				OffsetBindBoneIndex.Add(BoneIndex);
			}
			// 创建virtual bone particle
			if (RootBone.VirtualBoneLength > 0.f)
			{
				for (int i = 0; i < ParticleBoneIndices.Num(); ++i)
				{
					if (BoneOrder[i] == MaxOrder)
					{
						auto BoneIndex = ParticleBoneIndices[i];
						const FTransform& BoneLocalTransform = RefBonePose[BoneIndex];
						FVector VirtualBoneLocalPosition = BoneLocalTransform.GetTranslation().GetSafeNormal() * RootBone.VirtualBoneLength;
						int32 ParticleIndex = AddParticleUnique(RefSkeleton.GetBoneName(BoneIndex), ELoogPhysicsParticleType::VirtualBone, VirtualBoneLocalPosition, BoneLocalTransform.GetRotation());
						OrderToParticleIndex[MaxOrder+1].Add(ParticleIndex);
						OffsetRootIndex.Add(RootIndex);
						OffsetBindBoneIndex.Add(BoneIndex);

						int32 ParentParticleIndex = ParticleParticleIndices[i];
						auto& ParentParticle = Node.Particles[ParentParticleIndex];
						auto& ChildParticle = Node.Particles[ParticleIndex];
						ParentParticle.ChildParticleIndices.Add(ParticleIndex);
						ChildParticle.ParentParticleIndex = ParentParticleIndex;
						ChildParticle.RootParticleIndex = RootParticleIndex;
					}
				}
			}
		}

		ClothSection.EndIndex = Node.Particles.Num() - 1;
		// 设置一些属性
		float GroupMaxLength = 0.f;
		TArray<float> OffsetLength;
		OffsetLength.SetNumZeroed(ClothSection.EndIndex - ClothSection.StartIndex + 1);
		for (int32 ParticleIndex = ClothSection.StartIndex; ParticleIndex <= ClothSection.EndIndex; ++ParticleIndex)
		{
			auto& Particle = Node.Particles[ParticleIndex];

			int32 Offset = ParticleIndex - ClothSection.StartIndex;

			// 竖着链接 索引为正
			if (Particle.ParentParticleIndex != INDEX_NONE)
			{
				Particle.LinkedParticles.Add(Particle.ParentParticleIndex);
			}
			for (int32 LinkedParticle : Particle.ChildParticleIndices)
			{
				ensure(LinkedParticle != INDEX_NONE);
				Particle.LinkedParticles.Add(LinkedParticle);
			}
			if (Particle.ParticleType == ELoogPhysicsParticleType::FixedBone)
			{
				OffsetLength[Offset] = 0.f;
			}
			else
			{
				int32 ParentOffset = Particle.ParentParticleIndex - ClothSection.StartIndex;
				const float& ParentLength = OffsetLength[ParentOffset];
				int32 BindBoneIndex = OffsetBindBoneIndex[Offset];
				float ChildLength = ParentLength + RefBonePose[BindBoneIndex].GetTranslation().Length();
				OffsetLength[Offset] = ChildLength;
				if (GroupMaxLength < ChildLength)
				{
					GroupMaxLength = ChildLength;
				}
			}
		}
		ensure(GroupMaxLength >= 0.0f);
		float InvGroupMaxLength = 1.0f / GroupMaxLength;
		for (int32 ParticleIndex = ClothSection.StartIndex; ParticleIndex <= ClothSection.EndIndex; ++ParticleIndex)
		{
			auto& Particle = Node.Particles[ParticleIndex];
			int32 Offset = ParticleIndex - ClothSection.StartIndex;
			const float& Length = OffsetLength[Offset];
			float Depth = Length * InvGroupMaxLength;
			Particle.Depth = Depth;
		}

		// 横向链接
		TArray<FTransform> AbsoluteTransform;
		RefSkeleton.GetBoneAbsoluteTransforms(AbsoluteTransform);
		auto GetAbsolutePosition = [&](int32 ParticleIndex)
			{
				auto& Particle = Node.Particles[ParticleIndex];
				int32 ParticleIndexOffset1 = ParticleIndex - ClothSection.StartIndex;
				int32 BindBoneIndex = OffsetBindBoneIndex[ParticleIndexOffset1];
				FTransform& BoneTransform = AbsoluteTransform[BindBoneIndex];
				FVector AbsolutePosition;
				if (Particle.ParticleType == ELoogPhysicsParticleType::VirtualBone)
				{
					AbsolutePosition = BoneTransform.TransformVector(Particle.LocalRefPosition);
				}
				else
				{
					AbsolutePosition = BoneTransform.GetTranslation();
				}
				return AbsolutePosition;
			};
		if (BoneSection.bNeedConnectLines)
		{
			for (int32 Depth = 0; Depth < OrderToParticleIndex.Num(); ++Depth)
			{
				// 相同层级的质点，按照距离连接
				auto& Link = OrderToParticleIndex[Depth];
				if (Link.Num() > 1)
				{
					for (int LinkId = 0; LinkId < Link.Num(); ++LinkId)
					{
						int32 FirstIndex = Link[LinkId];
						FVector FirstPosition = GetAbsolutePosition(FirstIndex);
						int32 ParticleIndexOffset1 = FirstIndex - ClothSection.StartIndex;

						// find nearest
						if (BoneSection.bAutoConnection)
						{
							float NearestDistance = UE_MAX_FLT;
							int32 NearestIndex = INDEX_NONE;
							for (int32 ParticleIndex : Link)
							{
								if (ParticleIndex == Link[LinkId])
								{
									continue;
								}
								int32 RootIndex1 = OffsetRootIndex[ParticleIndexOffset1];
								int32 ParticleIndexOffset2 = ParticleIndex - ClothSection.StartIndex;
								int32 RootIndex2 = OffsetRootIndex[ParticleIndexOffset2];
								bool bFirstLast = RootIndex1 < RootIndex2 ? RootIndex1 == ParticleRoots[0] && RootIndex2 == ParticleRoots.Last() : RootIndex2 == ParticleRoots[0] && RootIndex1 == ParticleRoots.Last();
								if (!BoneSection.bLoopConnection && bFirstLast)
								{
									continue;
								}
								if (FMath::Abs(RootIndex1 - RootIndex2) > 1)
								{
									continue;
								}
								FVector SecondPosition = GetAbsolutePosition(ParticleIndex);
								float Distance = FVector::Distance(FirstPosition, SecondPosition);
								if (Distance < NearestDistance)
								{
									NearestDistance = Distance;
									NearestIndex = ParticleIndex;
								}
							}
							if (NearestIndex != INDEX_NONE)
							{
								auto& Particle = Node.Particles[FirstIndex];
								Particle.LinkedParticles.Add(-FirstIndex);
								// TODO 自动连接，通过距离判定
							}
						}
						else
						{
							// 如果是依次连接的，就只连接相邻的两条链 这种情况下一般一个链路不会有分叉
							for (int32 ParticleIndex : Link)
							{
								if (ParticleIndex == Link[LinkId])
								{
									continue;
								}
								int32 RootIndex1 = OffsetRootIndex[ParticleIndexOffset1];
								int32 ParticleIndexOffset2 = ParticleIndex - ClothSection.StartIndex;
								int32 RootIndex2 = OffsetRootIndex[ParticleIndexOffset2];
								bool bFirstLast = RootIndex1 < RootIndex2 ? RootIndex1 == ParticleRoots[0] && RootIndex2 == ParticleRoots.Last() : RootIndex2 == ParticleRoots[0] && RootIndex1 == ParticleRoots.Last();
								if (!BoneSection.bLoopConnection && bFirstLast)
								{
									continue;
								}
								if (!(BoneSection.bLoopConnection && bFirstLast) && FMath::Abs(RootIndex1 - RootIndex2) > 1)
								{
									continue;
								}
								auto& Particle = Node.Particles[FirstIndex];
								Particle.LinkedParticles.Add(-ParticleIndex);
							}
						}
					}
				}
			}
		}

		TSet<FInt32Vector2> EdgeSet;
		TSet<FInt32Vector2> TriangleEdgeSet;
		TSet<FInt32Vector3> TriangleSet;

		TArray<FInt32Vector3> Triangles;
		TArray<FInt32Vector2> Lines;
		for (int32 ParticleIndex = ClothSection.StartIndex; ParticleIndex <= ClothSection.EndIndex; ++ParticleIndex)
		{
			auto& Particle = Node.Particles[ParticleIndex];
			if (Particle.LinkedParticles.Num() == 0)
			{
				UE_LOG(LogLoogPhysics, Error, TEXT("Zero Connection [%s]"), *Particle.BindBone.BoneName.ToString())
			}
			else if (Particle.LinkedParticles.Num() == 1)
			{
				int32 LinkedParticle = Particle.LinkedParticles[0];
				LinkedParticle = LinkedParticle > 0 ? LinkedParticle : -LinkedParticle;
				FInt32Vector2 Edge = ParticleIndex < LinkedParticle ? FInt32Vector2(ParticleIndex, LinkedParticle) : FInt32Vector2(LinkedParticle, ParticleIndex);
				EdgeSet.Add(Edge);
			}
			else
			{
				// 能够构成三角形，但是也需要先把所有边记录下来
				for (int32 LinkedParticle : Particle.LinkedParticles)
				{
					LinkedParticle = LinkedParticle > 0 ? LinkedParticle : -LinkedParticle;
					FInt32Vector2 Edge = ParticleIndex < LinkedParticle ? FInt32Vector2(ParticleIndex, LinkedParticle) : FInt32Vector2(LinkedParticle, ParticleIndex);
					EdgeSet.Add(Edge);
				}
				FVector ThisPosition = GetAbsolutePosition(ParticleIndex);
				int32 ThisRootIndex = OffsetRootIndex[ParticleIndex - ClothSection.StartIndex];

				for (int32 I = 0; I < Particle.LinkedParticles.Num(); ++I)
				{
					int32 AIndex = Particle.LinkedParticles[I];
					AIndex = AIndex > 0 ? AIndex : -AIndex;
					FVector APosition = GetAbsolutePosition(AIndex);
					FVector AVector = APosition - ThisPosition;
					if (!AVector.Normalize())
					{
						continue;
					}
					for (int32 J = I + 1; J < Particle.LinkedParticles.Num(); ++J)
					{
						int32 BIndex = Particle.LinkedParticles[J];
						BIndex = BIndex > 0 ? BIndex : -BIndex;
						FVector BPosition = GetAbsolutePosition(BIndex);
						FVector BVector = BPosition - ThisPosition;
						if (!BVector.Normalize())
						{
							continue;
						}
						// 角度太大了。说明三个点快在一条线上，不构成三角形
						float VectorAngle = FMath::RadiansToDegrees(FMath::Acos(AVector | BVector));
						if (VectorAngle >= ProxyMeshBoneClothTriangleAngle)
						{
							continue;
						}
						// 不能是三条不同的链，它们不构成三角形
						int32 RootIndexA = OffsetRootIndex[AIndex - ClothSection.StartIndex];
						int32 RootIndexB = OffsetRootIndex[BIndex - ClothSection.StartIndex];
						if (RootIndexA != ThisRootIndex && RootIndexB != ThisRootIndex && RootIndexA != RootIndexB)
						{
							continue;
						}
						//三角形必须至少包含一条主边(即父子关系)
						bool bIsAnyParents = AIndex == Particle.ParentParticleIndex || BIndex == Particle.ParentParticleIndex;
						bool bIsAnyChild = Particle.ChildParticleIndices.Contains(AIndex) || Particle.ChildParticleIndices.Contains(BIndex);
						if (!bIsAnyParents && !bIsAnyChild)
						{
							continue;
						}
						TArray<int32> TempSort;
						TempSort.Add(ParticleIndex);
						TempSort.Add(AIndex);
						TempSort.Add(BIndex);
						TempSort.Sort();
						TriangleSet.Add(FInt32Vector3(TempSort[0], TempSort[1], TempSort[2]));
						FInt32Vector2 EdgeA = ParticleIndex < AIndex ? FInt32Vector2(ParticleIndex, AIndex) : FInt32Vector2(AIndex, ParticleIndex);
						TriangleEdgeSet.Add(EdgeA);
						FInt32Vector2 EdgeB = ParticleIndex < BIndex ? FInt32Vector2(ParticleIndex, BIndex) : FInt32Vector2(BIndex, ParticleIndex);
						TriangleEdgeSet.Add(EdgeB);
					}
				}
				
			}
		}
		if (TriangleSet.Num() > 0)
		{
			Triangles.Reserve(TriangleSet.Num());
			for (const FInt32Vector3& Triangle : TriangleSet)
			{
				Triangles.Add(Triangle);
			}
		}
		for (FInt32Vector2& TriangleEdge : TriangleEdgeSet)
		{
			EdgeSet.Remove(TriangleEdge);
		}
		if (EdgeSet.Num() > 0)
		{
			Lines.Reserve(EdgeSet.Num());
			for (const FInt32Vector2& Edge : EdgeSet)
			{
				Lines.Add(Edge);
			}
		}
		// 剔除重复的三角形
		TMap<FInt32Vector2, TArray<int32>> EdgeToTriangleMap;
		for (int32 TriangleIndex = 0; TriangleIndex < Triangles.Num(); ++TriangleIndex)
		{
			const FInt32Vector3& Triangle = Triangles[TriangleIndex];
			FInt32Vector2 Edge1 = FInt32Vector2(Triangle.X, Triangle.Y);
			FInt32Vector2 Edge2 = FInt32Vector2(Triangle.Y, Triangle.Z);
			FInt32Vector2 Edge3 = FInt32Vector2(Triangle.X, Triangle.Z);
			EdgeToTriangleMap.FindOrAdd(Edge1).Add(TriangleIndex);
			EdgeToTriangleMap.FindOrAdd(Edge2).Add(TriangleIndex);
			EdgeToTriangleMap.FindOrAdd(Edge3).Add(TriangleIndex);
		}
		auto GetC = [](const FInt32Vector2& OneEdge, const FInt32Vector3& OneTriangle)
		{
			if (OneEdge == FInt32Vector2(OneTriangle.X, OneTriangle.Y))
			{
				return OneTriangle.Z;
			}
			else if (OneEdge == FInt32Vector2(OneTriangle.X, OneTriangle.Z))
			{
				return OneTriangle.Y;
			}
			else
			{
				return OneTriangle.X;
			}
		};
		TSet<FInt32Vector4> QuadSet;
		TSet<int32> NeedRemoveTriangle;
		for (const TPair<FInt32Vector2, TArray<int32>>& KV : EdgeToTriangleMap)
		{
			const FInt32Vector2& Edge = KV.Key;
			const TArray<int32>& TriangleList = KV.Value;
			if (TriangleList.Num() < 2)
			{
				continue;
			}
			FVector XPosition = GetAbsolutePosition(Edge.X);
			FVector YPosition = GetAbsolutePosition(Edge.Y);
			for (int32 I = 0; I < TriangleList.Num() - 1; ++I)
			{
				const int32& Triangle1Index1 = TriangleList[I];
				const auto& Triangle1 = Triangles[Triangle1Index1];
				int32 C1 = GetC(Edge, Triangle1);
				FVector C1Position = GetAbsolutePosition(C1);
				for (int32 J = I + 1; J < TriangleList.Num(); ++J)
				{
					const int32& Triangle1Index2 = TriangleList[J];
					const auto& Triangle2 = Triangles[Triangle1Index2];
					int32 C2 = GetC(Edge, Triangle2);
					FVector C2Position = GetAbsolutePosition(C2);

					//        C1 +
					//         / \
                    // edge.x +---+ edge.y
					//         \ /
					//        C2 +

					//判断两个三角形的夹角是否够大 够大的夹角无需剔除
					FVector EV = YPosition - XPosition;
					FVector Va = C1Position - XPosition;
					FVector VB = C2Position - XPosition;
					FVector Na = FVector::CrossProduct(Va, EV);
					FVector Nb = FVector::CrossProduct(EV, VB);

					float Len1 = Na.Length();
					float Len2 = Nb.Length();

					float CosSita = FVector::DotProduct(Na, Nb) / (Len1 * Len2);
					CosSita = FMath::Clamp(CosSita, -1.0f, 1.0f);
					float Ang = FMath::Acos(CosSita);
					Ang = FMath::Abs(FMath::RadiansToDegrees(Ang));
					constexpr float ProxyMeshTrianglePairAngle = 20.f;
					if (Ang > ProxyMeshTrianglePairAngle)
					{
						continue;
					}

					// 判断两个三角形是否使用了相同的四边形顶点，相同的顶点只需要一对三角形
					TArray<int32> QuadArray;
					QuadArray.Add(Edge.X);
					QuadArray.Add(Edge.Y);
					QuadArray.Add(C1);
					QuadArray.Add(C2);
					QuadArray.Sort();
					FInt32Vector4 Quad = FInt32Vector4(QuadArray[0], QuadArray[1], QuadArray[2], QuadArray[3]);
					if (QuadSet.Contains(Quad))
					{
						NeedRemoveTriangle.Add(Triangle1Index1);
						NeedRemoveTriangle.Add(Triangle1Index2);
					}
					else
					{
						QuadSet.Add(Quad);
					}
				}
			}
		}
		EdgeSet.Empty();
		for (int32 i = 0; i < Triangles.Num(); ++i)
		{
			if (NeedRemoveTriangle.Contains(i))
			{
				continue;
			}
			EdgeSet.Add(FInt32Vector2(Triangles[i].X, Triangles[i].Y));
			EdgeSet.Add(FInt32Vector2(Triangles[i].X, Triangles[i].Z));
			EdgeSet.Add(FInt32Vector2(Triangles[i].Y, Triangles[i].Z));
		}
		TArray<FInt32Vector2> FinalEdges;
		for (const FInt32Vector2& Edge : EdgeSet)
		{
			FinalEdges.Add(Edge);
		}
		for (const FInt32Vector2& Line : Lines)
		{
			FinalEdges.Add(Line);
		}
		ClothSection.Edges = FinalEdges;

		// 临边三角形合成一个三角形对
		TArray<FTrianglePair> TrianglePairs;
		for (const TPair<FInt32Vector2, TArray<int32>>& KV : EdgeToTriangleMap)
		{
			const FInt32Vector2& Edge = KV.Key;
			const TArray<int32>& TriangleList = KV.Value;
			if (TriangleList.Num() < 2)
			{
				continue;
			}
			FVector XPosition = GetAbsolutePosition(Edge.X);
			FVector YPosition = GetAbsolutePosition(Edge.Y);
			for (int32 I = 0; I < TriangleList.Num() - 1; ++I)
			{
				const int32& TriangleIndex1 = TriangleList[I];
				if (NeedRemoveTriangle.Contains(TriangleIndex1))
				{
					continue;
				}
				const auto& Triangle1 = Triangles[TriangleIndex1];
				int32 C1 = GetC(Edge, Triangle1);
				FVector C1Position = GetAbsolutePosition(C1);
				for (int32 J = I + 1; J < TriangleList.Num(); ++J)
				{
					const int32& TriangleIndex2 = TriangleList[J];
					if (NeedRemoveTriangle.Contains(TriangleIndex2))
					{
						continue;
					}
					const auto& Triangle2 = Triangles[TriangleIndex2];
					int32 C2 = GetC(Edge, Triangle2);
					FVector C2Position = GetAbsolutePosition(C2);

					float RestAngle;
					ELoogPhysicsTriangleSignFlag SignFlag;
					LoogPhysicsMath::CalculateDihedralAngle(C1Position, C2Position, XPosition, YPosition, RestAngle, SignFlag);
					float AngleDegree = FMath::Abs(FMath::RadiansToDegrees(RestAngle));
					// 二面角比较小，说明两个三角形几乎在同一平面上，不需要改成四边形体积
					if (AngleDegree < PhysicsDefine::TriangleBendingMaxAngle)
					{
						auto& TrianglePair = TrianglePairs.AddZeroed_GetRef();
						TrianglePair.V0 = C1;
						TrianglePair.V1 = C2;
						TrianglePair.V2 = Edge.X;
						TrianglePair.V3 = Edge.Y;
						TrianglePair.RestAngleOrVolume = RestAngle;
						TrianglePair.SignFlag = SignFlag;
						continue;
					}
					// 如果二面角法线的夹角几乎等于180度，说明两个三角形几乎平行。这种一般不会出现在不了中，给个提醒看看？
					check(AngleDegree >= PhysicsDefine::TriangleBendingMaxAngle && AngleDegree < 179.f);
					if (AngleDegree >= PhysicsDefine::TriangleBendingMaxAngle && AngleDegree < 179.f)
					{
						LoogPhysicsMath::CalculateVolume(C1Position, C2Position, XPosition, YPosition, RestAngle, SignFlag);
						auto& TrianglePair = TrianglePairs.AddZeroed_GetRef();
						TrianglePair.V0 = C1;
						TrianglePair.V1 = C2;
						TrianglePair.V2 = Edge.X;
						TrianglePair.V3 = Edge.Y;
						TrianglePair.RestAngleOrVolume = RestAngle;
						TrianglePair.SignFlag = SignFlag;
					}
				}
			}
		}
		ClothSection.TrianglePairs = TrianglePairs;
	}
}

void UAnimGraphNode_LoogPhysics::ModifyParticleProperty()
{
	
}
UE_ENABLE_OPTIMIZATION
#undef LOCTEXT_NAMESPACE
