// Copyright LoogLong. All Rights Reserved.
#include "AnimNode_LoogPhysics.h"
#include "Animation/AnimInstanceProxy.h"
#include "PhysicsEngine/PhysicsAsset.h"
#include "PhysicsEngine/SkeletalBodySetup.h"


DECLARE_CYCLE_STAT(TEXT("LoogPhysics_Eval"), STAT_LoogPhysics_Eval, STATGROUP_Anim);
DEFINE_LOG_CATEGORY(LogLoogPhysics);

UE_DISABLE_OPTIMIZATION

int32 bUseMC2Rotation = 0;
FAutoConsoleVariableRef CVarUseMC2Rotation(
	TEXT("lp.bUseMC2Rotation"),
	bUseMC2Rotation,
	TEXT("Whether to use MC2 style rotation calculation in LoogPhysics or not. 0: false, 1: true"),
	ECVF_Default
);

namespace LoogPhysicsMath
{
	constexpr float FrictionMass = 3.0f;
	constexpr float DepthMass = 5.0f;
	constexpr float FixedMass = 50.0f;

	FVector ShiftPosition(const FVector& InOldPosition, const FVector& InOldPivotPosition, const FVector& InShiftVector, const FQuat& InShiftRotation)
	{
		FVector OldPosition = InOldPosition - InOldPivotPosition;
		FVector ShiftedPosition = InShiftRotation.RotateVector(OldPosition) + InShiftVector;
		return ShiftedPosition + InOldPivotPosition;
	}

	float CalcInverseMass(float Friction)
	{
		const float Mass = 1.0f + Friction * FrictionMass;
		return 1.0f / Mass;
	}

	float CalcInverseMass(const float Friction, const float Depth)
	{
		float Mass = 1.0f;
		Mass += Friction * FrictionMass;
		float A = (1.0f - Depth);
		Mass += A * A * DepthMass;
		return 1.0f / Mass;
	}

	float CalcInverseMass(float Friction, float Depth, bool bFixed, float InFixedMass)
	{
		return bFixed ? (1.0f / InFixedMass) : CalcInverseMass(Friction, Depth);
	}

	/** return angle in radians*/
	float AngleBetweenVector(const FVector& V1, const FVector& V2)
	{
		// find angle between V1 and V2
		float Len1 = V1.Length();
		float Len2 = V2.Length();

		float CosSita = FVector::DotProduct(V1, V2) / (Len1 * Len2);
		CosSita = FMath::Clamp(CosSita, -1.0f, 1.0f);
		return FMath::Acos(CosSita);
	}

	bool ClampAngle(const FVector& Dir, const FVector& BaseDir, float MaxAngle, FVector& OutDir)
	{
		if (!bUseMC2Rotation)
		{
			FQuat RotBetween = FQuat::FindBetweenVectors(Dir, BaseDir);
			float DiffAngle = RotBetween.GetAngle();
			if (DiffAngle <= MaxAngle)
			{
				OutDir = Dir;
				return false;
			}
			float T = (DiffAngle - MaxAngle) / DiffAngle;
			FQuat Q = FQuat::Slerp(FQuat::Identity, RotBetween, T);
			OutDir = Q.RotateVector(Dir);
			return true;
		}
		FVector V1 = Dir.GetSafeNormal();
		FVector V2 = BaseDir.GetSafeNormal();

		const float C = FMath::Clamp(V1 | V2, -1.f, 1.f);
		float Angle = FMath::Acos(C);

		if (Angle <= MaxAngle)
		{
			// 当前已经再clamp的范围内，无需clamp
			OutDir = Dir;
			return false;
		}

		float T = (Angle - MaxAngle) / Angle;

		FVector Axis;
		// 当夹角约180°或0° 此时无法通过叉乘得到旋转轴，需要特别处理
		if (FMath::Abs(1.0f + C) < PhysicsDefine::Epsilon)
		{
			Angle = (float)UE_PI;

			if (V1.X > V1.Y && V1.X > V1.Z)
			{
				Axis = V1 ^ FVector::YAxisVector;
			}
			else
			{
				Axis = V1 ^ FVector::XAxisVector;
			}
		}
		else if (FMath::Abs(1.0f - C) < PhysicsDefine::Epsilon)
		{
			OutDir = Dir;
			return false;
		}
		else
		{
			Axis = V1 ^ V2;
		}
		FQuat Q = FQuat(Axis.GetSafeNormal(), Angle * T);

		OutDir = Q * Dir;
		return true;
	}

	FQuat FromToRotationWithoutNormalize(const FVector& V1, const FVector& V2, float T = 1.0f)
	{
		if (!bUseMC2Rotation)
		{
			FQuat RotBetween = FQuat::FindBetweenVectors(V1, V2);
			if (T == 1.0f)
			{
				return RotBetween;
			}
			return FQuat::Slerp(FQuat::Identity, RotBetween, T);
		}
		float C = FMath::Clamp(V1 | V2, -1.f, 1.f);
		float Angle = FMath::Acos(C);

		FVector Axis;
		// 当夹角约180°或0° 此时无法通过叉乘得到旋转轴，需要特别处理
		if (FMath::Abs(1.0f + C) < PhysicsDefine::Epsilon)
		{
			Angle = (float)UE_PI;

			if (V1.X > V1.Y && V1.X > V1.Z)
			{
				Axis = V1 ^ FVector::YAxisVector;
			}
			else
			{
				Axis = V1 ^ FVector::XAxisVector;
			}
		}
		else if (FMath::Abs(1.0f - C) < PhysicsDefine::Epsilon)
		{
			return FQuat::Identity;
		}
		else
		{
			Axis = V1 ^ V2;
		}

		return FQuat(Axis.GetSafeNormal(), Angle * T);
	}

	float ClosestPtPointSegmentRatio(const FVector& C, const FVector& A, const FVector& B)
	{
		FVector AB = B - A;
		// 通过计算参数化位置 d(t) = A + t * (B - A) 将 C 投影到 ab 上
		float Dot = AB | AB;
		// 考虑 A 和 B 相同的坐标
		if (Dot == 0.0f)
			return 0.0f;
	
		float T = ((C - A) | AB) / Dot;
		// 如果它在线段AB之外，则将T限制在AB最近的点。
		T = FMath::Clamp(T, 0.f, 1.f);
		return T;
	}

	FVector Project(const FVector& v, const FVector& n)
	{
		return (v | n ) * n;
	}

	FVector ProjectOnPlane(const FVector& v, const FVector& n)
	{
		return v - FVector::DotProduct(v, n) * n;
	}

	float IntersectPointPlaneDist(const FVector& PlanePos, const FVector& PlaneDir, const FVector& Pos, FVector& OutPos)
	{
		const FVector V = Pos - PlanePos;

		// 挤出矢量
		FVector Gv = Project(V, PlaneDir);
		float Len = Gv.Length();

		if ((PlaneDir | V) < 0.0f)
		{
			// 挤出发生
			// 挤出坐标
			OutPos = Pos - Gv;
			// 返回到面的距离，结果为负数
			return -Len;
		}
		else
		{
			// 无需挤出。什么都不做。
			OutPos = Pos;
			//返回到面的距离（+）
			return Len;
		}
	}

	/**
	 * 
	 * @param P1 线段1的起点
	 * @param Q1 线段1的终点
	 * @param P2 线段2的起点
	 * @param Q2 线段2的终点
	 * @param S 线段1上最近点的参数化位置（0-起点，1-终点）
	 * @param T 线段2上最近点的参数化位置（0-起点，1-终点）
	 * @param C1 线段1上最近点坐标
	 * @param C2 线段2上最近点坐标
	 */
	void ClosestPtSegmentSegment(const FVector& P1, const FVector& Q1, const FVector& P2, const FVector& Q2, float& S, float& T, FVector& C1, FVector& C2)
	{
		// S = 0.0f;
		// T = 0.0f;
		FVector D1 = Q1 - P1; // 线段 s1 的方向向量
		FVector D2 = Q2 - P2; // 线段 s2 的方向向量
		FVector R = P1 - P2;
		float A = D1 | D1; // 线段 s1 的长度平方，始终 >= 0
		float E = D2 | D2; // 线段 s2 的长度平方，始终 >= 0
		float F = D2 | R;
		// 检查任意一条线段是否退化为点
		if (A <= PhysicsDefine::Epsilon && E <= PhysicsDefine::Epsilon)
		{
			// 两条线段都退化为点
			S = 0.0f;
			T = 0.0f;
			C1 = P1;
			C2 = P2;
			return;
		}
		if (A <= PhysicsDefine::Epsilon)
		{
			// 第一条线段退化为点
			S = 0.0f;
			T = FMath::Clamp(F / E, 0.f, 1.f);
		}
		else
		{
			float C = D1 | R;
			if (E <= PhysicsDefine::Epsilon)
			{
				// 第二条线段退化为点
				T = 0.0f;
				S = FMath::Clamp(-C / A, 0.f, 1.f);
			}
			else
			{
				// 一般情况
				float B = D1 | D2;
				float Denom = A * E - B * B; // 非负
				// 如果线段不平行，计算 L1 上对应 L2 的最近点 S 并对 S 限制到 [0,1]；若平行则选择任意 S（此处为 0）
				if (Denom != 0.0f)
				{
					S = FMath::Clamp((B * F - C * E) / Denom, 0.f, 1.f);
				}
				else
				{
					S = 0.0f;
				}
				// 计算 L2 上与 S 对应的 T
				// T = dot((P1 + d1 * S) - P2, d2) / dot(d2, d2) = (b * S + f) / e
				T = (B * S + F) / E;
				// 如果 T 在 [0,1] 范围内则结束。
				// 否则对 T 做限制，并使用新 T 重新计算 S（并将 S 限制到 [0,1]）
				// S = dot((P2 + d2 * T) - P1, d1) / dot(d1, d1) = (T * b - c) / a
				if (T < 0.0f)
				{
					T = 0.0f;
					S = FMath::Clamp(-C / A, 0.f, 1.f);
				}
				else if (T > 1.0f)
				{
					T = 1.0f;
					S = FMath::Clamp((B - C) / A, 0.f, 1.f);
				}
			}
		}

		C1 = P1 + D1 * S;
		C2 = P2 + D2 * T;
	}

	/**
	 * 
	 * @param P1 线段1的起点
	 * @param Q1 线段1的终点
	 * @param P2 线段2的起点
	 * @param Q2 线段2的终点
	 * @param S 线段1上最近点的参数化位置（0-起点，1-终点）
	 * @param T 线段2上最近点的参数化位置（0-起点，1-终点）
	 */
	void ClosestPtSegmentSegment2(const FVector& P1, const FVector& Q1, const FVector& P2, const FVector& Q2, float& S, float& T)
	{
		// S = 0.0f;
		// T = 0.0f;
		FVector D1 = Q1 - P1; // 线段 s1 的方向向量
		FVector D2 = Q2 - P2; // 线段 s2 的方向向量
		FVector R = P1 - P2;
		float A = FVector::DotProduct(D1, D1); // 线段 s1 的长度平方（始终 >= 0）
		float E = FVector::DotProduct(D2, D2); // 线段 s2 的长度平方（始终 >= 0）
		float F = FVector::DotProduct(D2, R);

		// 检查是否有任一线段退化为点
		if (A <= PhysicsDefine::Epsilon && E <= PhysicsDefine::Epsilon)
		{
			// 两条线段均退化为点
			S = 0.0f;
			T = 0.0f;
		}
		else if (A <= PhysicsDefine::Epsilon)
		{
			// 第一条线段退化为点
			S = 0.0f;
			T = FMath::Clamp(F / E, 0.0f, 1.0f);
		}
		else
		{
			float C = FVector::DotProduct(D1, R);
			if (E <= PhysicsDefine::Epsilon)
			{
				// 第二条线段退化为点
				T = 0.0f;
				S = FMath::Clamp(-C / A, 0.0f, 1.0f);
			}
			else
			{
				// 一般情况
				float B = FVector::DotProduct(D1, D2);
				float Denom = A * E - B * B; // 非负
				// 若两线段不平行，计算 L1 上对应 L2 最近点的 S 并对 S 限制到 [0,1]；否则选任意 S（这里选 0）
				if (!FMath::IsNearlyZero(Denom))
				{
					S = FMath::Clamp((B * F - C * E) / Denom, 0.0f, 1.0f);
				}
				else
				{
					S = 0.0f;
				}

				// 计算 L2 上与 S 对应的 T
				T = (B * S + F) / E;

				// 若 T 不在 [0,1]，则对 T 做限制并根据新 T 重新计算 S（并将 S 限制到 [0,1]）
				if (T < 0.0f)
				{
					T = 0.0f;
					S = FMath::Clamp(-C / A, 0.0f, 1.0f);
				}
				else if (T > 1.0f)
				{
					T = 1.0f;
					S = FMath::Clamp((B - C) / A, 0.0f, 1.0f);
				}
			}
		}
	}

	bool CalcVolume(const FVector* nextPosBuffer, const FVector4f& invMassBuffer, float RestVolume, float stiffness, FVector* addPosBuffer)
	{
		FVector nextPos0 = nextPosBuffer[0];
		FVector nextPos1 = nextPosBuffer[1];
		FVector nextPos2 = nextPosBuffer[2];
		FVector nextPos3 = nextPosBuffer[3];

		float invMass0 = invMassBuffer[0];
		float invMass1 = invMassBuffer[1];
		float invMass2 = invMassBuffer[2];
		float invMass3 = invMassBuffer[3];

		float volume = (1.0f / 6.0f) * FVector::DotProduct(FVector::CrossProduct(nextPos1 - nextPos0, nextPos2 - nextPos0), nextPos3 - nextPos0);
		volume *= PhysicsDefine::VolumeScale; // 浮点数误差

		FVector grad0 = FVector::CrossProduct(nextPos1 - nextPos2, nextPos3 - nextPos2);
		FVector grad1 = FVector::CrossProduct(nextPos2 - nextPos0, nextPos3 - nextPos0);
		FVector grad2 = FVector::CrossProduct(nextPos0 - nextPos1, nextPos3 - nextPos1);
		FVector grad3 = FVector::CrossProduct(nextPos1 - nextPos0, nextPos2 - nextPos0);

		float lambda =
			invMass0 * grad0.SquaredLength() +
			invMass1 * grad1.SquaredLength() +
			invMass2 * grad2.SquaredLength() +
			invMass3 * grad3.SquaredLength();
		lambda *= PhysicsDefine::VolumeScale; // 浮点数误差

		if (FMath::Abs(lambda) < PhysicsDefine::Epsilon)
		{
			return false;
		}

		lambda = stiffness * (RestVolume - volume) / lambda;

		addPosBuffer[0] = lambda * invMass0 * grad0;
		addPosBuffer[1] = lambda * invMass1 * grad1;
		addPosBuffer[2] = lambda * invMass2 * grad2;
		addPosBuffer[3] = lambda * invMass3 * grad3;

		return true;
	}

	bool CalcDihedralAngle(
		float sign,
		const FVector* nextPosBuffer,
		const FVector4f& invMassBuffer,
		float restAngle,
		float stiffness,
		FVector* addPosBuffer
	)
	{
		FVector nextPos0 = nextPosBuffer[0];
		FVector nextPos1 = nextPosBuffer[1];
		FVector nextPos2 = nextPosBuffer[2];
		FVector nextPos3 = nextPosBuffer[3];

		float invMass0 = invMassBuffer[0];
		float invMass1 = invMassBuffer[1];
		float invMass2 = invMassBuffer[2];
		float invMass3 = invMassBuffer[3];

		FVector e = nextPos3 - nextPos2;
		float elen = e.Length();
		if (elen < PhysicsDefine::Epsilon)
		{
			return false;
		}

		float invElen = 1.0f / elen;

		FVector n1 = FVector::CrossProduct(nextPos2 - nextPos0, nextPos3 - nextPos0);
		FVector n2 = FVector::CrossProduct(nextPos3 - nextPos1, nextPos2 - nextPos1);

		float n1_lengsq = n1.SquaredLength();
		float n2_lengsq = n2.SquaredLength();

		// 稀に発生する長さ０に対処
		if (n1_lengsq == 0.0f || n2_lengsq == 0.0f)
		{
			return false;
		}
		//Develop.Assert(n1_lengsq > 0.0f);
		//Develop.Assert(n2_lengsq > 0.0f);
		n1 /= n1_lengsq;
		n2 /= n2_lengsq;

		FVector d0 = elen * n1;
		FVector d1 = elen * n2;
		FVector d2 = FVector::DotProduct(nextPos0 - nextPos3, e) * invElen * n1 + FVector::DotProduct(nextPos1 - nextPos3, e) * invElen * n2;
		FVector d3 = FVector::DotProduct(nextPos2 - nextPos0, e) * invElen * n1 + FVector::DotProduct(nextPos2 - nextPos1, e) * invElen * n2;

		n1.Normalize();
		n2.Normalize();
		float dot = FVector::DotProduct(n1, n2);
		dot = FMath::Clamp(dot, -1.f, 1.f);
		float phi = FMath::Acos(dot);

		float lambda =
			invMass0 * d0.SquaredLength() +
			invMass1 * d1.SquaredLength() +
			invMass2 * d2.SquaredLength() +
			invMass3 * d3.SquaredLength();

		if (lambda == 0.0f)
		{
			return false;
		}

		// 方向性
		float dirSign = FVector::DotProduct(FVector::CrossProduct(n1, n2), e) < 0.f ? -1.f : 1.f;
		if (sign != 0)
		{
			// 有向二面角(DirectionDihedralAngle)
			phi *= dirSign;
		}
		else
		{
			// 无向二面角(DihedralAngle)
			lambda *= dirSign;
		}

		lambda = (restAngle - phi) / lambda * stiffness;

		FVector corr0 = -invMass0 * lambda * d0;
		FVector corr1 = -invMass1 * lambda * d1;
		FVector corr2 = -invMass2 * lambda * d2;
		FVector corr3 = -invMass3 * lambda * d3;

		addPosBuffer[0] = corr0;
		addPosBuffer[1] = corr1;
		addPosBuffer[2] = corr2;
		addPosBuffer[3] = corr3;

		return true;
	}
}


float FLoogPhysicsCapsule::PointCapsuleColliderDetection(FVector& InOutNextPos, float Radius, const UE::Geometry::FAxisAlignedBox3d& PointAABB, FVector& OutNormal) const
{
	OutNormal = FVector::Zero();

	// AABB判定
	if (!PointAABB.Intersects(AABB))
		return FLT_MAX;

	FVector StartOldPos = OldStartPosition;
	FVector EndOldOos = OldEndPosition;
	FVector StartPos = NowStartPosition;
	FVector EndPos = NowEndPosition;
	float StartRadius = RadiusA;
	float EndRadius = RadiusB;

	// 计算碰撞器移动前的挤压平面位置
	float T = LoogPhysicsMath::ClosestPtPointSegmentRatio(InOutNextPos, StartOldPos, EndOldOos);
	float R = FMath::Lerp(StartRadius, EndRadius, T);
	FVector D = FMath::Lerp(StartOldPos, EndOldOos, T);
	FVector V = InOutNextPos - D;

	// 运动前碰撞体的局部向量
	FVector lv = InverseOldRot * V;

	// 移动后的变换
	D = FMath::Lerp(StartPos, EndPos, T);
	V = NowRotation * lv;
	FVector N = V.GetSafeNormal();
	FVector C = D + N * (R + Radius);

	// 冲突法线
	OutNormal = N;

	// c = 平面位置
	// n = 平面方向
	// 平面碰撞检测和挤出
	return LoogPhysicsMath::IntersectPointPlaneDist(C, N, InOutNextPos, InOutNextPos);
}

float FLoogPhysicsCapsule::EdgeCapsuleColliderDetection(FVector& InOutNextPosC0, FVector& InOutNextPosC1, const FVector2f& EdgeRadius,
	const UE::Geometry::FAxisAlignedBox3d& EdgeAABB, float CollisionFrictionRange, FVector& OutNormal) const
{
	OutNormal = FVector::Zero();

	// AABB判定
	if (!EdgeAABB.Intersects(AABB))
		return FLT_MAX;

	const FVector& StartOldPos = OldStartPosition;
	const FVector& EndOldPos = OldEndPosition;
	const FVector& StartPos = NowStartPosition;
	const FVector& EndPos = NowEndPosition;
	const float& StartRadius = RadiusA;
	const float& EndRadius = RadiusB;

	// 移动前两条线段的最近点
	float S, T;
	FVector ClosestA, ClosestB;
	LoogPhysicsMath::ClosestPtSegmentSegment(InOutNextPosC0, InOutNextPosC1, StartOldPos, EndOldPos, S, T, ClosestA, ClosestB);
	FVector ClosestVector = ClosestA - ClosestB;
	float ClosestLen = ClosestVector.Length(); 
	if (ClosestLen < PhysicsDefine::Epsilon)
		return FLT_MAX;

	// 挤出法线
	FVector ClosestNormal = ClosestVector / ClosestLen;
	OutNormal = ClosestNormal;

	// ★ 考虑胶囊半径的修正
	// 之前，边缘-胶囊检测假设胶囊在起点和终点的半径相同。
	// 因此，如果胶囊在起点和终点的半径不同，则会发生错误的碰撞检测，导致较大的振动。
	// （当网格边缘比胶囊边缘长时，例如在 BoneCloth 中，这一点尤为明显。）
	// 因此，如果起点和终点的半径不同，则在初始计算中，胶囊边缘会相对于最近点的半径进行偏移。
	// 基于此，再次进行边缘-边缘检测。
	// 虽然并非完美，但这种方法可以提供大致理想的检测结果，并显著减少振动问题。
	// （但是，仍然可能出现轻微振动。）
	if (StartRadius != EndRadius)
	{
		// 沿挤出法线方向移动胶囊体的中心线，并考虑胶囊体半径
		FVector StartOldPos2 = StartOldPos + ClosestNormal * StartRadius;
		FVector EndOldPos2 = EndOldPos + ClosestNormal * EndRadius;

		// 再次计算该线段上的最近点 S/T
		LoogPhysicsMath::ClosestPtSegmentSegment2(InOutNextPosC0, InOutNextPosC1, StartOldPos2, EndOldPos2, S, T);

		// 最后，重写结果以使用移位后的 S/T
		ClosestA = FMath::Lerp(InOutNextPosC0, InOutNextPosC1, S);
		ClosestB = FMath::Lerp(StartOldPos, EndOldPos, T);
		ClosestVector = ClosestA - ClosestB;
		ClosestLen = ClosestVector.Length();
		ClosestNormal = ClosestVector / ClosestLen;
		OutNormal = ClosestNormal;
	}

	// 位移
	FVector Delta0 = StartPos - StartOldPos;
	FVector Delta1 = EndPos - EndOldPos;


	FVector DeltaB = FMath::Lerp(Delta0, Delta1, T);

	float L1 = FVector::DotProduct(ClosestNormal, DeltaB);
	float L = ClosestLen - L1;

	float EdgeRadiusA = FMath::Lerp(EdgeRadius.X, EdgeRadius.Y, S);
	float EdgeRadiusB = FMath::Lerp(StartRadius, EndRadius, T);
	float Thickness = EdgeRadiusA + EdgeRadiusB;

	if (L > (Thickness + CollisionFrictionRange))
		return FLT_MAX;

	// 将当前距离投影到接触法线上
	FVector D = FMath::Lerp(StartPos, EndPos, T);
	ClosestVector = ClosestA - D;
	L = FVector::DotProduct(ClosestNormal, ClosestVector);
	if (L > Thickness)
	{
		// 无接触
		// 返回到接触面的距离
		return L - Thickness;
	}

	// 移动距离
	float C = Thickness - L;

	// 只拉开边缘
	float B0 = 1.0f - S;
	float B1 = S;

	FVector Grad0 = ClosestNormal * B0;
	FVector Grad1 = ClosestNormal * B1;

	float Scale = B0 * B0 + B1 * B1;
	if (Scale == 0.0f)
		return FLT_MAX;

	Scale = C / Scale;

	FVector Corr0 = Scale * Grad0;
	FVector Corr1 = Scale * Grad1;

	InOutNextPosC0 += Corr0;
	InOutNextPosC1 += Corr1;

	// 返回挤出距离
	return -C;
}

FAnimNode_LoogPhysics::FAnimNode_LoogPhysics()
{
}

void FAnimNode_LoogPhysics::Initialize_AnyThread(const FAnimationInitializeContext& Context)
{
	Super::Initialize_AnyThread(Context);
	const FBoneContainer& RequiredBones = Context.AnimInstanceProxy->GetRequiredBones();
	InitializeBoneReferences(RequiredBones);
	InitializeSimulationBuffer();
	InitializeCollider(RequiredBones);
}

void FAnimNode_LoogPhysics::CacheBones_AnyThread(const FAnimationCacheBonesContext& Context)
{
	Super::CacheBones_AnyThread(Context);
}

void FAnimNode_LoogPhysics::ResetDynamics(ETeleportType InTeleportType)
{
	PendingTeleportType = InTeleportType;
}

void FAnimNode_LoogPhysics::UpdateInternal(const FAnimationUpdateContext& Context)
{
	Super::UpdateInternal(Context);
}

void FAnimNode_LoogPhysics::GatherDebugData(FNodeDebugData& DebugData)
{
	Super::GatherDebugData(DebugData);
}

void FAnimNode_LoogPhysics::EvaluateSkeletalControl_AnyThread(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	SCOPE_CYCLE_COUNTER(STAT_LoogPhysics_Eval);
	const auto& RequiredBones = Output.AnimInstanceProxy->GetRequiredBones();
	

	// pre simulation time update
	FrameTimeUpdate(Output);

	AlwaysTeamUpdate(Output);

	AlwaysWindUpdate(Output);

	// read bone transform to update collider base transform
	ReadBoneTransform(Output, RequiredBones);


	{
		// read bone transform to update proxy mesh
		SimulationPreProxyMeshUpdate(Output, RequiredBones);

		// 计算team的中心、全局惯性量、计算风场
		SimulationCalcCenterAndInertiaAndWind(Output);

		// 应用粒子全局惯性
		SimulationPreTeamUpdate();

		// 碰撞体的基准位置更新，和应用全局惯性
		ColliderSimulationPreUpdate();

		for (int32 i = 0; i < SimStepCount; ++i)
		{
			// 计算每步时间和每步的插值比例，更新team的局部惯性，更新风
			SimulationStepTeamUpdate();

			// 应用局部惯性给碰撞体，并且预计算每步需要用到的数据
			ColliderSimulationStartStep();

			// 应用局部惯性给粒子，并使用前向欧拉积分粒子位置
			SimulationStepUpdateParticles();

			// TODO 混合动画姿势 当前全用动画姿势
			SimulationStepUpdateBaseLinePose();

			// root与其所有递归的child直接的距离约束
			TetherConstraintSolverConstraint();

			// particle与其直接相连(代理网格中)的particle间的距离约束
			DistanceConstraintSolverConstraint();

			// base line的角度约束+角度限制
			AngleConstraintSolverConstraint();

			SimulationClearTempBuffer();

			TriangleBendingConstraintSolverConstraint();

			TriangleBendingConstraintSumConstraint();

			// 碰撞检测与响应
			SolveColliderCollisionConstraint();

			// particle与其直接相连(代理网格中)的particle间的距离约束
			DistanceConstraintSolverConstraint();

			MotionConstraintSolverConstraint();

			SimulationStepPostTeam();

			ColliderManagerSimulationEndStep();
		}

		// 模拟完成后计算显示位置: 将模拟完成后的位置转换为代理网格的位置
		SimulationCalcDisplayPosition();

		// 在知道新的代理网格的位置后，通过基线，重新计算代理网格的旋转
		VirtualMeshManagerSimulationPostProxyMeshUpdateLine();

		// TODO 在知道新的代理网格的位置后，通过三角形，重新计算代理网格的旋转：
		VirtualMeshManagerSimulationPostProxyMeshUpdateTriangle();
		VirtualMeshManagerSimulationPostProxyMeshUpdateTriangleSum();

		// 使用代理网格更细bone transform
		VirtualMeshManagerSimulationPostProxyMeshUpdateWorldTransform();
		VirtualMeshManagerSimulationPostProxyMeshUpdateLocalTransform();

		ColliderManagerSimulationPostUpdate();
		TeamManagerSimulationPostTeamUpdate();
	}


	WriteBackTransforms(Output, OutBoneTransforms);
}

bool FAnimNode_LoogPhysics::IsValidToEvaluate(const USkeleton* Skeleton, const FBoneContainer& RequiredBones)
{
	for (auto& Particle : Particles)
	{
		if (!Particle.BindBone.IsValidToEvaluate(RequiredBones))
		{
			return false;
		}
	}
	return true;
}

void FAnimNode_LoogPhysics::InitializeBoneReferences(const FBoneContainer& RequiredBones)
{
	for (auto& Particle : Particles)
	{
		Particle.BindBone.Initialize(RequiredBones);
	}
}


void FAnimNode_LoogPhysics::InitializeSimulationBuffer()
{
	StepBasicPositionBuffer.SetNumZeroed(Particles.Num());
	StepBasicRotationBuffer.SetNumZeroed(Particles.Num());
	LengthBufferArray.SetNumZeroed(Particles.Num());
	LocalPosBufferArray.SetNumZeroed(Particles.Num());
	LocalRotBufferArray.SetNumZeroed(Particles.Num());
	RotationBufferArray.SetNumZeroed(Particles.Num());
	RestorationVectorBufferArray.SetNumZeroed(Particles.Num());

	tempVectorBufferA.SetNumZeroed(Particles.Num());
	tempRotationBufferA.SetNumZeroed(Particles.Num());
	tempVectorBufferB.SetNumZeroed(Particles.Num());
	tempCountBuffer.SetNumZeroed(Particles.Num());
	tempFloatBufferA.SetNumZeroed(Particles.Num());
}

void FAnimNode_LoogPhysics::InitializeCollider(const FBoneContainer& RequiredBones)
{
	for (auto& ClothTeam : ClothTeams)
	{
		bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr;
		if (!bParameterValid)
		{
			continue;
		}
		const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();
		if (UPhysicsAsset* PhysicsAsset = PhysicsParameter.PhysicsAsset.Get())
		{
			for (const TObjectPtr<USkeletalBodySetup>& SkeletalBodySetup : PhysicsAsset->SkeletalBodySetups)
			{
				for (const auto& TaperedCapsuleElem : SkeletalBodySetup->AggGeom.TaperedCapsuleElems)
				{
					FLoogPhysicsCapsule Capsule;
					Capsule.BindBone.BoneName = SkeletalBodySetup->BoneName;
					Capsule.BindBone.Initialize(RequiredBones);
					if (!Capsule.BindBone.IsValidToEvaluate(RequiredBones))
					{
						continue;
					}
					// 所有物理资产中的胶囊体都是Z轴方向
					FVector Dir = FVector::ZAxisVector;
					float HalfLength = TaperedCapsuleElem.Length * 0.5f;
					Capsule.CenterA = Dir * HalfLength;
					Capsule.RadiusA = TaperedCapsuleElem.Radius0;
					Capsule.CenterB = - Dir * HalfLength;
					Capsule.RadiusB = TaperedCapsuleElem.Radius1;
					Capsule.LocalTransform = TaperedCapsuleElem.GetTransform();
					Capsule.TaperedCapsuleElem = TaperedCapsuleElem;
					ClothTeam.CapsuleColliders.Add(Capsule);
				}
			}
		}
	}
}

void FAnimNode_LoogPhysics::FrameTimeUpdate(FComponentSpacePoseContext& Output)
{
	constexpr int32 DefaultSimulationFrequency = 90;
	int32 FrameRate = FMath::Clamp(SimulationFrameRate, MinFrameRate, MaxFrameRate);
	SimulationDeltaTime = 1.0f / FrameRate;
	float T = DefaultSimulationFrequency / static_cast<float>(FrameRate);
	SimulationPower = FVector4(T,           // (3.0 ~ 1.0 ~ 0.6)
		T > 1.0f ? FMath::Pow(T, 0.5f) : T, // (1.73 ~ 1.0 ~ 0.6)
		T > 1.0f ? FMath::Pow(T, 0.3f) : T, // (1.39 ~ 1.0 ~ 0.6)
		//FMath::Pow(t, 1.5f) // (5.19 ~ 1.0 ~ 0.46)
		FMath::Pow(T, 1.8f) // (7.22 ~ 1.0 ~ 0.39)
	);
}

void FAnimNode_LoogPhysics::AlwaysTeamUpdate(const FComponentSpacePoseContext& Output)
{
	FrameDeltaTime = Output.AnimInstanceProxy->GetDeltaSeconds();
	if (bNeedResetSimulation)
	{
		CurrentTickTime = 0;
		PrevTickTime = 0;

		PrevFrameUpdateTime = 0.f;
		FrameUpdateTime = 0.f;

		NowUpdateTime = 0;
		OldUpdateTime = 0.f;
	}
	float TickTime = CurrentTickTime + FrameDeltaTime;
	int32 DesiredSimulateCount = static_cast<int32>((TickTime - NowUpdateTime) / SimulationDeltaTime);
	SimStepCount = FMath::Min(DesiredSimulateCount, MaxSimulationPerFrame);
	SkipCount = DesiredSimulateCount - SimStepCount;
	if (SkipCount > 0)
	{
		TickTime = TickTime - SimulationDeltaTime * SkipCount;
	}
	if (SimStepCount > 0 && FrameDeltaTime == 0.f)
	{
		SimStepCount = 0;
		SkipCount = 0;
		NowUpdateTime = TickTime - SimulationDeltaTime + 0.0001f;
	}
	if (SimStepCount > 0)
	{
		PrevFrameUpdateTime = FrameUpdateTime;
		FrameUpdateTime = TickTime;
		OldUpdateTime = NowUpdateTime;
	}
	PrevTickTime = CurrentTickTime;
	CurrentTickTime = TickTime;
}

void FAnimNode_LoogPhysics::AlwaysWindUpdate(FComponentSpacePoseContext& Output)
{
	//TODO Wind
}

void FAnimNode_LoogPhysics::ReadBoneTransform(FComponentSpacePoseContext& Output, const FBoneContainer& RequiredBones)
{
	// 先读取local space transform 不会触发transform的计算
	for (auto& Particle : Particles)
	{
		FCompactPoseBoneIndex BoneIndex = Particle.BindBone.GetCompactPoseIndex(RequiredBones);
		const auto& BoneLocalTransform = Output.Pose.GetLocalSpaceTransform(BoneIndex);
		switch (Particle.ParticleType)
		{
		case ELoogPhysicsParticleType::FixedBone:
		case ELoogPhysicsParticleType::MovedBone:
			Particle.BoneLocalPosition = BoneLocalTransform.GetTranslation();
			Particle.BoneLocalRotation = BoneLocalTransform.GetRotation();
			break;
		case ELoogPhysicsParticleType::VirtualBone:
			Particle.BoneLocalPosition = Particle.LocalRefPosition;
			Particle.BoneLocalRotation = FQuat::Identity;
			break;
		case ELoogPhysicsParticleType::Max:
		default:
			break;
		}
	}

	// 后读取component space transform,需要从root开始计算transform。如果交替读取会有很大的性能损失
	const auto& ComponentTransform = Output.AnimInstanceProxy->GetComponentTransform();
	for (auto& Particle : Particles)
	{
		FCompactPoseBoneIndex BoneIndex = Particle.BindBone.GetCompactPoseIndex(RequiredBones);
		const auto& BoneTransform = Output.Pose.GetComponentSpaceTransform(BoneIndex) * ComponentTransform;
		switch (Particle.ParticleType)
		{
		case ELoogPhysicsParticleType::FixedBone:
		case ELoogPhysicsParticleType::MovedBone:
			Particle.BonePosition = BoneTransform.GetTranslation();
			Particle.BoneRotation = BoneTransform.GetRotation();
			break;
		case ELoogPhysicsParticleType::VirtualBone:
			Particle.BonePosition = BoneTransform.TransformPosition(Particle.LocalRefPosition);
			Particle.BoneRotation = BoneTransform.GetRotation();
			break;
		case ELoogPhysicsParticleType::Max:
		default:
			break;
		}
	}

	for (auto& ClothTeam : ClothTeams)
	{
		for (int32 Index = 0; Index < ClothTeam.CapsuleColliders.Num(); ++Index)
		{
			auto& RuntimeCollider = ClothTeam.CapsuleColliders[Index];

			FTransform BoneComponentSpaceTransform = Output.Pose.GetComponentSpaceTransform(RuntimeCollider.BindBone.GetCompactPoseIndex(RequiredBones));
			RuntimeCollider.BoneTransform = BoneComponentSpaceTransform * ComponentTransform;
		}
	}
}

void FAnimNode_LoogPhysics::SimulationPreProxyMeshUpdate(FComponentSpacePoseContext& Output, const FBoneContainer& RequiredBones)
{
	for (auto& Particle : Particles)
	{
		Particle.VirtualMeshPosition = Particle.BonePosition;
		Particle.VirtualMeshRotation = Particle.BoneRotation;
	}
}

void FAnimNode_LoogPhysics::SimulationCalcCenterAndInertiaAndWind(FComponentSpacePoseContext& Output)
{
	const auto& ComponentTransform = Output.AnimInstanceProxy->GetComponentTransform();
	for (auto& ClothTeam : ClothTeams)
	{
		bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr ;
		if (!bParameterValid)
		{
			continue;
		}
		const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();
		
		auto& ClothTeamData = ClothTeam.TeamData;
		const FCompactPoseBoneIndex RootBoneIndex = FCompactPoseBoneIndex(0);
		const FTransform RootBoneTransform = Output.Pose.GetComponentSpaceTransform(RootBoneIndex) * ComponentTransform;
		FVector ComponentWorldPos = RootBoneTransform.GetTranslation();
		FQuat ComponentWorldRot = RootBoneTransform.GetRotation();
		ClothTeamData.ComponentWorldPosition = ComponentWorldPos;
		ClothTeamData.ComponentWorldRotation = ComponentWorldRot;

		FVector OldComponentWorldPosition = ClothTeamData.OldComponentWorldPosition;
		FQuat OldComponentWorldRotation = ClothTeamData.OldComponentWorldRotation;

		FVector CenterWorldPosition = ComponentWorldPos;
		FQuat CenterWorldRotation = ComponentWorldRot;

		CalculateCenterTransform(ClothTeam, CenterWorldPosition, CenterWorldRotation);

		// handle world inertia
		// FVector FrameDeltaVector = ComponentWorldPos - OldComponentWorldPosition;

		ClothTeamData.FrameWorldPosition = CenterWorldPosition;
		ClothTeamData.FrameWorldRotation = CenterWorldRotation;
		if (bNeedResetSimulation)
		{
			ClothTeamData.OldComponentWorldPosition = ComponentWorldPos;
			ClothTeamData.OldComponentWorldRotation = ComponentWorldRot;

			OldComponentWorldPosition = ComponentWorldPos;
			OldComponentWorldRotation = ComponentWorldRot;

			ClothTeamData.OldFrameWorldPosition = CenterWorldPosition;
			ClothTeamData.OldFrameWorldRotation = CenterWorldRotation;

			ClothTeamData.PrevStepPosition = CenterWorldPosition;
			ClothTeamData.PrevStepRotation = CenterWorldRotation;

			ClothTeamData.NowStepPosition = CenterWorldPosition;
			ClothTeamData.NowStepRotation = CenterWorldRotation;
		}
		FVector WorkOldComponentPosition = OldComponentWorldPosition;
		FQuat WorkOldComponentRotation = OldComponentWorldRotation;
		if (bNeedResetSimulation)
		{
			ClothTeamData.bIsInertiaShift = false;
			ClothTeamData.FrameComponentShiftVector = FVector::Zero();
			ClothTeamData.FrameComponentShiftRotation = FQuat::Identity;
		}
		else
		{
			ClothTeamData.FrameComponentShiftVector = ComponentWorldPos - OldComponentWorldPosition;
			ClothTeamData.FrameComponentShiftRotation = ComponentWorldRot * OldComponentWorldRotation.Inverse();

			float MoveShiftRatio = 0.0f;
			float RotationShiftRatio = 0.0f;

			const auto& WorldInertia = PhysicsParameter.WorldInertia;
			float TranslationShift = 1.0f - WorldInertia;
			float RotationShift = 1.0f - WorldInertia;

			bool bIsInertiaShift = false;
			if (TranslationShift > PhysicsDefine::Epsilon || RotationShift > PhysicsDefine::Epsilon)
			{
				bIsInertiaShift = true;
				MoveShiftRatio = TranslationShift;
				RotationShiftRatio = RotationShift;

				WorkOldComponentPosition = FMath::Lerp(WorkOldComponentPosition, ComponentWorldPos, TranslationShift);
				WorkOldComponentRotation = FQuat::Slerp(WorkOldComponentRotation, ComponentWorldRot, RotationShift);
			}

			// 最大移动速度限制
			const float& MovementSpeedLimit = PhysicsParameter.WorldTranslationSpeedLimit;
			const float& rotationSpeedLimit = PhysicsParameter.WorldRotationSpeedLimit;
			FVector DeltaTranslation = ComponentWorldPos - WorkOldComponentPosition;
			FQuat DeltaRotation = ComponentWorldRot * WorkOldComponentRotation.Inverse();
			float FrameSpeed = FrameDeltaTime > PhysicsDefine::Epsilon ? DeltaTranslation.Length() / FrameDeltaTime : 0.f;
			if (FrameSpeed > MovementSpeedLimit && rotationSpeedLimit > 0.f)
			{
				bIsInertiaShift = true;
				float ShiftAlpha = FMath::Clamp((FrameSpeed - rotationSpeedLimit) / FrameSpeed, 0.f, 1.f);
				MoveShiftRatio = FMath::Lerp(MoveShiftRatio, 1.0f, ShiftAlpha);
			}
			float DeltaRotationAngle = FMath::RadiansToDegrees(DeltaRotation.GetAngle());
			float FrameRotationSpeed = FrameDeltaTime > PhysicsDefine::Epsilon ? DeltaRotationAngle / FrameDeltaTime : 0.f;
			if (FrameRotationSpeed > MovementSpeedLimit && rotationSpeedLimit > 0.f)
			{
				bIsInertiaShift = true;
				float ShiftAlpha = FMath::Clamp((FrameRotationSpeed - rotationSpeedLimit) / FrameRotationSpeed, 0.f, 1.f);
				RotationShiftRatio = FMath::Lerp(RotationShiftRatio, 1.0f, ShiftAlpha);
			}

			// 其他影响因素
			float OtherShiftRatio = 0.f;
			// 由于更新跳过引起的偏移
			// 当发生更新跳过时，会在跳过时间内执行世界惯性偏移。
			if (SkipCount > 0)
			{
				OtherShiftRatio = FMath::Lerp(OtherShiftRatio, 1.0f, FMath::Clamp((SkipCount * SimulationDeltaTime) / FrameDeltaTime, 0.f, 1.f));
			}
			if (ClothTeamData.VelocityWeight < 1.f)
			{
				OtherShiftRatio = FMath::Lerp(OtherShiftRatio, 1.0f, 1.0f - ClothTeamData.VelocityWeight);
			}
			if (OtherShiftRatio > 0.f)
			{
				bIsInertiaShift = true;
				MoveShiftRatio = FMath::Lerp(MoveShiftRatio, 1.0f, OtherShiftRatio);
				RotationShiftRatio = FMath::Lerp(RotationShiftRatio, 1.0f, OtherShiftRatio);
			}
			ClothTeamData.bIsInertiaShift = bIsInertiaShift;
			if (bIsInertiaShift)
			{
				ClothTeamData.FrameComponentShiftVector *= MoveShiftRatio;
				ClothTeamData.FrameComponentShiftRotation = FQuat::Slerp(FQuat::Identity, ClothTeamData.FrameComponentShiftRotation, RotationShift);

				// component shift center data
				ClothTeamData.OldFrameWorldPosition = LoogPhysicsMath::ShiftPosition(ClothTeamData.OldFrameWorldPosition, ClothTeamData.OldComponentWorldPosition, ClothTeamData.FrameComponentShiftVector, ClothTeamData.FrameComponentShiftRotation);
				ClothTeamData.OldFrameWorldRotation = ClothTeamData.FrameComponentShiftRotation * ClothTeamData.OldFrameWorldRotation;

				ClothTeamData.NowStepPosition = LoogPhysicsMath::ShiftPosition(ClothTeamData.NowStepPosition, ClothTeamData.OldComponentWorldPosition, ClothTeamData.FrameComponentShiftVector, ClothTeamData.FrameComponentShiftRotation);
				ClothTeamData.NowStepRotation = ClothTeamData.FrameComponentShiftRotation * ClothTeamData.NowStepRotation;
			}
		}

		if (bNeedResetSimulation)
		{
			ClothTeamData.VelocityWeight = PhysicsParameter.StablizationTimeAfterReset > PhysicsDefine::Epsilon ? 0.f : 1.0f;
			ClothTeamData.BlendWeight = ClothTeamData.VelocityWeight;
		}

		// TODO Wind
	}
}

void FAnimNode_LoogPhysics::SimulationPreTeamUpdate()
{
	for (auto& ClothTeam : ClothTeams)
	{
		if (bNeedResetSimulation)
		{
			for (int32 ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
			{
				FLoogPhysicsParticle& Particle = Particles[ParticleIndex];
				Particle.NextPosition = Particle.VirtualMeshPosition;
				Particle.OldPosition = Particle.VirtualMeshPosition;
				Particle.OldRotation = Particle.VirtualMeshRotation;

				Particle.PrevBasePosition = Particle.VirtualMeshPosition;
				Particle.PrevBaseRotation = Particle.VirtualMeshRotation;

				Particle.StepBasePosition = Particle.VirtualMeshPosition;
				Particle.StepBaseRotation = Particle.VirtualMeshRotation;

				Particle.VelocityPosition = Particle.VirtualMeshPosition;
				Particle.DisplayPosition = Particle.VirtualMeshPosition;

				Particle.Velocity = FVector::Zero();
				Particle.RealVelocity = FVector::Zero();

				Particle.Friction = 0.f;
				Particle.StaticFriction = 0.f;
				Particle.CollisionNormal = FVector::Zero();
			}
		}
		else if (ClothTeam.TeamData.bIsInertiaShift)
		{
			auto& ClothTeamData = ClothTeam.TeamData;
			for (int32 ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
			{
				FLoogPhysicsParticle& Particle = Particles[ParticleIndex];
				FVector OldPos = Particle.OldPosition;
				FQuat OldRot = Particle.OldRotation;
				FVector OldBasePosition = Particle.PrevBasePosition;
				FQuat OldBaseRotation = Particle.PrevBaseRotation;
				FVector DisplayPos = Particle.DisplayPosition;
				FVector Velocity = Particle.Velocity;
				FVector RealVelocity = Particle.RealVelocity;

				OldPos = LoogPhysicsMath::ShiftPosition(OldPos, ClothTeamData.OldComponentWorldPosition, ClothTeamData.FrameComponentShiftVector, ClothTeamData.FrameComponentShiftRotation);
				OldRot = ClothTeamData.FrameComponentShiftRotation * OldRot;

				OldBasePosition = LoogPhysicsMath::ShiftPosition(OldBasePosition, ClothTeamData.OldComponentWorldPosition, ClothTeamData.FrameComponentShiftVector, ClothTeamData.FrameComponentShiftRotation);
				OldBaseRotation = ClothTeamData.FrameComponentShiftRotation * OldBaseRotation;

				DisplayPos = LoogPhysicsMath::ShiftPosition(DisplayPos, ClothTeamData.OldComponentWorldPosition, ClothTeamData.FrameComponentShiftVector, ClothTeamData.FrameComponentShiftRotation);

				Velocity = ClothTeamData.FrameComponentShiftRotation * Velocity;
				RealVelocity = ClothTeamData.FrameComponentShiftRotation * RealVelocity;


				Particle.OldPosition = OldPos;
				Particle.OldRotation = OldRot;
				Particle.PrevBasePosition = OldBasePosition;
				Particle.PrevBaseRotation = OldBaseRotation;
				Particle.DisplayPosition = DisplayPos;
				Particle.Velocity = Velocity;
				Particle.RealVelocity = RealVelocity;
			}
		}
	}
}

void FAnimNode_LoogPhysics::ColliderSimulationPreUpdate()
{
	for (auto& ClothTeam : ClothTeams)
	{
		if (ClothTeam.CapsuleColliders.Num() == 0)
		{
			continue;
		}
		auto& ClothTeamData = ClothTeam.TeamData;
		for (int32 ColliderIndex = 0; ColliderIndex < ClothTeam.CapsuleColliders.Num(); ++ColliderIndex)
		{
			auto& RuntimeCollider = ClothTeam.CapsuleColliders[ColliderIndex];
			FTransform FrameBaseTransSimSpace = RuntimeCollider.LocalTransform * RuntimeCollider.BoneTransform;
			RuntimeCollider.FrameBasePosition = FrameBaseTransSimSpace.GetTranslation();
			RuntimeCollider.FrameBaseRotation = FrameBaseTransSimSpace.GetRotation();

			if (bNeedResetSimulation)
			{
				RuntimeCollider.OldFrameBasePosition = RuntimeCollider.FrameBasePosition;
				RuntimeCollider.OldFrameBaseRotation = RuntimeCollider.FrameBaseRotation;

				RuntimeCollider.NowPosition = RuntimeCollider.FrameBasePosition;
				RuntimeCollider.NowRotation = RuntimeCollider.FrameBaseRotation;

				RuntimeCollider.OldPosition = RuntimeCollider.FrameBasePosition;
				RuntimeCollider.OldRotation = RuntimeCollider.FrameBaseRotation;
			}
			else if (ClothTeam.TeamData.bIsInertiaShift)
			{
				const FVector& PrevFrameWorldPosition = ClothTeamData.OldComponentWorldPosition;
				RuntimeCollider.OldFrameBasePosition = LoogPhysicsMath::ShiftPosition(RuntimeCollider.OldFrameBasePosition, PrevFrameWorldPosition, ClothTeamData.FrameComponentShiftVector, ClothTeamData.FrameComponentShiftRotation);
				RuntimeCollider.OldFrameBaseRotation = ClothTeamData.FrameComponentShiftRotation * RuntimeCollider.OldFrameBaseRotation;

				RuntimeCollider.NowPosition = LoogPhysicsMath::ShiftPosition(RuntimeCollider.NowPosition, PrevFrameWorldPosition, ClothTeamData.FrameComponentShiftVector, ClothTeamData.FrameComponentShiftRotation);
				RuntimeCollider.NowRotation = ClothTeamData.FrameComponentShiftRotation * RuntimeCollider.NowRotation;

				RuntimeCollider.OldPosition = LoogPhysicsMath::ShiftPosition(RuntimeCollider.OldPosition, PrevFrameWorldPosition, ClothTeamData.FrameComponentShiftVector, ClothTeamData.FrameComponentShiftRotation);
				RuntimeCollider.OldRotation = ClothTeamData.FrameComponentShiftRotation * RuntimeCollider.OldRotation;
			}
		}
	}
}

void FAnimNode_LoogPhysics::SimulationStepTeamUpdate()
{
	// 时间更新
	NowUpdateTime += SimulationDeltaTime;
	float DeltaTimeFromFrameStart = CurrentTickTime - PrevFrameUpdateTime;
	FrameInterpolation = DeltaTimeFromFrameStart > 0 ? (NowUpdateTime - PrevFrameUpdateTime) / DeltaTimeFromFrameStart : 1.0f;
	FrameInterpolation = FMath::Clamp(FrameInterpolation, 0.f, 1.f);

	for (auto& ClothTeam : ClothTeams)
	{
		bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr;
		if (!bParameterValid)
		{
			continue;
		}
		const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();

		// update team inertial data
		auto& ClothTeamData = ClothTeam.TeamData;
		ClothTeamData.PrevStepPosition = ClothTeamData.NowStepPosition;
		ClothTeamData.PrevStepRotation = ClothTeamData.NowStepRotation;
		ClothTeamData.NowStepPosition = FMath::Lerp(ClothTeamData.OldFrameWorldPosition, ClothTeamData.FrameWorldPosition, FrameInterpolation);
		ClothTeamData.NowStepRotation = FQuat::Slerp(ClothTeamData.OldFrameWorldRotation, ClothTeamData.FrameWorldRotation, FrameInterpolation);

		// from old to new
		ClothTeamData.StepVector = ClothTeamData.NowStepPosition - ClothTeamData.PrevStepPosition;
		ClothTeamData.StepRotation = ClothTeamData.NowStepRotation * ClothTeamData.PrevStepRotation.Inverse();

		// calculate local inertia
		const auto& LocalInertia = PhysicsParameter.LocalInertia;
		float LocalMovementInertia = 1.0f - LocalInertia;
		float LocalRotationInertia = 1.0f - LocalInertia;

		// speed limit
		FVector LocalVector = ClothTeamData.StepVector * LocalInertia;
		float localMovementSpeed = LocalVector.Length() / SimulationDeltaTime; // CM/S
		if (localMovementSpeed > PhysicsParameter.LocalMovementSpeedLimit && PhysicsParameter.LocalMovementSpeedLimit >= 0.0f)
		{
			float T = PhysicsParameter.LocalMovementSpeedLimit / localMovementSpeed;
			LocalMovementInertia = FMath::Lerp(1.0f, LocalMovementInertia, T);
		}

		float stepAngle = ClothTeamData.StepRotation.GetAngle();
		float localAngle = stepAngle * LocalInertia;
		float localAngleSpeed = FMath::RadiansToDegrees(localAngle / SimulationDeltaTime); 
		if (localAngleSpeed > PhysicsParameter.LocalRotationSpeedLimit && PhysicsParameter.LocalRotationSpeedLimit >= 0.0f)
		{
			float t = PhysicsParameter.LocalRotationSpeedLimit / localAngleSpeed;
			LocalRotationInertia = FMath::Lerp(1.0f, LocalRotationInertia, t);
		}

		ClothTeamData.StepTranslationInertiaRatio = LocalMovementInertia;
		ClothTeamData.StepRotationInertiaRatio = LocalRotationInertia;

		ClothTeamData.InertiaVector = ClothTeamData.StepVector * LocalMovementInertia;
		ClothTeamData.InertiaRotation = FQuat::Slerp(FQuat::Identity, ClothTeamData.StepRotation, LocalRotationInertia);
		// UE_LOG(LogAnimation, Warning, TEXT("InertiaVector:{%f, %f, %f}"), ClothTeamData.InertiaVector.X, ClothTeamData.InertiaVector.Y, ClothTeamData.InertiaVector.Z);

		// 计算离心作用参数
		ClothTeamData.AngularVelocity = stepAngle / SimulationDeltaTime;
		if (ClothTeamData.AngularVelocity > PhysicsDefine::Epsilon)
		{
			ClothTeamData.RotationAxis = ClothTeamData.StepRotation.GetRotationAxis();
		}
		else
		{
			ClothTeamData.RotationAxis = FVector::Zero();
		}

		float GravityDot = 1.0f;
		// if (PhysicsParameter.GravityDirection.Length() > PhysicsDefine::Epsilon)
		// {
		// 	FVector initLocalGravityDirection = ;
		//
		// 	FVector worldFalloffDir = ClothTeamData.NowStepRotation * initLocalGravityDirection;
		// 	GravityDot = math.dot(worldFalloffDir, PhysicsParameter.GravityDirection);
		// 	GravityDot = FMath::Clamp(GravityDot * 0.5f + 0.5f, 0.f, 1.f);
		// }
		ClothTeamData.GravityDot = GravityDot;

		float GravityRatio = 1.f;
		if (PhysicsParameter.GravityMagnitude > PhysicsDefine::Epsilon && PhysicsParameter.GravityFalloff > PhysicsDefine::Epsilon)
		{
			GravityRatio = FMath::Lerp(FMath::Clamp(1.0f - PhysicsParameter.GravityFalloff, 0.f, 1.f), 1.0f, FMath::Clamp(1.0f - GravityDot, 0.f, 1.f));
		}
		ClothTeamData.GravityRatio = GravityRatio;

		if (ClothTeamData.VelocityWeight < 1.0f)
		{
			float AddWeight = PhysicsParameter.StablizationTimeAfterReset > PhysicsDefine::Epsilon ? SimulationDeltaTime / PhysicsParameter.StablizationTimeAfterReset : 1.0f;
			ClothTeamData.VelocityWeight = FMath::Clamp(ClothTeamData.VelocityWeight + AddWeight, 0.f, 1.f);
		}

		ClothTeamData.BlendWeight = FMath::Clamp(ClothTeamData.VelocityWeight, 0.f, 1.f);
		// TODO wind Update
	}

}

void FAnimNode_LoogPhysics::ColliderSimulationStartStep()
{
	for (auto& ClothTeam : ClothTeams)
	{
		bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr;
		if (!bParameterValid)
		{
			continue;
		}
		// const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();

		auto& ClothTeamData = ClothTeam.TeamData;
		for (int32 ColliderIndex = 0; ColliderIndex < ClothTeam.CapsuleColliders.Num(); ++ColliderIndex)
		{
			auto& RuntimeCollider = ClothTeam.CapsuleColliders[ColliderIndex];

			RuntimeCollider.NowPosition = FMath::Lerp(RuntimeCollider.OldFrameBasePosition, RuntimeCollider.FrameBasePosition, FrameInterpolation);
			RuntimeCollider.NowRotation = FQuat::Slerp(RuntimeCollider.OldFrameBaseRotation, RuntimeCollider.FrameBaseRotation, FrameInterpolation);

			// 应用局部惯性
			RuntimeCollider.OldPosition = FMath::Lerp(RuntimeCollider.OldPosition, RuntimeCollider.NowPosition, ClothTeamData.StepTranslationInertiaRatio);
			RuntimeCollider.OldRotation = FQuat::Slerp(RuntimeCollider.OldRotation, RuntimeCollider.NowRotation, ClothTeamData.StepRotationInertiaRatio);

			FVector NowStartPosition = RuntimeCollider.NowPosition + RuntimeCollider.NowRotation.RotateVector(RuntimeCollider.CenterA);
			FVector NowEndPosition = RuntimeCollider.NowPosition + RuntimeCollider.NowRotation.RotateVector(RuntimeCollider.CenterB);

			FVector OldStartPosition = RuntimeCollider.OldPosition + RuntimeCollider.OldRotation.RotateVector(RuntimeCollider.CenterA);
			FVector OldEndPosition = RuntimeCollider.OldPosition + RuntimeCollider.OldRotation.RotateVector(RuntimeCollider.CenterB);
			// 预先创建一些数据
			UE::Geometry::FAxisAlignedBox3d aabb1{ FVector::Min(OldStartPosition, NowStartPosition) - RuntimeCollider.RadiusA, FVector::Max(OldStartPosition, NowStartPosition) + RuntimeCollider.RadiusA };
			UE::Geometry::FAxisAlignedBox3d aabb2{ FVector::Min(OldEndPosition, NowEndPosition) - RuntimeCollider.RadiusB, FVector::Max(OldEndPosition, NowEndPosition) + RuntimeCollider.RadiusB };
			aabb1.Contain(aabb2);
			RuntimeCollider.AABB = aabb1;

			RuntimeCollider.InverseOldRot = RuntimeCollider.OldRotation.Inverse();

			RuntimeCollider.NowStartPosition = NowStartPosition;
			RuntimeCollider.NowEndPosition = NowEndPosition;
			RuntimeCollider.OldStartPosition = OldStartPosition;
			RuntimeCollider.OldEndPosition = OldEndPosition;
		}
	}
}

void FAnimNode_LoogPhysics::SimulationStepUpdateParticles()
{
	for (auto& ClothTeam : ClothTeams)
	{
		bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr;
		if (!bParameterValid)
		{
			continue;
		}
		const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();
		const auto& TeamData = ClothTeam.TeamData;
		
		for (int32 ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
		{
			auto& Particle = Particles[ParticleIndex];

			FVector NextPosition = Particle.OldPosition;
			FVector VelocityPosition = Particle.OldPosition;
			// 先更新一下基准位置

			FVector BasePosition = FMath::Lerp(Particle.PrevBasePosition, Particle.VirtualMeshPosition, FrameInterpolation);
			FQuat BaseRotation = FQuat::Slerp(Particle.PrevBaseRotation, Particle.VirtualMeshRotation, FrameInterpolation);

			Particle.StepBasePosition = BasePosition;
			Particle.StepBaseRotation = BaseRotation;

			StepBasicPositionBuffer[ParticleIndex] = BasePosition;
			StepBasicRotationBuffer[ParticleIndex] = BaseRotation;

			if (Particle.ParticleType != ELoogPhysicsParticleType::FixedBone)
			{
				// inertia apply to position
				FVector Velocity = Particle.Velocity;

				FVector InertiaVector = TeamData.InertiaVector;
				FQuat InertiaRotation = TeamData.InertiaRotation;

				float InertiaDepth = PhysicsParameter.DepthInertia * (1.0f - Particle.Depth * Particle.Depth);
				InertiaVector = FMath::Lerp(InertiaVector, TeamData.StepVector, InertiaDepth);
				InertiaRotation = FQuat::Slerp(InertiaRotation, TeamData.StepRotation, InertiaDepth);

				FVector LocalPosition = NextPosition - TeamData.PrevStepPosition;
				LocalPosition = InertiaRotation.RotateVector(LocalPosition);
				LocalPosition = LocalPosition + InertiaVector;
				NextPosition = TeamData.PrevStepPosition + LocalPosition;
				VelocityPosition = NextPosition;

				// inertia apply to velocity
				Velocity = InertiaRotation.RotateVector(Velocity);
				Velocity = Velocity * TeamData.VelocityWeight;

				// velocity damping
				float LinearDamping = PhysicsParameter.EvaluateLinearDamping(Particle.Depth) * 0.2f;
				Velocity = Velocity * FMath::Clamp(1.0f - LinearDamping * SimulationPower.Z, 0.f, 1.0f);

				// force
				FVector TotalExternalForce = FVector::Zero();
				TotalExternalForce += PhysicsParameter.GravityDirection * (PhysicsParameter.GravityMagnitude * TeamData.GravityRatio);

				// TODO Wind!!!
				FVector WindForce = FVector::Zero();
				TotalExternalForce += WindForce;

				Velocity = Velocity + TotalExternalForce * SimulationDeltaTime;

				NextPosition = NextPosition + Velocity * SimulationDeltaTime;
			}
			else
			{
				NextPosition = BasePosition;
				VelocityPosition = BasePosition;
			}
			Particle.NextPosition = NextPosition;
			Particle.VelocityPosition = VelocityPosition;
		}
	}
}

void FAnimNode_LoogPhysics::SimulationStepUpdateBaseLinePose()
{


}

void FAnimNode_LoogPhysics::TetherConstraintSolverConstraint()
{
	for (auto& ClothTeam : ClothTeams)
	{
		bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr;
		if (!bParameterValid)
		{
			continue;
		}
		const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();
		for (int ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
		{
			auto& Particle = Particles[ParticleIndex];
			if (Particle.ParticleType == ELoogPhysicsParticleType::FixedBone)
			{
				continue;
			}
			if (Particle.RootParticleIndex == INDEX_NONE)
			{
				continue;
			}
			auto& RootParticle = Particles[Particle.RootParticleIndex];

			FVector CurrentVector = RootParticle.NextPosition - Particle.NextPosition;
			float CurrentLength = CurrentVector.Length();
			if (CurrentLength < PhysicsDefine::Epsilon)
			{
				continue;
			}
			float BaseRestLength = (RootParticle.StepBasePosition - Particle.StepBasePosition).Length();


			float Ratio = CurrentLength / BaseRestLength;
			float CompressionLimit = 1.0f - PhysicsParameter.TetherConstraintCompressionLimit;
			float StretchLimit = 1.0f + PhysicsDefine::TetherStretchLimit;
			float Dist;
			float Stiffness;
			float VelocityAttenuation;
			if (Ratio < CompressionLimit)
			{
				Dist = CurrentLength - CompressionLimit * BaseRestLength;
				Stiffness = PhysicsDefine::TetherCompressionStiffness * FMath::Clamp((CompressionLimit - Ratio) / PhysicsDefine::TetherStiffnessWidth, 0.f, 1.f);
				VelocityAttenuation = PhysicsDefine::TetherCompressionVelocityAttenuation;
			}
			else if (Ratio > StretchLimit)
			{
				Dist = CurrentLength - StretchLimit * BaseRestLength;
				Stiffness = PhysicsDefine::TetherStretchStiffness * FMath::Clamp((Ratio - StretchLimit) / PhysicsDefine::TetherStiffnessWidth, 0.f, 1.f);
				VelocityAttenuation = PhysicsDefine::TetherStretchVelocityAttenuation;
			}
			else
			{
				continue;
			}
			FVector Corr = (CurrentVector / CurrentLength) * Stiffness * Dist;
			Particle.NextPosition += Corr;
			Particle.VelocityPosition += Corr * VelocityAttenuation;
		}
	}
}

void FAnimNode_LoogPhysics::DistanceConstraintSolverConstraint()
{
	for (auto& ClothTeam : ClothTeams)
	{
		bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr;
		if (!bParameterValid)
		{
			continue;
		}
		const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();
		float AnimeRatio = PhysicsParameter.AnimationPoseRatio;

		for (int ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
		{
			auto& Particle = Particles[ParticleIndex];
			if (Particle.ParticleType == ELoogPhysicsParticleType::FixedBone)
			{
				continue;
			}
			if (Particle.RootParticleIndex == INDEX_NONE)
			{
				continue;
			}
			if (Particle.LinkedParticles.Num() == 0)
			{
				continue;
			}
			FVector NextPosition = Particle.NextPosition;
			float InvMass = LoogPhysicsMath::CalcInverseMass(Particle.Friction, Particle.Depth, Particle.ParticleType == ELoogPhysicsParticleType::FixedBone, LoogPhysicsMath::FixedMass);
			float Stiffness = PhysicsParameter.EvaluateStructureStiffness(Particle.Depth);
			Stiffness = Stiffness * SimulationPower.Y;
			FVector BasePosition = Particle.StepBasePosition;

			FVector AddPosition = FVector::Zero();
			int32 AddCount = 0;
			for (auto& LinkedParticleIndex : Particle.LinkedParticles)
			{
				bool bHorizontalLinked = LinkedParticleIndex < 0;
				auto& LinkedParticle = Particles[bHorizontalLinked ? -LinkedParticleIndex : LinkedParticleIndex];


				FVector LinkedNextPosition = LinkedParticle.NextPosition;
				FVector LinkedBasePosition = LinkedParticle.StepBasePosition;

				FVector CurrentVector = LinkedNextPosition - NextPosition;

				FVector BaseVector = BasePosition - LinkedBasePosition;
				float RestDistance = BaseVector.Length();
				if (RestDistance < PhysicsDefine::Epsilon)
				{
					AddPosition += CurrentVector * 0.5f;
					AddCount++;
				}
				else
				{
					float RestLength = FMath::Lerp(RestDistance, RestDistance, AnimeRatio); // TODO animate ratio
					float CurrentDistance = CurrentVector.Length();
					if (CurrentDistance < PhysicsDefine::Epsilon)
					{
						continue;
					}
					FVector Direction = CurrentVector / CurrentDistance;
					float LinkedInvMass = LoogPhysicsMath::CalcInverseMass(LinkedParticle.Friction, LinkedParticle.Depth, LinkedParticle.ParticleType == ELoogPhysicsParticleType::FixedBone, LoogPhysicsMath::FixedMass);
					float FinalStiffness = bHorizontalLinked ? Stiffness * PhysicsDefine::DistanceHorizontalStiffness : Stiffness;


					FVector corr = (CurrentDistance - RestLength) * FinalStiffness * Direction / (InvMass + LinkedInvMass);
					FVector corr0 = InvMass * corr;

					AddPosition += corr0;
					AddCount++;
				}

			}
			if (AddCount > 0)
			{
				AddPosition = AddPosition / static_cast<float>(AddCount);
				NextPosition += AddPosition;
				Particle.NextPosition = NextPosition;
				Particle.VelocityPosition = Particle.VelocityPosition + AddPosition * PhysicsDefine::DistanceVelocityAttenuation;
			}
		}
	}
}

void FAnimNode_LoogPhysics::AngleConstraintSolverConstraint()
{
	for (auto& ClothTeam : ClothTeams)
	{
		bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr;
		if (!bParameterValid)
		{
			continue;
		}
		const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();
		bool bUseAngleLimit = PhysicsParameter.bUseAngleLimit  && PhysicsParameter.AngleLimit > 0.f;
		bool bUseAngleRestoration = PhysicsParameter.bUseAngleRestoration;
		if (!bUseAngleLimit && !bUseAngleRestoration)
		{
			continue;
		}
		float GravityFalloff = FMath::Lerp(1.0f - PhysicsParameter.RestorationGravityFalloff, 1.0f, ClothTeam.TeamData.GravityDot);

		float LimitStiffness = PhysicsParameter.AngleLimitStiffness;
		float RestorationAttn = PhysicsParameter.RestorationVelocityAttenuation;

		// 我们已经按照从父到子的顺序排列，并且两个baseline间本就无依赖(BoneCloth特性), 因而可以直接遍历
		for (int ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
		{
			auto& Particle = Particles[ParticleIndex];

			const FVector& NextPos = Particle.NextPosition;
			const FVector& BasicPos = StepBasicPositionBuffer[ParticleIndex];
			const FQuat& BasicRot = StepBasicRotationBuffer[ParticleIndex];

			RotationBufferArray[ParticleIndex] = BasicRot;

			if (Particle.ParentParticleIndex == INDEX_NONE)
			{
				continue;
			}
			if (Particle.ParticleType == ELoogPhysicsParticleType::FixedBone)
			{
				continue;
			}
			auto& ParentParticle = Particles[Particle.ParentParticleIndex];

			const FVector& ParentNextPos = ParentParticle.NextPosition;
			const FVector& ParentBasicPos = StepBasicPositionBuffer[Particle.ParentParticleIndex];
			const FQuat& ParentBasicRot = StepBasicRotationBuffer[Particle.ParentParticleIndex];

			if (bUseAngleLimit)
			{
				FVector Vector = NextPos - ParentNextPos;
				float VectorLength = Vector.Length();

				FVector BasicVector = BasicPos - ParentBasicPos;
				float BasicVectorLength = BasicVector.Length();
				if (VectorLength < PhysicsDefine::Epsilon || BasicVectorLength < PhysicsDefine::Epsilon)
				{
					LengthBufferArray[ParticleIndex] = 0.f;
					LocalPosBufferArray[ParticleIndex] = FVector::Zero();
					LocalRotBufferArray[ParticleIndex] = FQuat::Identity;
				}
				else
				{
					FVector BVector = BasicVector / BasicVectorLength;
					FQuat InvParentRot = ParentBasicRot.Inverse();
					FVector LocalPos = InvParentRot.RotateVector(BVector);
					FQuat LocalRot = InvParentRot * BasicRot;

					LengthBufferArray[ParticleIndex] = VectorLength;
					LocalPosBufferArray[ParticleIndex] = LocalPos;
					LocalRotBufferArray[ParticleIndex] = LocalRot;
				}

			}

			if (bUseAngleRestoration)
			{
				FVector RestorationVector = BasicPos - ParentBasicPos;
				RestorationVectorBufferArray[ParticleIndex] = RestorationVector;
			}
		}
		for (int32 IterIndex = 0; IterIndex < PhysicsDefine::AngleLimitIteration; ++IterIndex)
		{
			float iterationRatio = static_cast<float>(IterIndex) / (PhysicsDefine::AngleLimitIteration - 1); // 0.0 ~ 1.0

			// ReSharper disable once CppTooWideScope
			constexpr float LimitRotRatio = 0.4f;
			float RestorationRotRatio = FMath::Lerp(0.1f, 0.5f, iterationRatio);

			for (int ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
			{
				auto& Particle = Particles[ParticleIndex];
				if (Particle.ParticleType == ELoogPhysicsParticleType::FixedBone)
				{
					continue;
				}
				FVector ChildPos = Particle.NextPosition;
				float ChildDepth = Particle.Depth;
				float ChildInvMass = LoogPhysicsMath::CalcInverseMass(Particle.Friction);

				auto& ParentParticle = Particles[Particle.ParentParticleIndex];
				FVector ParentPos = ParentParticle.NextPosition;
				float ParentInvMass = LoogPhysicsMath::CalcInverseMass(ParentParticle.Friction);

				if (bUseAngleLimit)
				{
					FQuat ParentRot = RotationBufferArray[Particle.ParentParticleIndex];
					FVector LocalPos = LocalPosBufferArray[ParticleIndex];
					FQuat LocalRot = LocalRotBufferArray[ParticleIndex];

					FVector CurrentVector = ChildPos - ParentPos;
					float CurrentVectorLen = CurrentVector.Length();
					if (CurrentVectorLen < PhysicsDefine::Epsilon)
					{
						goto EndAngleLimit;
					}

					FVector TargetVector = ParentRot * LocalPos;
					float TargetVectorLen = TargetVector.Length();
					if (TargetVectorLen < PhysicsDefine::Epsilon)
					{
						FVector add = ParentPos - ChildPos;
						Particle.NextPosition = ParentPos;
						Particle.VelocityPosition = Particle.VelocityPosition + add;
						RotationBufferArray[ParticleIndex] = ParentRot * LocalRot;
						goto EndAngleLimit;
					}
					CurrentVector /= CurrentVectorLen;
					TargetVector /= TargetVectorLen;

					float BasicLen = LengthBufferArray[ParticleIndex];
					CurrentVectorLen = FMath::Lerp(CurrentVectorLen, BasicLen, 0.5f);
					if (BasicLen < PhysicsDefine::Epsilon || CurrentVectorLen < PhysicsDefine::Epsilon)
					{
						goto EndAngleLimit;
					}
					CurrentVector = CurrentVector * CurrentVectorLen;

					float MaxAngleDeg = PhysicsParameter.EvaluateAngleLimit(ChildDepth);
					float MaxAngleRad = FMath::DegreesToRadians(MaxAngleDeg);
					float AngleRad = LoogPhysicsMath::AngleBetweenVector(CurrentVector, TargetVector);
					FVector RestorationVector = CurrentVector;
					if (AngleRad > MaxAngleRad)
					{
						float RecoveryAngle = FMath::Lerp(AngleRad, MaxAngleRad, LimitStiffness);
						LoogPhysicsMath::ClampAngle(CurrentVector, TargetVector, RecoveryAngle, RestorationVector);
					}

					// 计算旋转中心
					FVector RotPos = ParentPos + CurrentVector * LimitRotRatio;

					FVector ParentFeaturePosition = RotPos - RestorationVector * LimitRotRatio;
					FVector ChildFeaturePosition = RotPos + RestorationVector * (1.0f - LimitRotRatio);

					FVector ParentAdd = ParentFeaturePosition - ParentPos;
					FVector ChildAdd = ChildFeaturePosition - ChildPos;

					ChildAdd *= ChildInvMass;
					ParentAdd *= ParentInvMass;

					constexpr float Attn = PhysicsDefine::AngleLimitAttenuation;

					if (Particle.ParticleType != ELoogPhysicsParticleType::FixedBone)
					{
						ChildPos += ChildAdd;
						Particle.NextPosition = ChildPos;
						Particle.VelocityPosition = Particle.VelocityPosition + ChildAdd * Attn;
					}

					if (ParentParticle.ParticleType != ELoogPhysicsParticleType::FixedBone)
					{
						ParentPos += ParentAdd;
						ParentParticle.NextPosition = ParentPos;
						ParentParticle.VelocityPosition = ParentParticle.VelocityPosition + ParentAdd * Attn;
					}

					// 旋转修正
					CurrentVector = ChildPos - ParentPos;
					CurrentVectorLen = CurrentVector.Length();
					if (CurrentVectorLen < PhysicsDefine::Epsilon)
					{
						goto EndAngleLimit;
					}
					CurrentVector /= CurrentVectorLen;
					FQuat NextRot = ParentRot * LocalRot;
					FQuat Q = LoogPhysicsMath::FromToRotationWithoutNormalize(TargetVector, CurrentVector);
					NextRot = Q * NextRot;
					RotationBufferArray[ParticleIndex] = NextRot;
				}

				EndAngleLimit:

				if (bUseAngleRestoration)
				{
					FVector TargetVector = RestorationVectorBufferArray[ParticleIndex];
					float TargetVectorLen = TargetVector.Length();
					if (TargetVectorLen < PhysicsDefine::Epsilon)
					{
						FVector add = ParentPos - ChildPos;
						Particle.NextPosition = ParentPos;
						Particle.VelocityPosition = Particle.VelocityPosition + add;
						continue;
					}

					FVector CurrentVector = ChildPos - ParentPos;
					float CurrentVectorLen = CurrentVector.Length();
					if (CurrentVectorLen < PhysicsDefine::Epsilon)
					{
						continue;
					}

					float RestorationStiffness = PhysicsParameter.EvaluateAngleRestorationStiffness(ChildDepth) * 0.2f;
					RestorationStiffness = FMath::Clamp(RestorationStiffness * SimulationPower.W, 0.f, 1.f);


					RestorationStiffness *= GravityFalloff;

					FQuat Q = LoogPhysicsMath::FromToRotationWithoutNormalize(CurrentVector / CurrentVectorLen, TargetVector / TargetVectorLen, RestorationStiffness);
					FVector RestorationVector = Q * CurrentVector;

					// 旋转中心
					FVector RotCenterPos = ParentPos + CurrentVector * RestorationRotRatio;

					FVector ParentFeaturePos = RotCenterPos - RestorationVector * RestorationRotRatio;
					FVector ChildFeaturePos = RotCenterPos + RestorationVector * (1.0f - RestorationRotRatio);

					FVector ParentAdd = ParentFeaturePos - ParentPos;
					FVector ChildAdd = ChildFeaturePos - ChildPos;

					ChildAdd *= ChildInvMass;
					ParentAdd *= ParentInvMass;

					if (Particle.ParticleType != ELoogPhysicsParticleType::FixedBone)
					{
						ChildPos += ChildAdd;
						Particle.NextPosition = ChildPos;
						Particle.VelocityPosition = Particle.VelocityPosition + ChildAdd * RestorationAttn;
					}

					if (ParentParticle.ParticleType != ELoogPhysicsParticleType::FixedBone)
					{
						ParentPos += ParentAdd;
						ParentParticle.NextPosition = ParentPos;
						ParentParticle.VelocityPosition = ParentParticle.VelocityPosition + ParentAdd * RestorationAttn;
					}
				}
			}
		}

		//重置buffer
		for (int ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
		{
			LengthBufferArray[ParticleIndex] = 0;
			LocalPosBufferArray[ParticleIndex] = FVector::Zero();
			RestorationVectorBufferArray[ParticleIndex] = FVector::Zero();
		}
	}
}

void FAnimNode_LoogPhysics::SimulationClearTempBuffer()
{
	//重置buffer
	tempVectorBufferA.Empty(Particles.Num());
	tempVectorBufferA.SetNumZeroed(Particles.Num());
	tempRotationBufferA.Empty(Particles.Num());
	tempRotationBufferA.SetNumZeroed(Particles.Num());
	tempVectorBufferB.Empty(Particles.Num());
	tempVectorBufferB.SetNumZeroed(Particles.Num());
	tempCountBuffer.Empty(Particles.Num());
	tempCountBuffer.SetNumZeroed(Particles.Num());
	tempFloatBufferA.Empty(Particles.Num());
	tempFloatBufferA.SetNumZeroed(Particles.Num());
}

void FAnimNode_LoogPhysics::TriangleBendingConstraintSolverConstraint()
{
	for (auto& ClothTeam : ClothTeams)
	{
		bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr;
		if (!bParameterValid)
		{
			continue;
		}
		const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();
		if (PhysicsParameter.TriangleBendingStiffness <= 0.f)
		{
			continue;
		}
		if (ClothTeam.TrianglePairs.Num() == 0)
		{
			continue;
		}
		float Stiffness = FMath::Clamp(PhysicsParameter.TriangleBendingStiffness * SimulationPower.Y, 0.f, 1.f);
		for (const FTrianglePair& TrianglePair : ClothTeam.TrianglePairs)
		{
			auto& Particle0 = Particles[TrianglePair.V0];
			auto& Particle1 = Particles[TrianglePair.V1];
			auto& Particle2 = Particles[TrianglePair.V2];
			auto& Particle3 = Particles[TrianglePair.V3];
			int32 ParticleIndex[4] = { TrianglePair.V0 ,TrianglePair.V1 ,TrianglePair.V2 ,TrianglePair.V3 };
			FLoogPhysicsParticle* LocalParticles[4] = { &Particle0, &Particle1, &Particle2, &Particle3 };
			FVector NextPosBuffer[4];
			FVector AddPosBuffer[4];
			FVector4f InvMassBuffer;
			for (int32 i = 0; i < 4; ++i)
			{
				NextPosBuffer[i] = LocalParticles[i]->NextPosition;
				AddPosBuffer[i] = FVector::Zero();
				InvMassBuffer[i] = LoogPhysicsMath::CalcInverseMass(LocalParticles[i]->Friction, LocalParticles[i]->Depth);
			}
			float RestAngle = TrianglePair.RestAngleOrVolume;
			const ELoogPhysicsTriangleSignFlag& SignOrVolume = TrianglePair.SignFlag;
			bool bResult;
			if (SignOrVolume == ELoogPhysicsTriangleSignFlag::Volume)
			{
				// Volume
				const float& volumeRest = RestAngle;
				bResult = LoogPhysicsMath::CalcVolume(NextPosBuffer, InvMassBuffer, volumeRest, Stiffness, AddPosBuffer);
			}
			else
			{
				// Triangle Bending
				// 方向二面角
				float sign = SignOrVolume == ELoogPhysicsTriangleSignFlag::Negative ? -1.f : 1.f;
				RestAngle *= sign;

				bResult = LoogPhysicsMath::CalcDihedralAngle(sign, NextPosBuffer, InvMassBuffer, RestAngle, Stiffness, AddPosBuffer);
			}

			// 存储一下
			if (bResult)
			{
				for (int i = 0; i < 4; i++)
				{
					tempVectorBufferA[ParticleIndex[i]] = tempVectorBufferA[ParticleIndex[i]] + AddPosBuffer[i];
					tempCountBuffer[ParticleIndex[i]]++;
				}
			}
		}
	}
}

void FAnimNode_LoogPhysics::TriangleBendingConstraintSumConstraint()
{
	for (auto& ClothTeam : ClothTeams)
	{
		bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr;
		if (!bParameterValid)
		{
			continue;
		}
		const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();
		if (PhysicsParameter.TriangleBendingStiffness <= 0.f)
		{
			continue;
		}
		if (ClothTeam.TrianglePairs.Num() == 0)
		{
			continue;
		}
		for (int32 ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
		{
			auto& Particle = Particles[ParticleIndex];
			if (Particle.ParticleType != ELoogPhysicsParticleType::FixedBone)
			{
				int32 Count = tempCountBuffer[ParticleIndex];
				if (Count > 0)
				{
					FVector Add = tempVectorBufferA[ParticleIndex] / static_cast<float>(Count);
					Particle.NextPosition = Particle.NextPosition + Add;
				}
			}
			tempVectorBufferA[ParticleIndex] = FVector::Zero();
			tempCountBuffer[ParticleIndex] = 0;
		}
	}
}

void FAnimNode_LoogPhysics::SolveColliderCollisionConstraint()
{
	for (auto& ClothTeam : ClothTeams)
	{
		bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr;
		if (!bParameterValid)
		{
			continue;
		}
		const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();
		if (PhysicsParameter.CollisionMethod == ELoogPhysicsColliderCollisionMethod::Point)
		{
			ColliderCollisionConstraintSolverPointConstraint(ClothTeam);
		}
		else if (PhysicsParameter.CollisionMethod == ELoogPhysicsColliderCollisionMethod::Edge)
		{
			ColliderCollisionConstraintSolverEdgeConstraint(ClothTeam);
			ColliderCollisionConstraintSumEdgeConstraint(ClothTeam);
		}
	}
}

void FAnimNode_LoogPhysics::ColliderCollisionConstraintSolverPointConstraint(FLoogPhysicsClothTeam& InClothTeam)
{
	if (InClothTeam.CapsuleColliders.Num() == 0)
	{
		return;
	}
	bool bParameterValid = InClothTeam.ParameterAsset.Get() != nullptr;
	if (!bParameterValid)
	{
		return;
	}
	const auto& PhysicsParameter = *InClothTeam.ParameterAsset.Get();
	for (int32 ParticleIndex = InClothTeam.StartIndex; ParticleIndex <= InClothTeam.EndIndex; ++ParticleIndex)
	{
		auto& Particle = Particles[ParticleIndex];
		if (Particle.ParticleType == ELoogPhysicsParticleType::FixedBone)
		{
			continue;
		}
		FVector NextPos = Particle.NextPosition;
		float Depth = Particle.Depth;

		// 质点半径
		float Radius = FMath::Max(PhysicsParameter.EvaluateRadius(Depth), 0.0001f); // safe;

		float MinDist = FLT_MAX;

		// 碰撞数据
		int32 CollisionColliderId = INDEX_NONE;
		FVector CollisionNormal = FVector::Zero();
		FVector N = FVector::Zero();

		// 质点挤出数据
		FVector AddPos = FVector::Zero();
		int32 AddCnt = 0;
		FVector AddN = FVector::Zero();

		// 碰撞检测时与碰撞体的最大距离（碰撞摩擦范围/collisionFrictionRange）
		// 根据粒子大小计算
		float CollisionFrictionRange = Radius * 1.0f; // 1.0f?

		// 粒子 AABB
		UE::Geometry::FAxisAlignedBox3d ParticleAABB(NextPos - Radius, NextPos + Radius);
		ParticleAABB.Expand(CollisionFrictionRange);

		for (int32 ColliderIndex = 0; ColliderIndex < InClothTeam.CapsuleColliders.Num(); ++ColliderIndex)
		{
			const auto& Collider = InClothTeam.CapsuleColliders[ColliderIndex];
			FVector TempNextPos = NextPos;
			float Dist = Collider.PointCapsuleColliderDetection(TempNextPos, Radius, ParticleAABB, N);

			// 明确发生了挤推
			if (Dist <= 0.0f)
			{
				// 将所有挤出矢量和接触法线相加
				AddPos += (TempNextPos - NextPos);
				AddN += N;
				AddCnt++;
			}

			// 当以一定距离接近碰撞体时（动摩擦/静摩擦会影响碰撞）
			if (Dist <= CollisionFrictionRange)
			{
				// 将所有接触法线相加，并记录到碰撞体的最近距离。
				CollisionColliderId = ColliderIndex;
				CollisionNormal += N;
				MinDist = FMath::Min(MinDist, Dist);
			}
		}
		if (AddCnt > 0)
		{
			// 限制粒子移动距离，使其与组合接触法线的长度成正比。
			// 这可以防止夹在两个碰撞器之间的粒子剧烈运动。
			AddN /= AddCnt;
			float len = AddN.Length();
			if (len < PhysicsDefine::Epsilon)
			{
				AddPos = FVector::Zero();
			}
			else
			{
				float t = FMath::Min(len, 1.0f);
				AddPos /= AddCnt;
				NextPos += AddPos * t;
			}
		}

		// 摩擦系数(friction)计算
		if (CollisionColliderId >= 0 && CollisionFrictionRange > 0.0f && CollisionNormal.Length() > PhysicsDefine::Epsilon)
		{
			// 根据与碰撞器的距离而变化（0.0 ~ 接触面 1.0）
			float Friction = 1.0f - FMath::Clamp(MinDist / CollisionFrictionRange, 0.f, 1.f);
			Particle.Friction = FMath::Max(Friction, Particle.Friction); // 大きい方

			// 摩擦用接触法线平均化
			CollisionNormal = CollisionNormal.GetSafeNormal();
		}
		Particle.CollisionNormal = CollisionNormal;
		Particle.NextPosition = NextPos;
	}
}

void FAnimNode_LoogPhysics::ColliderCollisionConstraintSolverEdgeConstraint(FLoogPhysicsClothTeam& InClothTeam)
{
	if (InClothTeam.CapsuleColliders.Num() == 0)
	{
		return;
	}
	bool bParameterValid = InClothTeam.ParameterAsset.Get() != nullptr;
	if (!bParameterValid)
	{
		return;
	}
	const auto& PhysicsParameter = *InClothTeam.ParameterAsset.Get();
	for (const FInt32Vector2& Edge : InClothTeam.Edges)
	{
		auto& ParticleX = Particles[Edge.X];
		auto& ParticleY = Particles[Edge.Y];
		
		bool isMove0 = ParticleX.ParticleType != ELoogPhysicsParticleType::FixedBone;
		bool isMove1 = ParticleY.ParticleType != ELoogPhysicsParticleType::FixedBone;
		if (!isMove0 && !isMove1)
			continue;
		FVector NextPosEdgeC0 = ParticleX.NextPosition;
		FVector NextPosEdgeC1 = ParticleY.NextPosition;
		FVector2f EdgeDepth = FVector2f(ParticleX.Depth, ParticleY.Depth);
		FVector2f EdgeRadius = FVector2f(PhysicsParameter.EvaluateRadius(EdgeDepth.X), PhysicsParameter.EvaluateRadius(EdgeDepth.Y));

		// 碰撞检测时与碰撞器的最大距离
		// 根据粒子大小计算
		float CollisionFrictionRange = (EdgeRadius.X + EdgeRadius.Y) * 0.5f * 1.0f; // 1.0f?

		float MinDist = FLT_MAX;
		int CollisionColliderId = INDEX_NONE;
		FVector CollisionNormal = FVector::Zero();
		FVector N = FVector::Zero();

		// Edge的AABB
		UE::Geometry::FAxisAlignedBox3d EdgeAABB = UE::Geometry::FAxisAlignedBox3d(NextPosEdgeC0 - EdgeRadius.X, NextPosEdgeC0 + EdgeRadius.Y);
		UE::Geometry::FAxisAlignedBox3d AABB1 = UE::Geometry::FAxisAlignedBox3d(NextPosEdgeC1 - EdgeRadius.Y, NextPosEdgeC1 + EdgeRadius.Y);
		EdgeAABB.Contain(AABB1);
		EdgeAABB.Expand(CollisionFrictionRange);

		FVector AddPosC0 = FVector::Zero();
		FVector AddPosC1 = FVector::Zero();
		int AddCnt = 0;
		FVector AddN = FVector::Zero();

		// チーム内のコライダーをループ
		for (int32 ColliderIndex = 0; ColliderIndex < InClothTeam.CapsuleColliders.Num(); ++ColliderIndex)
		{
			const auto& Collider = InClothTeam.CapsuleColliders[ColliderIndex];
			FVector TempNextPosC0 = NextPosEdgeC0;
			FVector TempNextPosC1 = NextPosEdgeC1;
			
			float Dist = Collider.EdgeCapsuleColliderDetection(TempNextPosC0, TempNextPosC1, EdgeRadius, EdgeAABB, CollisionFrictionRange, N);

			// 需要挤压的
			if (Dist <= 0.0f)
			{
				// 将所有挤出向量和接触法线相加
				AddPosC0 += (TempNextPosC0 - NextPosEdgeC0);
				AddPosC1 += (TempNextPosC1 - NextPosEdgeC1);
				AddN += N;
				AddCnt++;
			}

			// 当以一定距离接近碰撞体时（动态摩擦/静摩擦的影响）
			if (Dist <= CollisionFrictionRange)
			{
				// 将所有接触法线相加，并记录到碰撞器的最近距离
				CollisionColliderId = ColliderIndex;
				CollisionNormal += N;
				MinDist = FMath::Min(MinDist, Dist);
			}
		}

		// 平均一下
		if (AddCnt > 0)
		{
			// 限制粒子移动距离，使其与组合接触法线的长度成正比。
			// 这可以防止夹在两个碰撞器之间的粒子剧烈运动。
			float InvAddCount = 1.0f / static_cast<float>(AddCnt);
			AddN *= InvAddCount;
			float Len = AddN.Length();
			if (Len > PhysicsDefine::Epsilon)
			{
				float T = FMath::Min(Len, 1.0f);
				AddPosC0 = AddPosC0 * (InvAddCount * T);
				AddPosC1 = AddPosC1 * (InvAddCount * T);

				// 書き戻し
				tempVectorBufferA[Edge.X] += AddPosC0;
				tempCountBuffer[Edge.X]++;

				tempVectorBufferA[Edge.Y] += AddPosC1;
				tempCountBuffer[Edge.Y]++;
			}
		}

		// //摩擦系数(friction)计算
		if (CollisionColliderId >= 0 && CollisionFrictionRange > 0.0f && CollisionNormal.Length() > PhysicsDefine::Epsilon)
		{
			// 根据与碰撞器的距离而变化（0.0 ~ 接触面1.0）
			float Friction = 1.0f - FMath::Clamp(MinDist / CollisionFrictionRange, 0.f, 1.f);
			CollisionNormal.Normalize();
			if (isMove0)
			{
				tempFloatBufferA[Edge.X] = FMath::Max(tempFloatBufferA[Edge.X], Friction);
				tempVectorBufferB[Edge.X] += CollisionNormal;
			}
			if (isMove1)
			{
				tempFloatBufferA[Edge.Y] = FMath::Max(tempFloatBufferA[Edge.Y], Friction);
				tempVectorBufferB[Edge.Y] += CollisionNormal;
			}
		}
	}
}

void FAnimNode_LoogPhysics::ColliderCollisionConstraintSumEdgeConstraint(const FLoogPhysicsClothTeam& InClothTeam)
{
	if (InClothTeam.CapsuleColliders.Num() == 0)
	{
		return;
	}
	for (int32 ParticleIndex = InClothTeam.StartIndex; ParticleIndex <= InClothTeam.EndIndex; ++ParticleIndex)
	{
		// next pos
		auto& Particle = Particles[ParticleIndex];
		int32 Count = tempCountBuffer[ParticleIndex];
		if (Count > 0)
		{
			float InvCount = 1.0f / static_cast<float>(Count);
			FVector Add = tempVectorBufferA[ParticleIndex] * InvCount;
			Particle.NextPosition = Particle.NextPosition + Add;
		}

		// friction
		float F = tempFloatBufferA[ParticleIndex];
		if (F > 0.0f && F > Particle.Friction)
		{
			Particle.Friction = F;
		}

		// collision normal
		FVector N = tempVectorBufferB[ParticleIndex];
		if (N.Length() > PhysicsDefine::Epsilon)
		{
			N.Normalize();
			Particle.CollisionNormal = N;
		}

		// 清除缓冲区
		tempVectorBufferA[ParticleIndex] = FVector::Zero();
		tempVectorBufferB[ParticleIndex] = FVector::Zero();
		tempCountBuffer[ParticleIndex] = 0;
		tempFloatBufferA[ParticleIndex] = 0.f;
	}
}

void FAnimNode_LoogPhysics::MotionConstraintSolverConstraint()
{

}

void FAnimNode_LoogPhysics::SimulationStepPostTeam()
{
	for (auto& ClothTeam : ClothTeams)
	{
		bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr;
		if (!bParameterValid)
		{
			continue;
		}
		const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();
		for (int ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
		{
			auto& Particle = Particles[ParticleIndex];
			float Depth = Particle.Depth;
			FVector NextPos = Particle.NextPosition;
			FVector OldPos = Particle.OldPosition;

			if (Particle.ParticleType != ELoogPhysicsParticleType::FixedBone)
			{
				FVector VelocityOldPos = Particle.VelocityPosition;

				// 动摩擦系数
				float Friction = Particle.Friction;
				FVector CollisionNormal = Particle.CollisionNormal;
				bool bIsCollision = CollisionNormal.Length() > PhysicsDefine::Epsilon && Friction > PhysicsDefine::Epsilon;
				float StaticFrictionParam = PhysicsParameter.Friction * PhysicsDefine::ColliderCollisionStaticFrictionRatio;
				float DynamicFrictionParam = PhysicsParameter.Friction * PhysicsDefine::ColliderCollisionDynamicFrictionRatio;

				// 静摩擦
				float StaticFriction = Particle.StaticFriction;
				if (bIsCollision && StaticFrictionParam > 0.0f)
				{
					// 根据切向运动速度计算
					FVector V = NextPos - OldPos;
					FVector TanV = V - LoogPhysicsMath::Project(V, CollisionNormal); // 切向平移向量
					float TangentVelocity = TanV.Length() / SimulationDeltaTime * 0.01f; // 切向运动速度 (m/s)

					// 如果速度低于静止速度，则增大系数
					if (TangentVelocity < StaticFrictionParam)
					{
						StaticFriction = FMath::Clamp(StaticFriction + 0.04f, 0.f, 1.f); // 系数增加（0.02？）
					}
					else
					{
						// 根据切向速度减小系数
						float Vel = TangentVelocity - StaticFrictionParam;
						float Value = FMath::Max(Vel / 0.2f, 0.05f);
						StaticFriction = FMath::Clamp(StaticFriction - Value, 0.f, 1.f);
					}

					// 沿接触面的切线方向将位置回退（基于静摩擦，将切向位移按静摩擦系数回滚）
					TanV *= (StaticFriction * 100.f); // 转换为cm/s
					NextPos -= TanV;
					VelocityOldPos -= TanV;
				}
				else
				{
					// 摩擦系数衰减
					StaticFriction = FMath::Clamp(StaticFriction - 0.05f, 0.f, 1.f);
				}
				Particle.StaticFriction = StaticFriction;

				// 根据位置计算速度（用于根据约束条件调整速度）
				FVector Velocity = (NextPos - VelocityOldPos) / SimulationDeltaTime;
				float VelocityLen = Velocity.Length();
				FVector NormalVelocity = VelocityLen > PhysicsDefine::Epsilon ?  Velocity / VelocityLen : FVector::Zero();

				// 动摩擦
				// 与冲击面的角度越大，阻尼越强（MC1）。
				if (bIsCollision && DynamicFrictionParam > 0.0f && VelocityLen >= PhysicsDefine::Epsilon)
				{
					float Dot = FVector::DotProduct(CollisionNormal, NormalVelocity);
					Dot = 0.5f + 0.5f * Dot; // 1.0(front) - 0.5(side) - 0.0(back)
					Dot *= Dot; // 加强两侧
					Dot = 1.0f - Dot; // 0.0(front) - 0.75(side) - 1.0(back)
					Velocity -= Velocity * (Dot * FMath::Clamp(Friction * DynamicFrictionParam, 0.f, 1.f));
				}
				// 摩擦系数衰减
				Friction *= PhysicsDefine::FrictionDampingRate;
				Particle.Friction = Friction;

				// 最大速度
				// 限制最大速度可以改善运动效果，因此应该添加此功能。
				// 这将使头发和其他物体的运动更加柔和，尤其是在旋转时。
				// 但是，请注意不要过度限制速度，因为这会降低碰撞器的推动精度。
				if (PhysicsParameter.ParticleSpeedLimit >= 0.0f)
				{
					// velocity = LoogPhysicsMath::ClampVector(velocity, PhysicsParameter.ParticleSpeedLimit);
					if (Velocity.Length() > PhysicsParameter.ParticleSpeedLimit)
					{
						Velocity = Velocity / Velocity.Length() * PhysicsParameter.ParticleSpeedLimit;
					}
				}
				// 离心加速
				if (ClothTeam.TeamData.AngularVelocity > PhysicsDefine::Epsilon && PhysicsParameter.CentrifualAcceleration > PhysicsDefine::Epsilon && VelocityLen >= PhysicsDefine::Epsilon)
				{
					// 旋转中心的局部坐标
					FVector LocalPos = NextPos - ClothTeam.TeamData.NowStepPosition;

					// 投影到旋转轴平面上
					FVector V = LoogPhysicsMath::ProjectOnPlane(LocalPos, ClothTeam.TeamData.RotationAxis);
					float R = V.Length();
					if (R > PhysicsDefine::Epsilon)
					{
						FVector N = V / R;

						// 角速度(rad/s)
						float W = ClothTeam.TeamData.AngularVelocity;

						// 重量（重量越重，离心力越大）
						// 这里，两端的重量要轻一些。
						//float m = (1.0f - depth) * 3.0f;
						//float m = 1.0f + (1.0f - depth) * 2.0f;
						float M = 1.0f + (1.0f - Depth); // fix
						//float m = 1.0f + depth * 3.0f;
						//const float m = 1;

						// 离心力
						float F = M * W * W * R;

						// 只有当旋转方向 u 和速度方向相同时（点积相乘），才会施加力。
						// 在实际物理中，离心力仅在绳子绷紧时才会产生，但确定绳子绷紧的状态并不容易。
						// 因此，这里使用这种近似方法。
						// 如果旋转方向和速度方向相反，则认为绳子处于松弛状态，不会产生离心力增强。
						FVector U = FVector::CrossProduct(ClothTeam.TeamData.RotationAxis, N).GetSafeNormal();
						F *= FMath::Clamp(FVector::DotProduct(NormalVelocity, U), 0.f, 1.f);

						// 将离心力添加到速度中
						Velocity += N * (F * PhysicsParameter.CentrifualAcceleration * 2.f);
					}
				}

				// 稳定速度
				Velocity *= ClothTeam.TeamData.VelocityWeight;
				Particle.Velocity = Velocity;
			}

			// 实际速度
			FVector realVelocity = (NextPos - OldPos) / SimulationDeltaTime;
			Particle.RealVelocity = realVelocity;

			// 记录预测位置
			Particle.OldPosition = NextPos;
		}
	}
}

void FAnimNode_LoogPhysics::ColliderManagerSimulationEndStep()
{
	for (auto& ClothTeam : ClothTeams)
	{
		for (int32 ColliderIndex = 0; ColliderIndex < ClothTeam.CapsuleColliders.Num(); ++ColliderIndex)
		{
			auto& Collider = ClothTeam.CapsuleColliders[ColliderIndex];
			Collider.OldPosition = Collider.NowPosition;
			Collider.OldRotation = Collider.NowRotation;
		}
	}
	
}

void FAnimNode_LoogPhysics::SimulationCalcDisplayPosition()
{
	for (auto& ClothTeam : ClothTeams)
	{
		bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr;
		if (!bParameterValid)
		{
			continue;
		}
		// const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();
		for (int32 ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
		{
			auto& Particle = Particles[ParticleIndex];

			FVector BasePos = Particle.VirtualMeshPosition;
			FQuat BaseRot = Particle.VirtualMeshRotation;

			if (Particle.ParticleType != ELoogPhysicsParticleType::FixedBone)
			{
				// 运动粒子
				FVector Dpos = Particle.OldPosition;

				// 未来预测：在高帧率时非常重要，可以让模拟结果看起来很平滑，否则会因为是固定模拟帧率导致输出pose与当前帧时间不匹配而产生抖动。
				// 根据上次计算的位置和实际速度预测下一步位置，并使用它们之间的帧时间位置作为显示位置。
				FVector Velocity = Particle.RealVelocity * SimulationDeltaTime;
				FVector FuturePos = Dpos + Velocity;
				float Interval = (NowUpdateTime + SimulationDeltaTime) - PrevTickTime;
				float T = Interval > 0.0f ? (CurrentTickTime - PrevTickTime) / Interval : 0.0f;
				FuturePos = FMath::Lerp(Particle.DisplayPosition, FuturePos, T);
				// 预测位置限制：防止预测位置过远导致穿透等问题
				int RootIndex = Particle.RootParticleIndex;
				if (RootIndex != INDEX_NONE)
				{
					auto& RootParticle = Particles[RootIndex];
					FVector RootPos = RootParticle.VirtualMeshPosition;
					float OriginalDist = FVector::Distance(RootPos, BasePos);
					float ClampDist = OriginalDist * PhysicsDefine::MaxDistanceRatioFuturePrediction; // 允许距离限制
					FVector V = FuturePos - RootPos;
					if (V.Length() > ClampDist)
					{
						V = V / V.Length() * ClampDist;
					}
					FuturePos = RootPos + V;
				}

				Dpos = FuturePos;

				// 表示位置
				FVector DispPos = Dpos;

				// 记录显示位置
				Particle.DisplayPosition = DispPos;

				// 混合一下
				FVector VirtualMeshPos = FMath::Lerp(Particle.VirtualMeshPosition, DispPos, ClothTeam.TeamData.BlendWeight);

				Particle.VirtualMeshPosition = VirtualMeshPos;
			}
			else
			{
				// 固定粒子
				// 显示位置始终保持在原始位置
				Particle.DisplayPosition = Particle.VirtualMeshPosition;
			}

			// 记录VirtualMesh的上一步位置/旋转
			if (SimStepCount > 0)
			{
				Particle.PrevBasePosition = BasePos;
				Particle.PrevBaseRotation = BaseRot;
			}

			// 保存临时位置/旋转供下一步使用
			tempVectorBufferA[ParticleIndex] = BasePos;
			tempRotationBufferA[ParticleIndex] = BaseRot;
		}
	}
}

void FAnimNode_LoogPhysics::VirtualMeshManagerSimulationPostProxyMeshUpdateLine()
{
	for (auto& ClothTeam : ClothTeams)
	{
		bool bParameterValid = ClothTeam.ParameterAsset.Get() != nullptr;
		if (!bParameterValid)
		{
			continue;
		}
		const auto& PhysicsParameter = *ClothTeam.ParameterAsset.Get();
		float AverageRate = PhysicsParameter.RotationalInterpolation; // 旋转平均比率
		float RootInterpolation = PhysicsParameter.RootRotation;
		float AnimeRatio = PhysicsParameter.AnimationPoseRatio;
		float BlendWeight = ClothTeam.TeamData.BlendWeight;


		for (int32 ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
		{
			auto& Particle = Particles[ParticleIndex];
			if (Particle.ParticleType == ELoogPhysicsParticleType::FixedBone)
			{
				continue;
			}
			// 以自身为父节点
			FVector Pos = Particle.VirtualMeshPosition;
			FQuat Rot = Particle.VirtualMeshRotation;

			FVector BasePos = tempVectorBufferA[ParticleIndex];
			FQuat BaseRot = tempRotationBufferA[ParticleIndex];
			FQuat BaseInvRot = BaseRot.Inverse();

			if (Particle.ChildParticleIndices.Num() > 0)
			{
				// 子节点的平均向量和当前向量和
				FVector ChildOriginalVectorSum = FVector::Zero();
				FVector ChildCurrentVectorSum = FVector::Zero();

				// 以自身为基准求子的旋转，同时向子添加平均向量
				for (const auto& ChildParticleIndex : Particle.ChildParticleIndices)
				{
					auto& ChildParticle = Particles[ChildParticleIndex];

					// 这种零距离状态
					bool bIsChildZeroDistance = ChildParticle.LocalRefPosition.Length() <= PhysicsDefine::Epsilon;

					// child当前的位置
					FVector ChildPos = ChildParticle.VirtualMeshPosition;

					// child当前的基础姿势和局部姿势
					FVector ChildBasePos = tempVectorBufferA[ChildParticleIndex];
					FQuat ChildBaseRot = tempRotationBufferA[ChildParticleIndex];
					FVector ChildBaseLocalPos = BaseInvRot * (ChildBasePos - BasePos);
					FQuat ChildBaseLocalRot = BaseInvRot * ChildBaseRot;

					// 以局部位置和局部旋转作为计算基础
					// 使用 AnimationPoseRatio 进行插值
					FVector LocalPos = FMath::Lerp(Particle.LocalRefPosition, ChildBaseLocalPos, AnimeRatio);
					FQuat LocalRot = FQuat::Slerp(Particle.LocalRefRotation, ChildBaseLocalRot, AnimeRatio);

					// child原始向量
					FVector ChildOriginalVector = bIsChildZeroDistance ? FVector::Zero() : Rot * LocalPos;

					ChildOriginalVectorSum += ChildOriginalVector;

					// 子质点当前向量
					if (ChildParticle.ParticleType != ELoogPhysicsParticleType::FixedBone)
					{
						FVector ChildCurrentVector = ChildPos - Pos;
						ChildCurrentVectorSum += ChildCurrentVector;

						// 确定子质点的旋转方向
						// 此时，子质点将沿父质点的方向旋转（1.0）。
						FQuat ChildRot = Rot * LocalRot;

						// 通过当前矢量位移进行校正
						if (bIsChildZeroDistance == false)
						{
							FQuat Q = LoogPhysicsMath::FromToRotationWithoutNormalize(ChildOriginalVector, ChildCurrentVector);
							ChildRot = Q * ChildRot;
						}

						ChildParticle.VirtualMeshRotation = ChildRot;
					}
					else
					{
						ChildCurrentVectorSum += ChildOriginalVector;
					}
				}

				// 旋转调整至child方向
				float T = Particle.ParticleType != ELoogPhysicsParticleType::FixedBone ? AverageRate : RootInterpolation;
				bool bChildOriginalVectorZero = ChildOriginalVectorSum.IsNearlyZero();
				bool bChildCurrentVectorZero = ChildCurrentVectorSum.IsNearlyZero();
				FQuat Cq = (bChildOriginalVectorZero || bChildCurrentVectorZero) ? FQuat::Identity : LoogPhysicsMath::FromToRotationWithoutNormalize(ChildOriginalVectorSum, ChildCurrentVectorSum, T);
				Rot = Cq * Rot;
			}

			// 混合权重应用
			Rot = FQuat::Slerp(BaseRot, Rot, BlendWeight);

			// 确定自身姿势
			Particle.VirtualMeshRotation = Rot;
		}
	}
}

void FAnimNode_LoogPhysics::VirtualMeshManagerSimulationPostProxyMeshUpdateTriangle()
{

}

void FAnimNode_LoogPhysics::VirtualMeshManagerSimulationPostProxyMeshUpdateTriangleSum()
{

}

void FAnimNode_LoogPhysics::VirtualMeshManagerSimulationPostProxyMeshUpdateWorldTransform()
{
	for (auto& ClothTeam : ClothTeams)
	{
		if (ClothTeam.ParameterAsset.Get() == nullptr)
		{
			continue;
		}
		for (int32 ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
		{
			auto& Particle = Particles[ParticleIndex];
			Particle.BonePosition = Particle.VirtualMeshPosition;
			Particle.BoneRotation = Particle.VirtualMeshRotation;
		}
	}
}

void FAnimNode_LoogPhysics::VirtualMeshManagerSimulationPostProxyMeshUpdateLocalTransform()
{
	for (auto& ClothTeam : ClothTeams)
	{
		if (ClothTeam.ParameterAsset.Get() == nullptr)
		{
			continue;
		}
		for (int32 ParticleIndex = ClothTeam.StartIndex; ParticleIndex <= ClothTeam.EndIndex; ++ParticleIndex)
		{
			auto& Particle = Particles[ParticleIndex];
			if (Particle.ParentParticleIndex == INDEX_NONE)
			{
				continue;
			}
			if (Particle.ParticleType == ELoogPhysicsParticleType::FixedBone)
			{
				continue;
			}
			auto& ParentParticle = Particles[Particle.ParentParticleIndex];
			FVector ParentPos = ParentParticle.BonePosition;
			FQuat ParentRot = ParentParticle.BoneRotation;

			FVector ChildPos = Particle.BonePosition;
			FQuat ChildRot = Particle.BoneRotation;

			FQuat InvParentRot = ParentRot.Inverse();
			FVector V = ChildPos - ParentPos;
			FVector LocalPos = InvParentRot.RotateVector(V);
			FQuat LocalRot = InvParentRot * ChildRot;

			Particle.BoneLocalPosition = LocalPos;
			Particle.BoneLocalRotation = LocalRot;
		}
	}
}

void FAnimNode_LoogPhysics::ColliderManagerSimulationPostUpdate()
{
	for (auto& ClothTeam : ClothTeams)
	{
		if (ClothTeam.ParameterAsset.Get() == nullptr)
		{
			continue;
		}
		for (int32 ColliderIndex = 0; ColliderIndex < ClothTeam.CapsuleColliders.Num(); ++ColliderIndex)
		{
			auto& Collider = ClothTeam.CapsuleColliders[ColliderIndex];
			Collider.OldFrameBasePosition = Collider.FrameBasePosition;
			Collider.OldFrameBaseRotation = Collider.FrameBaseRotation;
		}
	}
}

void FAnimNode_LoogPhysics::TeamManagerSimulationPostTeamUpdate()
{
	for (auto& ClothTeam : ClothTeams)
	{
		if (ClothTeam.ParameterAsset.Get() == nullptr)
		{
			continue;
		}
		auto& ClothTeamData = ClothTeam.TeamData;
		ClothTeamData.OldComponentWorldPosition = ClothTeamData.ComponentWorldPosition;
		ClothTeamData.OldComponentWorldRotation = ClothTeamData.ComponentWorldRotation;
		if (SimStepCount > 0)
		{
			ClothTeamData.OldFrameWorldPosition = ClothTeamData.FrameWorldPosition;
			ClothTeamData.OldFrameWorldRotation = ClothTeamData.FrameWorldRotation;
			SkipCount = 0;
		}
	}
	if (bNeedResetSimulation)
	{
		bNeedResetSimulation = false;
	}
	constexpr float LimitTime = 3600.0f; // 60min
	if (CurrentTickTime > LimitTime * 2)
	{
		CurrentTickTime -= LimitTime;
		PrevTickTime -= LimitTime;
		NowUpdateTime -= LimitTime;
		OldUpdateTime -= LimitTime;
		FrameUpdateTime -= LimitTime;
		PrevFrameUpdateTime -= LimitTime;
	}
}

void FAnimNode_LoogPhysics::WriteBackTransforms(FComponentSpacePoseContext& Output, TArray<FBoneTransform>& OutBoneTransforms)
{
	const auto& RequiredBones = Output.AnimInstanceProxy->GetRequiredBones();
	const auto& ComponentTransform = Output.AnimInstanceProxy->GetComponentTransform();

	// calculate local pos/rot first
	for (int32 ParticleIndex = 0; ParticleIndex < Particles.Num(); ++ParticleIndex)
	{
		auto& Particle = Particles[ParticleIndex];
		if (Particle.ParticleType == ELoogPhysicsParticleType::VirtualBone)
		{
			continue;
		}
		if (Particle.ParticleType == ELoogPhysicsParticleType::FixedBone)
		{
			// fixed的点，直接写入component位置/旋转
			auto CompactPoseIndex = Particle.BindBone.GetCompactPoseIndex(RequiredBones);
			FTransform BonePoseTransform = Output.Pose.GetComponentSpaceTransform(CompactPoseIndex);
			FVector BoneComponentPosition = ComponentTransform.InverseTransformPosition(Particle.BonePosition);
			FQuat BoneComponentRotation = ComponentTransform.InverseTransformRotation(Particle.BoneRotation);
			BonePoseTransform.SetTranslation(BoneComponentPosition);
			BonePoseTransform.SetRotation(BoneComponentRotation);
			OutBoneTransforms.Add(FBoneTransform(CompactPoseIndex, BonePoseTransform));
		}
		else
		{
			// 移动的点 写入local位置/旋转
			auto CompactPoseIndex = Particle.BindBone.GetCompactPoseIndex(RequiredBones);
			FTransform BonePoseTransform = Output.Pose.GetComponentSpaceTransform(CompactPoseIndex);
			FVector BoneComponentPosition = ComponentTransform.InverseTransformPosition(Particle.BonePosition);
			FQuat BoneComponentRotation = ComponentTransform.InverseTransformRotation(Particle.BoneRotation);
			BonePoseTransform.SetTranslation(BoneComponentPosition);
			BonePoseTransform.SetRotation(BoneComponentRotation);
			OutBoneTransforms.Add(FBoneTransform(CompactPoseIndex, BonePoseTransform));
		}
	}
	// important!
	OutBoneTransforms.Sort(FCompareBoneTransformIndex());
}

void FAnimNode_LoogPhysics::CalculateCenterTransform(FLoogPhysicsClothTeam& InClothTeam, FVector& OutCenterPos, FQuat& OutCenterRot)
{
	FVector FixedPosition = FVector::Zero();
	FVector SumNormal = FVector::Zero();
	FVector SumTangent = FVector::Zero();
	int32 AddCount = 0;

	for (int32 ParticleIndex = InClothTeam.StartIndex; ParticleIndex <= InClothTeam.EndIndex; ++ParticleIndex)
	{
		const auto& Particle = Particles[ParticleIndex];
		if (Particle.ParticleType == ELoogPhysicsParticleType::FixedBone)
		{
			FixedPosition += Particle.VirtualMeshPosition;
			SumNormal += Particle.VirtualMeshRotation.RotateVector(FVector::XAxisVector);
			SumTangent += Particle.VirtualMeshRotation.RotateVector(FVector::YAxisVector);
			++AddCount;
		}
	}

	if (AddCount == 0)
	{
		// Preserve original behavior: no fixed bones -> leave outputs unchanged.
		return;
	}

	float InvCount = 1.0f / static_cast<float>(AddCount);
	OutCenterPos = FixedPosition * InvCount;

	// Normalize averaged normal
	FVector Normal = SumNormal.GetSafeNormal();
	if (Normal.IsNearlyZero())
	{
		// Fallback to identity rotation if we cannot form a valid normal
		OutCenterRot = FQuat::Identity;
		return;
	}

	// Orthogonalize tangent against the computed normal (Gram-Schmidt)
	FVector Tangent = SumTangent - FVector::DotProduct(SumTangent, Normal) * Normal;

	// If tangent is nearly zero after projection, pick a stable arbitrary perpendicular tangent
	const float EpsSq = PhysicsDefine::Epsilon * PhysicsDefine::Epsilon;
	if (Tangent.SizeSquared() <= EpsSq)
	{
		// Choose an axis that is least aligned with Normal to build a stable perpendicular tangent
		FVector ReferenceAxis = (FMath::Abs(Normal.X) < 0.9f) ? FVector::XAxisVector : FVector::YAxisVector;
		Tangent = FVector::CrossProduct(Normal, ReferenceAxis);
		if (Tangent.SizeSquared() <= EpsSq)
		{
			// As a last resort, try Z axis
			Tangent = FVector::CrossProduct(Normal, FVector::ZAxisVector);
			if (Tangent.SizeSquared() <= EpsSq)
			{
				// Can't form a valid tangent, fallback to identity rotation
				OutCenterRot = FQuat::Identity;
				return;
			}
		}
	}

	Tangent = Tangent.GetSafeNormal();

	// Compute binormal to complete orthonormal basis
	FVector BiNormal = FVector::CrossProduct(Normal, Tangent);
	if (BiNormal.IsNearlyZero())
	{
		// If cross produced degenerate vector, fallback
		OutCenterRot = FQuat::Identity;
		return;
	}
	BiNormal = BiNormal.GetSafeNormal();

	// Build rotation matrix (columns: X, Y, Z) and convert to quaternion
	FMatrix RootRotationMatrix = FMatrix(Normal, Tangent, BiNormal, FVector::Zero());
	OutCenterRot = RootRotationMatrix.ToQuat();
	OutCenterRot.Normalize();
}
UE_ENABLE_OPTIMIZATION
