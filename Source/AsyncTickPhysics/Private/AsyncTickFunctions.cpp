#include "AsyncTickFunctions.h"

#include "Components/PrimitiveComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "PhysicsEngine/BodyInstance.h"

void UAsyncTickFunctions::ATP_AddForce(UPrimitiveComponent* Component, FVector Force, bool bAccelChange)
{
	if(Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component))
	{
		if(bAccelChange)
		{
			const float RigidMass = RigidHandle->M();
			const Chaos::FVec3 Acceleration = Force * RigidMass;
			RigidHandle->AddForce(Acceleration, false);
		}
		else
		{
			RigidHandle->AddForce(Force, false);
		}
	}
}

void UAsyncTickFunctions::ATP_AddForceAtPosition(UPrimitiveComponent* Component, FVector Position, FVector Force)
{
	if(Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component))
	{
		const Chaos::FVec3 WorldCOM = Chaos::FParticleUtilitiesGT::GetCoMWorldPosition(RigidHandle);
		const Chaos::FVec3 WorldTorque = Chaos::FVec3::CrossProduct(Position - WorldCOM, Force);
		RigidHandle->AddForce(Force, false);
		RigidHandle->AddTorque(WorldTorque, false);
	}
}

void UAsyncTickFunctions::ATP_AddTorque(UPrimitiveComponent* Component, FVector Torque, bool bAccelChange)
{
	if(Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component))
	{
		if(bAccelChange)
		{
			RigidHandle->AddTorque(Chaos::FParticleUtilitiesXR::GetWorldInertia(RigidHandle) * Torque, false);
		}
		else
		{
			RigidHandle->AddTorque(Torque, false);
		}
	}
}

void UAsyncTickFunctions::ATP_AddImpulse(UPrimitiveComponent* Component, FVector Impulse, bool bVelChange)
{
	if(Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component))
	{
		if(bVelChange)
		{
			RigidHandle->SetLinearImpulse(RigidHandle->LinearImpulse() + RigidHandle->M() * Impulse, false);
		}
		else
		{
			RigidHandle->SetLinearImpulse(RigidHandle->LinearImpulse() + Impulse, false);
		}
	}
}

void UAsyncTickFunctions::ATP_AddImpulseAtPosition(UPrimitiveComponent* Component, FVector Position, FVector Impulse)
{
	if(Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component))
	{
		const Chaos::FVec3 WorldCOM = Chaos::FParticleUtilitiesGT::GetCoMWorldPosition(RigidHandle);
		const Chaos::FVec3 AngularImpulse = Chaos::FVec3::CrossProduct(Position - WorldCOM, Impulse);
		RigidHandle->SetLinearImpulse(RigidHandle->LinearImpulse() + Impulse, false);
		RigidHandle->SetAngularImpulse(RigidHandle->AngularImpulse() + AngularImpulse, false);
		
		
	}
}

void UAsyncTickFunctions::ATP_AddAngularImpulseInRadians(UPrimitiveComponent* Component, FVector Torque,
	bool bVelChange)
{
	if(Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component))
	{
		if(bVelChange)
		{
			const Chaos::FMatrix33 WorldI = Chaos::FParticleUtilitiesXR::GetWorldInertia(RigidHandle);
			RigidHandle->SetAngularImpulse(RigidHandle->AngularImpulse() + (WorldI * Torque), false);
		} else
		{
			RigidHandle->SetAngularImpulse(RigidHandle->AngularImpulse() + Torque, false);
		}
	}
}

void UAsyncTickFunctions::ATP_AddAngularImpulseInDegrees(UPrimitiveComponent* Component, FVector Torque,
	bool bVelChange)
{
	ATP_AddAngularImpulseInRadians(Component, FMath::DegreesToRadians(Torque), bVelChange);
}

FTransform UAsyncTickFunctions::ATP_GetTransform(UPrimitiveComponent* Component)
{
	if(const Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component))
	{
		const Chaos::FRigidTransform3 WorldCOM = Chaos::FParticleUtilitiesGT::GetActorWorldTransform(RigidHandle);
		return WorldCOM;
	}
	return FTransform();
}

FVector UAsyncTickFunctions::ATP_GetLinearVelocity(UPrimitiveComponent* Component)
{
	if(const Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component))
	{
		return RigidHandle->V();
	}
	return FVector::ZeroVector;
}

FVector UAsyncTickFunctions::ATP_GetAngularVelocity(UPrimitiveComponent* Component)
{
	if(const Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component))
	{
		return RigidHandle->W();
	}
	return FVector::ZeroVector;
}

void UAsyncTickFunctions::ATP_SetLinearVelocity(UPrimitiveComponent* Component, FVector Velocity, bool bAddToCurrent,
	FName BoneName)
{
	if(Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component, BoneName))
	{
		if(bAddToCurrent)
		{
			RigidHandle->SetV(RigidHandle->V() + Velocity);
		} else
		{
			RigidHandle->SetV(Velocity);
		}
	}
}

void UAsyncTickFunctions::ATP_SetAngularVelocityInRadians(UPrimitiveComponent* Component, FVector AngVelocity,
	bool bAddToCurrent, FName BoneName)
{
	if(Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component, BoneName))
	{
		if(bAddToCurrent)
		{
			RigidHandle->SetW(RigidHandle->W() + AngVelocity);
		} else
		{
			RigidHandle->SetW(AngVelocity);
		}
	}
}

void UAsyncTickFunctions::ATP_SetAngularVelocityInDegrees(UPrimitiveComponent* Component, FVector AngVelocity,
	bool bAddToCurrent, FName BoneName)
{
	ATP_SetAngularVelocityInRadians(Component, FMath::DegreesToRadians(AngVelocity), bAddToCurrent, BoneName);
}

void UAsyncTickFunctions::ATP_SetWorldLocation(UPrimitiveComponent* Component, FVector Location)
{
	if(Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component))
	{
		const Chaos::FVec3 P = Location - RigidHandle->R().RotateVector(RigidHandle->CenterOfMass());
		RigidHandle->SetX(P);
	}
}

void UAsyncTickFunctions::ATP_SetWorldRotation(UPrimitiveComponent* Component, FRotator Rotation)
{
	if(Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component))
	{
		const Chaos::FRotation3 Q = Rotation.Quaternion() * RigidHandle->RotationOfMass().Inverse();
		RigidHandle->SetR(Q);
	}
}

void UAsyncTickFunctions::ATP_SetWorldLocationAndRotation(UPrimitiveComponent* Component, FVector Location,
	FRotator Rotation)
{
	if(Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component))
	{
		const Chaos::FRotation3 Q = Rotation.Quaternion() * RigidHandle->RotationOfMass().Inverse();
		const Chaos::FVec3 P = Location - Q.RotateVector(RigidHandle->CenterOfMass());
		RigidHandle->SetR(Q);
		RigidHandle->SetX(P);
	}
}

FVector UAsyncTickFunctions::ATP_GetLinearVelocityAtPoint(UPrimitiveComponent* Component, FVector Point, FName BoneName)
{
	if(Chaos::FRigidBodyHandle_Internal* RigidHandle = GetInternalHandle(Component, BoneName))
	{
		if(ensure(RigidHandle->CanTreatAsKinematic()))
		{
			const bool bIsRigid = RigidHandle->CanTreatAsRigid();
			const Chaos::FVec3 COM = bIsRigid ? Chaos::FParticleUtilitiesGT::GetCoMWorldPosition(RigidHandle) : static_cast<Chaos::FVec3>(Chaos::FParticleUtilitiesGT::GetActorWorldTransform(RigidHandle).GetTranslation());
			const Chaos::FVec3 Diff = Point - COM;
			return RigidHandle->V() - Chaos::FVec3::CrossProduct(Diff, RigidHandle->W());
		}
	}
	return FVector::ZeroVector;
}

Chaos::FRigidBodyHandle_Internal* UAsyncTickFunctions::GetInternalHandle(UPrimitiveComponent* Component, FName BoneName)
{
	if(IsValid(Component))
	{
		const FBodyInstance* BodyInstance = Component->GetBodyInstance(BoneName);
		
		if(BodyInstance)
		{
			if(const auto Handle = BodyInstance->ActorHandle)
			{
				if(Chaos::FRigidBodyHandle_Internal* RigidHandle = Handle->GetPhysicsThreadAPI())
				{
					return RigidHandle;
				}
			}
		}
	}
	return nullptr;
}
