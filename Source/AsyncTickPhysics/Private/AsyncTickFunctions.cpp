#include "AsyncTickFunctions.h"

#include "PhysicsProxy/SingleParticlePhysicsProxy.h"

void UAsyncTickFunctions::ATP_AddForce(UPrimitiveComponent* Component, FVector Force, bool bAccelChange)
{
	if(!Component)
		return;
	
	if(const FBodyInstance* BodyInstance = Component->GetBodyInstance())
	{
		if(auto Handle = BodyInstance->ActorHandle)
		{
			if(Chaos::FRigidBodyHandle_Internal* RigidHandle = Handle->GetPhysicsThreadAPI())
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
	}
}

void UAsyncTickFunctions::ATP_AddForceAtPosition(UPrimitiveComponent* Component, FVector Position, FVector Force)
{
	if(!Component)
		return;
	
	if(const FBodyInstance* BodyInstance = Component->GetBodyInstance())
	{
		if(auto Handle = BodyInstance->ActorHandle)
		{
			if(Chaos::FRigidBodyHandle_Internal* RigidHandle = Handle->GetPhysicsThreadAPI())
			{
				const Chaos::FVec3 WorldCOM = Chaos::FParticleUtilitiesGT::GetCoMWorldPosition(RigidHandle);
				const Chaos::FVec3 WorldTorque = Chaos::FVec3::CrossProduct(Position - WorldCOM, Force);
				RigidHandle->AddForce(Force, false);
				RigidHandle->AddTorque(WorldTorque, false);
			}
		}
	}
}

void UAsyncTickFunctions::ATP_AddTorque(UPrimitiveComponent* Component, FVector Torque, bool bAccelChange)
{
	if(!Component)
		return;
	
	if(const FBodyInstance* BodyInstance = Component->GetBodyInstance())
	{
		if(auto Handle = BodyInstance->ActorHandle)
		{
			if(Chaos::FRigidBodyHandle_Internal* RigidHandle = Handle->GetPhysicsThreadAPI())
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
	}
}

void UAsyncTickFunctions::ATP_AddImpulse(UPrimitiveComponent* Component, FVector Impulse, bool bVelChange)
{
	if(!Component)
		return;
	
	if(const FBodyInstance* BodyInstance = Component->GetBodyInstance())
	{
		if(auto Handle = BodyInstance->ActorHandle)
		{
			if(Chaos::FRigidBodyHandle_Internal* RigidHandle = Handle->GetPhysicsThreadAPI())
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
	}
}

void UAsyncTickFunctions::ATP_AddImpulseAtPosition(UPrimitiveComponent* Component, FVector Position, FVector Impulse)
{
	if(!Component)
		return;
	
	if(const FBodyInstance* BodyInstance = Component->GetBodyInstance())
	{
		if(auto Handle = BodyInstance->ActorHandle)
		{
			if(Chaos::FRigidBodyHandle_Internal* RigidHandle = Handle->GetPhysicsThreadAPI())
			{
				const Chaos::FVec3 WorldCOM = Chaos::FParticleUtilitiesGT::GetCoMWorldPosition(RigidHandle);
				const Chaos::FVec3 AngularImpulse = Chaos::FVec3::CrossProduct(Position - WorldCOM, Impulse);
				RigidHandle->SetLinearImpulse(RigidHandle->LinearImpulse() + Impulse, false);
				RigidHandle->SetAngularImpulse(RigidHandle->AngularImpulse() + AngularImpulse, false);
				
				
			}
		}
	}
}

void UAsyncTickFunctions::ATP_AddAngularImpulseInRadians(UPrimitiveComponent* Component, FVector Torque,
	bool bVelChange)
{
	if(!IsValid(Component))
		return;

	if(const FBodyInstance* BodyInstance = Component->GetBodyInstance())
	{
		if(auto Handle = BodyInstance->ActorHandle)
		{
			if(Chaos::FRigidBodyHandle_Internal* RigidHandle = Handle->GetPhysicsThreadAPI())
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
	}
}

void UAsyncTickFunctions::ATP_AddAngularImpulseInDegrees(UPrimitiveComponent* Component, FVector Torque,
	bool bVelChange)
{
	ATP_AddAngularImpulseInRadians(Component, FMath::DegreesToRadians(Torque), bVelChange);
}

FTransform UAsyncTickFunctions::ATP_GetTransform(UPrimitiveComponent* Component)
{
	if(IsValid(Component))
	{
		if(const FBodyInstance* BodyInstance = Component->GetBodyInstance())
		{
			if(auto Handle = BodyInstance->ActorHandle)
			{
				if(Chaos::FRigidBodyHandle_Internal* RigidHandle = Handle->GetPhysicsThreadAPI())
				{
					const Chaos::FRigidTransform3 WorldCOM = Chaos::FParticleUtilitiesGT::GetActorWorldTransform(RigidHandle);

					return WorldCOM;
				}
			}

			/*return BodyInstance->GetUnrealWorldTransform();*/
		}
	}
	return FTransform();
}
