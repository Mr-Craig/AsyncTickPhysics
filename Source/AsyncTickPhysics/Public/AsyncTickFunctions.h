// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "AsyncTickFunctions.generated.h"

/**
 * 
 */
UCLASS()
class ASYNCTICKPHYSICS_API UAsyncTickFunctions : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
public:
	UFUNCTION(BlueprintCallable, Category = "ATP - AsyncTickPhysics")
		static void ATP_AddForce(UPrimitiveComponent* Component, FVector Force, bool bAccelChange);
	UFUNCTION(BlueprintCallable, Category = "ATP - AsyncTickPhysics")
		static void ATP_AddForceAtPosition(UPrimitiveComponent* Component, FVector Position, FVector Force);
	UFUNCTION(BlueprintCallable, Category = "ATP - AsyncTickPhysics")
		static void ATP_AddTorque(UPrimitiveComponent* Component, FVector Torque, bool bAccelChange);
	UFUNCTION(BlueprintCallable, Category = "ATP - AsyncTickPhysics")
		static void ATP_AddImpulse(UPrimitiveComponent* Component, FVector Impulse, bool bVelChange);
	UFUNCTION(BlueprintCallable, Category = "ATP - AsyncTickPhysics")
		static void ATP_AddImpulseAtPosition(UPrimitiveComponent* Component, FVector Position, FVector Impulse);
	UFUNCTION(BlueprintCallable, Category = "ATP - AsyncTickPhysics")
    	static void ATP_AddAngularImpulseInRadians(UPrimitiveComponent* Component, FVector Torque, bool bVelChange);
	UFUNCTION(BlueprintCallable, Category = "ATP - AsyncTickPhysics")
		static void ATP_AddAngularImpulseInDegrees(UPrimitiveComponent* Component, FVector Torque, bool bVelChange);

	UFUNCTION(BlueprintCallable, Category = "ATP - AsyncTickPhysics")
		static FTransform ATP_GetTransform(UPrimitiveComponent* Component);
};
