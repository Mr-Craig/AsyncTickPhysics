#include "AsyncTickPawn.h"

#include "AsyncTickManager.h"

void AAsyncTickPawn::BeginPlay()
{
	Super::BeginPlay();

	FAsyncTickManager* AsyncTickManager = FAsyncTickManager::GetPhysicsManagerFromScene(this->GetWorld()->GetPhysicsScene());
	AsyncTickManager->AddPawn(this);
}

void AAsyncTickPawn::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
	
	FAsyncTickManager* AsyncTickManager = FAsyncTickManager::GetPhysicsManagerFromScene(this->GetWorld()->GetPhysicsScene());
	AsyncTickManager->RemovePawn(this);
}
