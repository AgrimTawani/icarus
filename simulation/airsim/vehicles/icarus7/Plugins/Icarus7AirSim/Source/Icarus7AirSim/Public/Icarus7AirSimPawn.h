#pragma once

#include "CoreMinimal.h"
#include "Vehicles/Multirotor/FlyingPawn.h"
#include "Icarus7AirSimPawn.generated.h"

class UBoxComponent;
class UChildActorComponent;
class URotatingMovementComponent;
class USceneComponent;
class UStaticMeshComponent;

/**
 * Native AirSim pawn for the Icarus7 visual model.
 *
 * AirSim 1.8 discovers cameras and rotors by exact component name. Keep the
 * Front*Camera/BackCenterCamera/BottomCenterCamera and Rotation0..3 names
 * stable: they are a runtime ABI, not presentation labels.
 */
UCLASS()
class ICARUS7AIRSIM_API AIcarus7AirSimPawn : public AFlyingPawn
{
    GENERATED_BODY()

public:
    AIcarus7AirSimPawn();

private:
    UStaticMeshComponent* AddVisualMesh(const TCHAR* ComponentName, const TCHAR* AssetPath);
    UChildActorComponent* AddCamera(const TCHAR* ComponentName, const FVector& LocationCm,
                                    const FRotator& Rotation);
    UBoxComponent* AddCollisionBox(const TCHAR* ComponentName, const FVector& ExtentCm,
                                  const FVector& LocationCm, float YawDegrees = 0.0f);
    void AddRotor(int32 AirSimIndex, const TCHAR* Label, const TCHAR* AssetPath,
                  const FVector& PivotCm);

    UPROPERTY(VisibleAnywhere, Category = "Icarus7")
    USceneComponent* VehicleRoot;

    UPROPERTY(VisibleAnywhere, Category = "Icarus7")
    USceneComponent* VisualRoot;

    UPROPERTY(VisibleAnywhere, Category = "Icarus7")
    TArray<UStaticMeshComponent*> VisualMeshes;

    UPROPERTY(VisibleAnywhere, Category = "Icarus7")
    TArray<USceneComponent*> RotorPivots;

    UPROPERTY(VisibleAnywhere, Category = "Icarus7")
    TArray<UStaticMeshComponent*> RotorMeshes;

    UPROPERTY(VisibleAnywhere, Category = "Icarus7")
    TArray<URotatingMovementComponent*> RotorMovements;

    UPROPERTY(VisibleAnywhere, Category = "Icarus7")
    TArray<UBoxComponent*> CollisionBoxes;

    UPROPERTY(VisibleAnywhere, Category = "Icarus7")
    UChildActorComponent* FrontCenterCamera;

    UPROPERTY(VisibleAnywhere, Category = "Icarus7")
    UChildActorComponent* FrontRightCamera;

    UPROPERTY(VisibleAnywhere, Category = "Icarus7")
    UChildActorComponent* FrontLeftCamera;

    UPROPERTY(VisibleAnywhere, Category = "Icarus7")
    UChildActorComponent* BackCenterCamera;

    UPROPERTY(VisibleAnywhere, Category = "Icarus7")
    UChildActorComponent* BottomCenterCamera;
};
