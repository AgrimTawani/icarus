#include "Icarus7AirSimPawn.h"

#include "Components/BoxComponent.h"
#include "Components/ChildActorComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/RotatingMovementComponent.h"
#include "PIPCamera.h"
#include "UObject/UObjectGlobals.h"

namespace
{
constexpr float GeometryOriginAboveCgCm = 5.9009901f;

const TCHAR* VisualAssetPaths[] = {
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_AGXCooling.SM_Icarus7_AGXCooling"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_ARKFlow.SM_Icarus7_ARKFlow"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_ARKFlowMount.SM_Icarus7_ARKFlowMount"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Arm_FL.SM_Icarus7_Arm_FL"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Arm_FR.SM_Icarus7_Arm_FR"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Arm_RL.SM_Icarus7_Arm_RL"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Arm_RR.SM_Icarus7_Arm_RR"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Battery.SM_Icarus7_Battery"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_BatteryLead.SM_Icarus7_BatteryLead"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_BatteryTray.SM_Icarus7_BatteryTray"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Body.SM_Icarus7_Body"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_CompanionDeck.SM_Icarus7_CompanionDeck"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_CompanionPower.SM_Icarus7_CompanionPower"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_ESCMount.SM_Icarus7_ESCMount"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_FinalWiring.SM_Icarus7_FinalWiring"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_FrontCameraMount.SM_Icarus7_FrontCameraMount"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_HRTK.SM_Icarus7_HRTK"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_HRTKMount.SM_Icarus7_HRTKMount"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_JetsonAGXOrin.SM_Icarus7_JetsonAGXOrin"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_LandingRails.SM_Icarus7_LandingRails"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_LogitechBRIO.SM_Icarus7_LogitechBRIO"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_MID360.SM_Icarus7_MID360"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_MiniBaseboard.SM_Icarus7_MiniBaseboard"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Motor_FL.SM_Icarus7_Motor_FL"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Motor_FR.SM_Icarus7_Motor_FR"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Motor_RL.SM_Icarus7_Motor_RL"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Motor_RR.SM_Icarus7_Motor_RR"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_PM02D.SM_Icarus7_PM02D"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_PM02DMount.SM_Icarus7_PM02DMount"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_PM9A1a_NVMe.SM_Icarus7_PM9A1a_NVMe"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Pixhawk6XPro.SM_Icarus7_Pixhawk6XPro"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_PixhawkIsolators.SM_Icarus7_PixhawkIsolators"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_PixhawkMount.SM_Icarus7_PixhawkMount"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Pololu12V.SM_Icarus7_Pololu12V"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_PowerWiring.SM_Icarus7_PowerWiring"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_RP3Receiver.SM_Icarus7_RP3Receiver"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_RealSenseD435i.SM_Icarus7_RealSenseD435i"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_ReceiverAntennas.SM_Icarus7_ReceiverAntennas"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_RegulatorMount.SM_Icarus7_RegulatorMount"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_RogueAGX202.SM_Icarus7_RogueAGX202"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_SensorMounts.SM_Icarus7_SensorMounts"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Straps.SM_Icarus7_Straps"),
    TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Tekko32_ESC.SM_Icarus7_Tekko32_ESC")
};
}

AIcarus7AirSimPawn::AIcarus7AirSimPawn()
{
    PrimaryActorTick.bCanEverTick = true;
    AutoPossessPlayer = EAutoReceiveInput::Disabled;
    AutoPossessAI = EAutoPossessAI::Disabled;

    VehicleRoot = CreateDefaultSubobject<USceneComponent>(TEXT("VehicleRoot"));
    SetRootComponent(VehicleRoot);

    // The Unreal mesh origin is the modeled frame origin. AirSim physics acts
    // at the component-derived centre of gravity, 59.009901 mm below it.
    VisualRoot = CreateDefaultSubobject<USceneComponent>(TEXT("VisualRoot"));
    VisualRoot->SetupAttachment(VehicleRoot);
    VisualRoot->SetRelativeLocation(FVector(0.0f, 0.0f, GeometryOriginAboveCgCm));

    for (int32 Index = 0; Index < UE_ARRAY_COUNT(VisualAssetPaths); ++Index) {
        const FString Name = FString::Printf(TEXT("Icarus7Visual%02d"), Index);
        AddVisualMesh(*Name, VisualAssetPaths[Index]);
    }

    // AirSim/ArduPilot Quad-X actuator order: FR, RL, FL, RR.
    // The supplied propeller assets retain global mesh coordinates, so each
    // mesh is offset beneath a pivot at its motor centre before it is rotated.
    AddRotor(0, TEXT("FR"),
             TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Prop_FR.SM_Icarus7_Prop_FR"),
             FVector(17.677701f, 17.677701f, 0.209954f));
    AddRotor(1, TEXT("RL"),
             TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Prop_RL.SM_Icarus7_Prop_RL"),
             FVector(-17.677701f, -17.677701f, 0.209954f));
    AddRotor(2, TEXT("FL"),
             TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Prop_FL.SM_Icarus7_Prop_FL"),
             FVector(17.677701f, -17.677701f, 0.209954f));
    AddRotor(3, TEXT("RR"),
             TEXT("/Game/Icarus7FlightControlReview/Meshes/SM_Icarus7_Prop_RR.SM_Icarus7_Prop_RR"),
             FVector(-17.677701f, 17.677701f, 0.209954f));

    // Conservative compound collision excludes propeller discs. AirSim owns
    // kinematics; Unreal components report hits but do not simulate rigid-body
    // motion independently.
    AddCollisionBox(TEXT("CollisionBody"), FVector(11.5f, 8.0f, 4.5f),
                    FVector(0.0f, 0.0f, 3.0f));
    AddCollisionBox(TEXT("CollisionLandingGear"), FVector(11.0f, 7.0f, 5.3f),
                    FVector(0.0f, 0.0f, -2.5f));
    AddCollisionBox(TEXT("CollisionArmFR"), FVector(10.5f, 1.8f, 1.4f),
                    FVector(8.5f, 8.5f, 5.2f), 45.0f);
    AddCollisionBox(TEXT("CollisionArmRL"), FVector(10.5f, 1.8f, 1.4f),
                    FVector(-8.5f, -8.5f, 5.2f), 45.0f);
    AddCollisionBox(TEXT("CollisionArmFL"), FVector(10.5f, 1.8f, 1.4f),
                    FVector(8.5f, -8.5f, 5.2f), -45.0f);
    AddCollisionBox(TEXT("CollisionArmRR"), FVector(10.5f, 1.8f, 1.4f),
                    FVector(-8.5f, 8.5f, 5.2f), -45.0f);

    // Exact names are consumed in AFlyingPawn::initializeForBeginPlay().
    FrontCenterCamera = AddCamera(TEXT("FrontCenterCamera"), FVector(11.99f, 0.0f, 5.10f), FRotator::ZeroRotator);
    FrontRightCamera = AddCamera(TEXT("FrontRightCamera"), FVector(11.99f, 3.0f, 5.10f), FRotator::ZeroRotator);
    FrontLeftCamera = AddCamera(TEXT("FrontLeftCamera"), FVector(11.99f, -3.0f, 5.10f), FRotator::ZeroRotator);
    BackCenterCamera = AddCamera(TEXT("BackCenterCamera"), FVector(-10.0f, 0.0f, 5.0f), FRotator(0.0f, 180.0f, 0.0f));
    BottomCenterCamera = AddCamera(TEXT("BottomCenterCamera"), FVector(10.10f, 0.0f, -2.09f), FRotator(-90.0f, 0.0f, 0.0f));
}

UStaticMeshComponent* AIcarus7AirSimPawn::AddVisualMesh(const TCHAR* ComponentName,
                                                        const TCHAR* AssetPath)
{
    UStaticMeshComponent* Component = CreateDefaultSubobject<UStaticMeshComponent>(ComponentName);
    Component->SetupAttachment(VisualRoot);
    Component->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    Component->SetGenerateOverlapEvents(false);
    UStaticMesh* Mesh = Cast<UStaticMesh>(StaticLoadObject(UStaticMesh::StaticClass(), nullptr, AssetPath));
    Component->SetStaticMesh(Mesh);
    VisualMeshes.Add(Component);
    return Component;
}

UChildActorComponent* AIcarus7AirSimPawn::AddCamera(const TCHAR* ComponentName,
                                                    const FVector& LocationCm,
                                                    const FRotator& Rotation)
{
    UChildActorComponent* Camera = CreateDefaultSubobject<UChildActorComponent>(ComponentName);
    Camera->SetupAttachment(VehicleRoot);
    Camera->SetChildActorClass(APIPCamera::StaticClass());
    Camera->SetRelativeLocation(LocationCm);
    Camera->SetRelativeRotation(Rotation);
    return Camera;
}

UBoxComponent* AIcarus7AirSimPawn::AddCollisionBox(const TCHAR* ComponentName,
                                                   const FVector& ExtentCm,
                                                   const FVector& LocationCm,
                                                   float YawDegrees)
{
    UBoxComponent* Box = CreateDefaultSubobject<UBoxComponent>(ComponentName);
    Box->SetupAttachment(VehicleRoot);
    Box->SetBoxExtent(ExtentCm);
    Box->SetRelativeLocation(LocationCm);
    Box->SetRelativeRotation(FRotator(0.0f, YawDegrees, 0.0f));
    Box->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    Box->SetCollisionObjectType(ECC_Pawn);
    Box->SetCollisionResponseToAllChannels(ECR_Block);
    Box->SetNotifyRigidBodyCollision(true);
    Box->SetGenerateOverlapEvents(false);
    Box->SetSimulatePhysics(false);
    CollisionBoxes.Add(Box);
    return Box;
}

void AIcarus7AirSimPawn::AddRotor(int32 AirSimIndex, const TCHAR* Label,
                                  const TCHAR* AssetPath, const FVector& PivotCm)
{
    const FString PivotName = FString::Printf(TEXT("RotorPivot%s"), Label);
    USceneComponent* Pivot = CreateDefaultSubobject<USceneComponent>(*PivotName);
    Pivot->SetupAttachment(VisualRoot);
    Pivot->SetRelativeLocation(PivotCm);

    const FString MeshName = FString::Printf(TEXT("Propeller%s"), Label);
    UStaticMeshComponent* MeshComponent = CreateDefaultSubobject<UStaticMeshComponent>(*MeshName);
    MeshComponent->SetupAttachment(Pivot);
    MeshComponent->SetRelativeLocation(-PivotCm);
    MeshComponent->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    MeshComponent->SetGenerateOverlapEvents(false);
    UStaticMesh* Mesh = Cast<UStaticMesh>(StaticLoadObject(UStaticMesh::StaticClass(), nullptr, AssetPath));
    MeshComponent->SetStaticMesh(Mesh);

    const FString RotationName = FString::Printf(TEXT("Rotation%d"), AirSimIndex);
    URotatingMovementComponent* Rotation = CreateDefaultSubobject<URotatingMovementComponent>(*RotationName);
    Rotation->SetUpdatedComponent(Pivot);
    Rotation->RotationRate = FRotator::ZeroRotator;

    RotorPivots.Add(Pivot);
    RotorMeshes.Add(MeshComponent);
    RotorMovements.Add(Rotation);
}
