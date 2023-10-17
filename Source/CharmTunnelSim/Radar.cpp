// Fill out your copyright notice in the Description page of Project Settings.


#include "Radar.h"
#include "Engine/World.h"
#include "Kismet/KismetMathLibrary.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"
#include "MathUtil.h"

// Sets default values
ARadar::ARadar()
{
    // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.TickGroup = TG_PostPhysics;

    TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
    TraceParams.bTraceComplex = true;
    TraceParams.bReturnPhysicalMaterial = false;

}

// Called when the game starts or when spawned
void ARadar::BeginPlay()
{
	Super::BeginPlay();
    PrevLocation = GetActorLocation();
    SetHorizontalFOV(Description.HorizontalFOV);
    SetPointsPerSecond(Description.PointsPerSecond);
    SetRange(Description.Range);
    SetVerticalFOV(Description.VerticalFOV);

    rosInstance = Cast<UROSIntegrationGameInstance>(GetGameInstance());
    // IF ROS CONNECTION IS ALREADY MADE WE CREATE ROS TOPIC IN THE START
    if (rosInstance->bIsConnected) {
        // Initialize a topic
        RadarDataTopic = NewObject<UTopic>(UTopic::StaticClass());
        RadarDataTopic->Init(rosInstance->ROSIntegrationCore, Description.topicName, TEXT("sensor_msgs/PointCloud2"), 0);

        // (Optional) Advertise the topic
        RadarDataTopic->Advertise();
    }


    /// DATA FIELDS FOR POINTCLOUD
    pointcloud->fields.SetNum(7);

    pointcloud->fields[0].name = "x";
    pointcloud->fields[0].offset = 0;
    pointcloud->fields[0].datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    pointcloud->fields[0].count = 1;

    pointcloud->fields[1].name = "y";
    pointcloud->fields[1].offset = 4;
    pointcloud->fields[1].datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    pointcloud->fields[1].count = 1;

    pointcloud->fields[2].name = "z";
    pointcloud->fields[2].offset = 8;
    pointcloud->fields[2].datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    pointcloud->fields[2].count = 1;

    pointcloud->fields[3].name = "Range";
    pointcloud->fields[3].offset = 12;
    pointcloud->fields[3].datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    pointcloud->fields[3].count = 1;

    pointcloud->fields[4].name = "Velocity";
    pointcloud->fields[4].offset = 16;
    pointcloud->fields[4].datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    pointcloud->fields[4].count = 1;

    pointcloud->fields[5].name = "AzimuthAngle";
    pointcloud->fields[5].offset = 20;
    pointcloud->fields[5].datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    pointcloud->fields[5].count = 1;

    pointcloud->fields[6].name = "ElevationAngle";
    pointcloud->fields[6].offset = 24;
    pointcloud->fields[6].datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    pointcloud->fields[6].count = 1;

    pointcloud->header.seq = 1;
    pointcloud->header.frame_id = "Radar";
    pointcloud->is_bigendian = false;
    pointcloud->point_step = point_step;
    pointcloud->height = 1;
    pointcloud->is_dense = true;
    
}


void ARadar::Tick(float DeltaTime)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(ARadar::PostPhysTick);

    // IF ROS INSTANCE IS CONNECTED AND WE DONT HAVE TOPIC MADE YET
    if (rosInstance->bIsConnected && !IsValid(RadarDataTopic)) {
        // Initialize a topic
        RadarDataTopic = NewObject<UTopic>(UTopic::StaticClass());
        RadarDataTopic->Init(rosInstance->ROSIntegrationCore, Description.topicName, TEXT("sensor_msgs/PointCloud2"), 0);

        // (Optional) Advertise the topic
        RadarDataTopic->Advertise();
    }

    CalculateCurrentVelocity(DeltaTime);

    SendLineTraces(DeltaTime);
}


void ARadar::SetHorizontalFOV(float NewHorizontalFOV)
{
    HorizontalFOV = NewHorizontalFOV;
}

void  ARadar::SetVerticalFOV(float NewVerticalFOV)
{
    VerticalFOV = NewVerticalFOV;
}

void ARadar::SetRange(float NewRange)
{
    Range = NewRange;
}

void ARadar::SetPointsPerSecond(int NewPointsPerSecond)
{
    PointsPerSecond = NewPointsPerSecond;
}

void ARadar::CalculateCurrentVelocity(const float DeltaTime)
{
    const FVector RadarLocation = GetActorLocation();
    CurrentVelocity = (RadarLocation - PrevLocation) / DeltaTime;
    PrevLocation = RadarLocation;
}

void ARadar::SendLineTraces(float DeltaTime)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(ARadar::SendLineTraces);

    constexpr float TO_METERS = 1e-2;
    const FTransform& ActorTransform = GetActorTransform();
    const FRotator& TransformRotator = ActorTransform.Rotator();
    const FVector& RadarLocation = GetActorLocation();
    const FVector ForwardVector = GetActorForwardVector();
    // const FVector& ForwardVector = GetActorForwardVector();
    const FVector TransformXAxis = ActorTransform.GetUnitAxis(EAxis::X);
    const FVector TransformYAxis = ActorTransform.GetUnitAxis(EAxis::Y);
    const FVector TransformZAxis = ActorTransform.GetUnitAxis(EAxis::Z);

    // Maximum radar radius in horizontal and vertical direction
    const float MaxRx = FMath::Tan(FMath::DegreesToRadians(HorizontalFOV * 0.5f)) * Range;
    const float MaxRy = FMath::Tan(FMath::DegreesToRadians(VerticalFOV * 0.5f)) * Range;
    const int NumPoints = (int)(PointsPerSecond * DeltaTime);

    if (NumPoints <= 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("No points to scan"));
        return;
    }

    HitLocations.Empty();
    RecordedHits.clear();
    RecordedHits.resize(NumPoints);
    
    RayArray.clear();
    RayArray.resize(NumPoints);

    FCriticalSection Mutex;

    ParallelFor(NumPoints, [&](int32 idx)
        {
            FHitResult OutHit(ForceInit);
            FRotator rot;
            rot.Pitch = VerticalFOV * 0.5f * FMath::RandRange(-1.0, 1.0);
            rot.Yaw = HorizontalFOV * 0.5f * FMath::RandRange(-1.0, 1.0);
            rot.Roll = 0.0;

            const FVector EndLocation = RadarLocation + rot.RotateVector({ ForwardVector }) * Range;

            GetWorld()->LineTraceSingleByChannel(
                OutHit,
                RadarLocation,
                EndLocation,
                ECC_GameTraceChannel5, //This is our Laser_trace channel
                TraceParams,
                FCollisionResponseParams::DefaultResponseParam
            );

            const TWeakObjectPtr<AActor> HittedActor = OutHit.GetActor();
            if (OutHit.bBlockingHit && HittedActor.Get()) {
                FVector hitLocation = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), OutHit.ImpactPoint);
                hitLocation = GetActorRotation().RotateVector(hitLocation);
                FVector2D AzimuthAndElevation = FMath::GetAzimuthAndElevation(
                    (EndLocation - RadarLocation).GetSafeNormal() * Range,
                    TransformXAxis,
                    TransformYAxis,
                    TransformZAxis
                );
                FRayData ray = { hitLocation.X, hitLocation.Y, hitLocation.Z,
                    OutHit.Distance * TO_METERS, CalculateRelativeVelocity(OutHit, RadarLocation),
                    AzimuthAndElevation.X, AzimuthAndElevation.Y };
                RayArray[idx] = ray;
                RecordedHits[idx] = OutHit.ImpactPoint;
            }
        });

    if (RayArray.size() < 1) {
        return;
    }

    for (auto& hit : RecordedHits) {
        // Add hit location to HitLocations array
        HitLocations.Add(hit);
    }

    const FRayData* RaysDataPtr = RayArray.data();
    pointcloud->data_ptr = reinterpret_cast<uint8*>(const_cast<FRayData*>(RaysDataPtr));

    pointcloud->width = RayArray.size();    /// HOW MANY POINTS IN TOTAL 
    pointcloud->row_step = pointcloud->width * pointcloud->point_step;  /// LENGHT OF DATA IN BYTES
    pointcloud->header.time = FROSTime::Now();

    // IF ROS IS CONNECTED AND TOPIC IS VALID WE PUBLISH DATA
    if (rosInstance->bIsConnected && IsValid(RadarDataTopic) && sizeof(pointcloud->data_ptr) > 0) {
        RadarDataTopic->Publish(pointcloud);
    }
}

float ARadar::CalculateRelativeVelocity(const FHitResult& OutHit, const FVector& RadarLocation)
{
    constexpr float TO_METERS = 1e-2;

    const TWeakObjectPtr<AActor> HittedActor = OutHit.GetActor();
    const FVector TargetVelocity = HittedActor->GetVelocity();
    const FVector TargetLocation = OutHit.ImpactPoint;
    const FVector Direction = (TargetLocation - RadarLocation).GetSafeNormal();
    const FVector DeltaVelocity = (TargetVelocity - CurrentVelocity);
    const float V = TO_METERS * FVector::DotProduct(DeltaVelocity, Direction);

    return V;
}
