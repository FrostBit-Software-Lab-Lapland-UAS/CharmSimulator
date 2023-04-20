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
        ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
        ExampleTopic->Init(rosInstance->ROSIntegrationCore, Description.topicName, TEXT("sensor_msgs/PointCloud2"), 0);

        // (Optional) Advertise the topic
        ExampleTopic->Advertise();
    }


    TArray<ROSMessages::sensor_msgs::PointCloud2::PointField> fields;
    ROSMessages::sensor_msgs::PointCloud2::PointField x = ROSMessages::sensor_msgs::PointCloud2::PointField();
    x.name = "x";
    x.offset = 0;
    x.datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    x.count = 1;

    ROSMessages::sensor_msgs::PointCloud2::PointField y = ROSMessages::sensor_msgs::PointCloud2::PointField();
    y.name = "y";
    y.offset = 4;
    y.datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    y.count = 1;

    ROSMessages::sensor_msgs::PointCloud2::PointField z = ROSMessages::sensor_msgs::PointCloud2::PointField();
    z.name = "z";
    z.offset = 8;
    z.datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    z.count = 1;

    ROSMessages::sensor_msgs::PointCloud2::PointField range = ROSMessages::sensor_msgs::PointCloud2::PointField();
    range.name = "Range";
    range.offset = 12;
    range.datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    range.count = 1;

    ROSMessages::sensor_msgs::PointCloud2::PointField velocityField = ROSMessages::sensor_msgs::PointCloud2::PointField();
    velocityField.name = "Velocity";
    velocityField.offset = 16;
    velocityField.datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    velocityField.count = 1;

    ROSMessages::sensor_msgs::PointCloud2::PointField azimuthAngle = ROSMessages::sensor_msgs::PointCloud2::PointField();
    azimuthAngle.name = "AzimuthAngle";
    azimuthAngle.offset = 20;
    azimuthAngle.datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    azimuthAngle.count = 1;

    ROSMessages::sensor_msgs::PointCloud2::PointField elevationAngle = ROSMessages::sensor_msgs::PointCloud2::PointField();
    elevationAngle.name = "ElevationAngle";
    elevationAngle.offset = 24;
    elevationAngle.datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    elevationAngle.count = 1;

    fields.Add(x);
    fields.Add(y);
    fields.Add(z);
    fields.Add(range);
    fields.Add(velocityField);
    fields.Add(azimuthAngle);
    fields.Add(elevationAngle);
    pointcloud->header.seq = 1;
    pointcloud->header.frame_id = "Radar";
    pointcloud->is_bigendian = false;
    pointcloud->fields = fields;
    pointcloud->point_step = point_step;
    pointcloud->height = 1;
    pointcloud->is_dense = true;
    
}


void ARadar::Tick(float DeltaTime)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(ARadar::PostPhysTick);

    // IF ROS INSTANCE IS CONNECTED AND WE DONT HAVE TOPIC MADE YET
    if (rosInstance->bIsConnected && !IsValid(ExampleTopic)) {
        // Initialize a topic
        ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
        ExampleTopic->Init(rosInstance->ROSIntegrationCore, Description.topicName, TEXT("sensor_msgs/PointCloud2"), 0);

        // (Optional) Advertise the topic
        ExampleTopic->Advertise();
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
    Rays.Empty();
    Rays.Init(RayData(), NumPoints);

    FCriticalSection Mutex;
    //GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
    {
        TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
        ParallelFor(NumPoints, [&](int32 idx) 
            {
                TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);
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
                    //FVector hitLocation = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), OutHit.ImpactPoint);
                    Rays[idx].didHit = true;
                    Rays[idx].hitLocation = OutHit.ImpactPoint;
                    Rays[idx].RelativeVelocity = CalculateRelativeVelocity(OutHit, RadarLocation);
                    Rays[idx].AzimuthAndElevation = FMath::GetAzimuthAndElevation(
                        (EndLocation - RadarLocation).GetSafeNormal() * Range,
                        TransformXAxis,
                        TransformYAxis,
                        TransformZAxis
                    );

                    Rays[idx].Distance = OutHit.Distance * TO_METERS;
                }
            });
    }
    //GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();
    for (int i = (Rays.Num() - 1); i >= 0; i--) {
        if (Rays[i].didHit == false) {
            Rays.RemoveAt(i);
        }
    }

    

    uint8* uArray = new uint8[Rays.Num() * point_step];



    for (int index = 0; index < Rays.Num(); index++) {
        HitLocations.Add(Rays[index].hitLocation); // This array can be used to visualize 
        FVector hitloc = UKismetMathLibrary::TransformLocation(this->GetTransform(), Rays[index].hitLocation); // Change transform from world space to local space

        float xLocation = hitloc.X;
        float yLocation = hitloc.Y;
        float zLocation = hitloc.Z;
        float distance = Rays[index].Distance;
        float velocity = Rays[index].RelativeVelocity;
        float azimuth = Rays[index].AzimuthAndElevation.X;
        float elevation = Rays[index].AzimuthAndElevation.Y;

        //FVector hit = FVector(Rays[index].hitLocation.X, Rays[index].hitLocation.Y, Rays[index].hitLocation.Z);

        unsigned char const* xChar = reinterpret_cast<unsigned char const*>(&xLocation);
        unsigned char const* yChar = reinterpret_cast<unsigned char const*>(&yLocation);
        unsigned char const* zChar = reinterpret_cast<unsigned char const*>(&zLocation);
        unsigned char const* dChar = reinterpret_cast<unsigned char const*>(&distance);
        unsigned char const* vChar = reinterpret_cast<unsigned char const*>(&velocity);
        unsigned char const* aChar= reinterpret_cast<unsigned char const*>(&azimuth);
        unsigned char const* eChar = reinterpret_cast<unsigned char const*>(&elevation);

        for (std::size_t i = 0; i != sizeof(float); ++i)
        {
            uArray[index * 28 + i] = (uint8_t)xChar[i];
        }
        for (std::size_t i = 0; i != sizeof(float); ++i)
        {
            uArray[index * 28 + i + 4] = (uint8_t)yChar[i];
        }
        for (std::size_t i = 0; i != sizeof(float); ++i)
        {
            uArray[index * 28 + i + 8] = (uint8_t)zChar[i];
        }
        for (std::size_t i = 0; i != sizeof(float); ++i)
        {
            uArray[index * 28 + i + 12] = (uint8_t)dChar[i];
        }
        for (std::size_t i = 0; i != sizeof(float); ++i)
        {
            uArray[index * 28 + i + 16] = (uint8_t)vChar[i];
        }
        for (std::size_t i = 0; i != sizeof(float); ++i)
        {
            uArray[index * 28 + i + 20] = (uint8_t)aChar[i];
        }
        for (std::size_t i = 0; i != sizeof(float); ++i)
        {
            uArray[index * 28 + i + 24] = (uint8_t)eChar[i];
        }
    }


    pointcloud->header.time = FROSTime::Now();
    pointcloud->data_ptr = uArray;
    pointcloud->width = Rays.Num();                     /// HOW MANY POINTS IN TOTAL 
    pointcloud->row_step = Rays.Num() * point_step;     /// LENGHT OF DATA IN BYTES

    // IF ROS IS CONNECTED AND TOPIC IS VALID WE PUBLISH DATA
    if (rosInstance->bIsConnected && IsValid(ExampleTopic) && sizeof(pointcloud->data_ptr) > 0) {
        ExampleTopic->Publish(pointcloud);
    }

    delete[] uArray;
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
