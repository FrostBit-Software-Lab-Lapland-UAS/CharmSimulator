
// Fill out your copyright notice in the Description page of Project Settings.


#include "Lidar.h"
#include "Kismet/KismetMathLibrary.h"
#include "Engine/World.h"
#include <cmath>
#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"
#include "Misc/Variant.h"
#include "Kismet/KismetMathLibrary.h"
#include "Kismet/GameplayStatics.h"


// Sets default values
ALidar::ALidar()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.TickGroup = TG_PostPhysics;
}

// Called when the game starts or when spawned
void ALidar::BeginPlay()
{
	Super::BeginPlay();
    rosInstance = Cast<UROSIntegrationGameInstance>(GetGameInstance());
    // IF ROS CONNECTION IS ALREADY MADE WE CREATE ROS TOPIC IN THE START
    if (rosInstance->bIsConnected) {
        // Initialize a topic
        ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
        ExampleTopic->Init(rosInstance->ROSIntegrationCore, Description.topicName, TEXT("sensor_msgs/PointCloud2"), 0);

        // (Optional) Advertise the topic
        ExampleTopic->Advertise();
    }

    /// DATA FIELDS FOR POINTCLOUD
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

    fields.Add(x);
    fields.Add(y);
    fields.Add(z);

    pointcloud->height = 1;
    pointcloud->is_dense = true;
    pointcloud->is_bigendian = false;
    pointcloud->fields = fields;
    pointcloud->point_step = point_step;
    pointcloud->header.seq = 1;
    pointcloud->header.frame_id = "Lidar";

    Set(Description);
}

void ALidar::Set(const FLidarDescription& LidarDescription)
{
    Description = LidarDescription;
    CreateLasers();
    PointsPerChannel.resize(Description.Channels);
}

void ALidar::CreateLasers()
{
    const auto NumberOfLasers = Description.Channels;
    check(NumberOfLasers > 0u);
    const float DeltaAngle = NumberOfLasers == 1u ? 0.f : (Description.UpperFovLimit - Description.LowerFovLimit) / static_cast<float>(NumberOfLasers - 1);
    LaserAngles.Empty(NumberOfLasers);
    for (int i = 0u; i < NumberOfLasers; ++i)
    {
        const float VerticalAngle = Description.UpperFovLimit - static_cast<float>(i) * DeltaAngle;
        LaserAngles.Emplace(VerticalAngle);
    }
}

void ALidar::Tick(float DeltaTime)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(ALidar::PostPhysTick);

    // IF ROS INSTANCE IS CONNECTED AND WE DONT HAVE TOPIC MADE YET
    if (rosInstance->bIsConnected && !IsValid(ExampleTopic)) {
        // Initialize a topic
        ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
        ExampleTopic->Init(rosInstance->ROSIntegrationCore, Description.topicName, TEXT("sensor_msgs/PointCloud2"), 0);

        // (Optional) Advertise the topic
        ExampleTopic->Advertise();
    }

    SimulateLidar(DeltaTime);
}

void ALidar::SimulateLidar(const float DeltaTime)
{
    TRACE_CPUPROFILER_EVENT_SCOPE(ALidar::SimulateLidar);
    const uint32 ChannelCount = Description.Channels;
    const uint32 PointsToScanWithOneLaser = FMath::RoundHalfFromZero(Description.PointsPerSecond * DeltaTime / float(ChannelCount));

    if (PointsToScanWithOneLaser <= 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("No points to scan"));
        return;
    }

    check(ChannelCount == LaserAngles.Num());

	ResetRecordedHits(ChannelCount, PointsToScanWithOneLaser); /// CLEAR DATA

    const float AngleDistanceOfTick = Description.RotationFrequency * Description.HorizontalFov * DeltaTime; // EXP. SYNCRHONOUS = 10 * 360 *0.10 = 360
    const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;                // HOW BIG IS ANGLE BETWEEN TWO POINTS

    //GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
    {
        TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
        ParallelFor(ChannelCount, [&](int32 idxChannel) {
            TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);

            FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
            TraceParams.bTraceComplex = true;
            TraceParams.bReturnPhysicalMaterial = false;

            for (auto idxPtsOneLaser = 0u; idxPtsOneLaser < PointsToScanWithOneLaser; idxPtsOneLaser++) {
                FHitResult HitResult;
                const float VertAngle = LaserAngles[idxChannel];
                const float HorizAngle = std::fmod(CurrentHorizontalAngle + AngleDistanceOfLaserMeasure * idxPtsOneLaser, Description.HorizontalFov) - Description.HorizontalFov / 2;

                if (ShootLaser(VertAngle, HorizAngle, HitResult, TraceParams)) {
                    WritePointAsync(idxChannel, HitResult);
                }
            };
            });
    }
    //GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();
	FTransform ActorTransf = GetTransform();
	ComputeAndSaveDetections(ActorTransf);
    
    // IF ROS IS CONNECTED AND TOPIC IS VALID WE PUBLISH DATA
    if (rosInstance->bIsConnected && IsValid(ExampleTopic) && sizeof(pointcloud->data_ptr) > 0) {
        bool didPub = ExampleTopic->Publish(pointcloud);
    }

    delete[] lidarDataByteArray;

    CurrentHorizontalAngle = std::fmod(CurrentHorizontalAngle + AngleDistanceOfTick, Description.HorizontalFov);
}

void ALidar::ResetRecordedHits(uint32_t Channels, uint32_t MaxPointsPerChannel) {
    
    pointcloud->header.time = FROSTime::Now();
    RecordedHits.resize(Channels);
    HitLocations.Empty();

    for (auto& hits : RecordedHits) {
        hits.clear();
        hits.reserve(MaxPointsPerChannel);
    }
}

/// SAVE HIT RESULT TO LASER SLOT
void ALidar::WritePointAsync(uint32_t channel, FHitResult& detection) {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
    //DEBUG_ASSERT(GetChannelCount() > channel);
    RecordedHits[channel].emplace_back(detection);
}

/// CREATE POINTCLOUD2
void ALidar::ComputeAndSaveDetections(const FTransform& SensorTransform) {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
    TArray<float> floats;
    int32 numberOfPoints = 0;

    /// THIS NEEDS TO BE DONE BEFORE WE CAN CONVER FLOATS TO BYTES
    /// WE GET L
    for (int idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
        for (auto& hit : RecordedHits[idxChannel]) {
            HitLocations.Add(hit.ImpactPoint);
            FVector hitLocation = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), hit.ImpactPoint); // CONVERT HIT LOCATION FROM WORLD SPACE TO LOCAL SPACE
            floats.Add(hitLocation.X);
            floats.Add(hitLocation.Y);
            floats.Add(hitLocation.Z);
            numberOfPoints = numberOfPoints + 1; 
        }
    }

    lidarDataByteArray = new uint8[numberOfPoints * point_step];

    for (int index = 0; index < floats.Num(); index++) {
        float f = floats[index];

        unsigned char const* p = reinterpret_cast<unsigned char const*>(&f);

        for (std::size_t i = 0; i != sizeof(float); ++i)
        {
            lidarDataByteArray[index * 4 + i] = (uint8_t)p[i];
        }
    }

    pointcloud->data_ptr = lidarDataByteArray;
    pointcloud->width = numberOfPoints;             /// HOW MANY POINTS IN TOTAL 
    pointcloud->row_step = numberOfPoints * point_step;     /// LENGHT OF DATA IN BYTES

}

/// SHOOTLASER AND RETURN TRUE IF WE HIT SOMETHING. SET HITVALUE TO HITRESULT WICH IS POINTER
bool ALidar::ShootLaser(const float VerticalAngle, const float HorizontalAngle, FHitResult& HitResult, FCollisionQueryParams& TraceParams) const
{
    TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);

    FHitResult HitInfo(ForceInit);

    FTransform ActorTransf = GetTransform();
    FVector LidarBodyLoc = ActorTransf.GetLocation() +FVector(0.0f, 0.0f, 4.0f); //We must add Z offset
    FRotator LidarBodyRot = ActorTransf.Rotator();

    FRotator LaserRot(VerticalAngle, HorizontalAngle, 0);  // float InPitch, float InYaw, float InRoll
    FRotator ResultRot = UKismetMathLibrary::ComposeRotators(
        LaserRot,
        LidarBodyRot
    );

    const auto Range = Description.Range * 100;
    FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;
    GetWorld()->LineTraceSingleByChannel(
        HitInfo,
        LidarBodyLoc,
        EndTrace,
        ECC_GameTraceChannel5, //This is our Laser_trace channel
        TraceParams,
        FCollisionResponseParams::DefaultResponseParam
    );

    if (HitInfo.bBlockingHit) {
        HitResult = HitInfo;
        return true;
    }
    else {
        return false;
    }
}

