
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
        LidarDataTopic = NewObject<UTopic>(UTopic::StaticClass());
        LidarDataTopic->Init(rosInstance->ROSIntegrationCore, Description.topicName, TEXT("sensor_msgs/PointCloud2"), 0);

        // (Optional) Advertise the topic
        LidarDataTopic->Advertise();
    }

    /// DATA FIELDS FOR POINTCLOUD
    pointcloud->fields.SetNum(4);

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

    pointcloud->fields[3].name = "intensity";
    pointcloud->fields[3].offset = 12;
    pointcloud->fields[3].datatype = ROSMessages::sensor_msgs::PointCloud2::PointField::EType::FLOAT32;
    pointcloud->fields[3].count = 1;

    pointcloud->height = 1;
    pointcloud->is_dense = true;
    pointcloud->is_bigendian = false;
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
    if (rosInstance->bIsConnected && !IsValid(LidarDataTopic)) {
        // Initialize a topic
        LidarDataTopic = NewObject<UTopic>(UTopic::StaticClass());
        LidarDataTopic->Init(rosInstance->ROSIntegrationCore, Description.topicName, TEXT("sensor_msgs/PointCloud2"), 0);

        // (Optional) Advertise the topic
        LidarDataTopic->Advertise();
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

    const float AngleDistanceOfTick = Description.RotationFrequency * Description.HorizontalFov * DeltaTime; // EXP. SYNCRHONOUS = 10 * 360 *0.10 = 360
    const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;                // HOW BIG IS ANGLE BETWEEN TWO POINTS

    ResetRecordedHits(ChannelCount, PointsToScanWithOneLaser); /// CLEAR DATA


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
	FTransform ActorTransf = GetTransform();
	ComputeAndSaveDetections(ActorTransf);
  
    // If ROS is connected and the LidarDataTopic is valid, publish the point cloud data
    if (rosInstance->bIsConnected && IsValid(LidarDataTopic) && sizeof(pointcloud->data_ptr) > 0) {
        bool didPub = LidarDataTopic->Publish(pointcloud);
    }

    CurrentHorizontalAngle = std::fmod(CurrentHorizontalAngle + AngleDistanceOfTick, Description.HorizontalFov);
}

void ALidar::ResetRecordedHits(uint32_t Channels, uint32_t MaxPointsPerChannel) {
    
    pointcloud->header.time = FROSTime::Now();
    RecordedHits.resize(Channels);
    HitLocations.Empty();
    PointArray.clear();

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

float ALidar::CalculateIntensity(FHitResult& detection, const FTransform& SensorTransform) {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);

    // Calculate the distance between the Lidar sensor and the detection point
    float distance = FVector::Distance(SensorTransform.GetLocation(), detection.ImpactPoint);

    // Calculate the incidence angle
    FVector detectionDirection = (detection.ImpactPoint - SensorTransform.GetLocation()).GetSafeNormal();
    float incidenceAngle = FMath::Acos(FVector::DotProduct(detectionDirection, detection.ImpactNormal));

    // Define a surface roughness factor
    // The exact value will depend on the characteristics of the specific rock wall.
    // This value might need adjustment for more accurate simulations.
    float surfaceRoughnessFactor = 0.75f;

    float intensity = 0.0f;
    if (distance > 0.0f && distance <= Description.Range)
    {
        float distanceInMeters = distance / 100.0f; // assuming that distance is given in centimeters
        float attenFactor = exp(-Description.AtmospAttenRate * distanceInMeters); // calculate attenuation due to atmosphere
        intensity = (1.0f / pow(distanceInMeters, 2)) * attenFactor; // calculate intensity considering inverse square law and atmospheric attenuation

        // Account for incidence angle - assume perfect diffuse reflection (Lambert's cosine law)
        intensity *= FMath::Cos(incidenceAngle) * surfaceRoughnessFactor;
    }

    return intensity;
}


/// CREATE POINTCLOUD2
void ALidar::ComputeAndSaveDetections(const FTransform& SensorTransform) {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);

    // Loop through each channel
    for (int idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
        // Loop through each recorded hit in the current channel
        for (auto& hit : RecordedHits[idxChannel]) {
            // Add hit location to HitLocations array
            HitLocations.Add(hit.ImpactPoint);

            // Convert hit location from world space to local space         
            FVector hitLocation = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), hit.ImpactPoint);

            // Create a new FPointData object with the hit location coordinates
            FPointData point = { hitLocation.X, hitLocation.Y, hitLocation.Z, CalculateIntensity(hit, SensorTransform)};

            // Add the new point to the PointArray
            PointArray.push_back(point);
        }
    }

    // If PointArray is empty, exit the function
    if (PointArray.size() < 1) {
        return;
    }

    // Create a pointer to the first element of the PointArray
    const FPointData* PointsDataPtr = PointArray.data();

    // Reinterpret the pointer to PointArray's data as a pointer to bytes and assign it to pointcloud->data_ptr
    pointcloud->data_ptr = reinterpret_cast<uint8*>(const_cast<FPointData*>(PointsDataPtr));

    // Set the width of the point cloud (number of points)
    pointcloud->width = PointArray.size();

    // Calculate the total size in bytes of the point cloud data
    pointcloud->row_step = pointcloud->width * pointcloud->point_step;
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
