// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "EnumContainer.h"
#include "ROSIntegration/Public/sensor_msgs/PointCloud2.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "Lidar.generated.h"

UCLASS()
class ALidar : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ALidar();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:
	ALidar(const FObjectInitializer& ObjectInitializer)
	{
		PrimaryActorTick.TickGroup = TG_PostPhysics;
		PrimaryActorTick.bCanEverTick = true;
		PrimaryActorTick.bStartWithTickEnabled = true;
	}

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ROSS")
	UROSIntegrationGameInstance* rosInstance;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Parameters", meta = (ExposeOnSpawn = "true"))
	int32 sensorIndex;

	virtual void Tick(float DeltaTime) override;

	UFUNCTION(BlueprintCallable)
	virtual void Set(const FLidarDescription& LidarDescription);

	TSharedPtr<ROSMessages::sensor_msgs::PointCloud2> pointcloud = MakeShareable(new ROSMessages::sensor_msgs::PointCloud2);
	int32 point_step = 16; /// THIS INDICATES HOW MANY BYTES SINGLE POINT HOLDS. CHANGE THIS IF FIELDS ARE ADDED
	float CurrentHorizontalAngle = 0.0f;

protected:

	/// Creates a Laser for each channel.
	void CreateLasers();

	/// Updates LidarMeasurement with the points read in DeltaTime.
	void SimulateLidar(const float DeltaTime);

	/// Shoot a laser ray-trace, return whether the laser hit something.
	bool ShootLaser(const float VerticalAngle, float HorizontalAngle, FHitResult& HitResult, FCollisionQueryParams& TraceParams) const;

	/// Clear the recorded data structure
	void ResetRecordedHits(uint32_t Channels, uint32_t MaxPointsPerChannel);

	/// Compute all raw detection information
	void ComputeRawDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const;

	/// Saving the hits the raycast returns per channel
	void WritePointAsync(uint32_t Channel, FHitResult& Detection);

	/// This method uses all the saved FHitResults, compute the
	/// RawDetections and then send it to the LidarData structure.
	void ComputeAndSaveDetections(const FTransform& SensorTransform);

	/// Saving the hits the raycast returns per channel
	float CalculateIntensity(FHitResult& Detection, const FTransform& SensorTransform);

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Parameters", meta = (ExposeOnSpawn = "true"))
	FLidarDescription Description;

	// Locations used for visualizing lidar in niagara
	UPROPERTY(BlueprintReadOnly)
	TArray<FVector> HitLocations;
	UPROPERTY()
	UTopic* LidarDataTopic;
	uint8* lidarDataByteArray;

	TArray<float> LaserAngles;

	int tickCount = 0;

	std::vector<std::vector<FHitResult>> RecordedHits;
	std::vector<FPointData> PointArray;

	std::vector<uint32_t> PointsPerChannel;



};
