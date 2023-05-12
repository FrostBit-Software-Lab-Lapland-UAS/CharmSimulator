// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "EnumContainer.h"
#include "ROSIntegration/Public/sensor_msgs/PointCloud2.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "Radar.generated.h"

UCLASS()
class CHARMTUNNELSIM_API ARadar : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ARadar();

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Parameters", meta = (ExposeOnSpawn = "true"))
	FRadarDescription Description;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Parameters", meta = (ExposeOnSpawn = "true"))
	int32 sensorIndex;

	UFUNCTION(BlueprintCallable, Category = "Radar")
	void SetHorizontalFOV(float NewHorizontalFOV);

	UFUNCTION(BlueprintCallable, Category = "Radar")
	void SetVerticalFOV(float NewVerticalFOV);

	UFUNCTION(BlueprintCallable, Category = "Radar")
	void SetRange(float NewRange);

	UFUNCTION(BlueprintCallable, Category = "Radar")
	void SetPointsPerSecond(int NewPointsPerSecond);

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ROSS")
	UROSIntegrationGameInstance* rosInstance;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Detection")
	float Range;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Detection")
	float HorizontalFOV;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Detection")
	float VerticalFOV;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Detection")
	int PointsPerSecond;


	UPROPERTY(BlueprintReadOnly)
	TArray<FVector> HitLocations;
	UPROPERTY()
	UTopic* RadarDataTopic;


	TSharedPtr<ROSMessages::sensor_msgs::PointCloud2> pointcloud = MakeShareable(new ROSMessages::sensor_msgs::PointCloud2);
	int32 point_step = 28; /// THIS INDICATES HOW MANY BYTES SINGLE POINT HOLDS. CHANGE THIS IF FIELDS ARE ADDED


	FCollisionQueryParams TraceParams;

	FVector CurrentVelocity;

	/// Used to compute the velocity of the radar
	FVector PrevLocation;

	struct RayData {
		FVector hitLocation;
		float Radius;
		float Angle;
		float RelativeVelocity;
		FVector2D AzimuthAndElevation;
		float Distance;
		bool didHit = false;
	};

	TArray<RayData> Rays;

private:

	void CalculateCurrentVelocity(const float DeltaTime);

	void SendLineTraces(float DeltaTime);

	float CalculateRelativeVelocity(const FHitResult& OutHit, const FVector& RadarLocation);


protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

};
