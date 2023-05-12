// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Engine/Classes/Camera/CameraComponent.h"
#include "Engine/Classes/Components/SceneCaptureComponent2D.h"
#include "Engine/Classes/Engine/TextureRenderTarget2D.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/sensor_msgs/Image.h"
#include "Async/Async.h"
#include "EnumContainer.h"
#include "Camera.generated.h"

UCLASS()
class CHARMTUNNELSIM_API ACamera : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ACamera();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Functions to capture and publish image
	void CaptureAndPublishImage();
	void ProcessAndPublishImage();

	// Function to capture render target
	void CaptureRenderTarget(FRHICommandListImmediate& RHICmdList, FRHITexture2D* RenderTargetTexture, FIntRect Rect, TArray<FColor>& ReadBuffer);

	// Function to initialize ROS topic
	void InitRosTopic();

	float deltaCount;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Parameters", meta = (ExposeOnSpawn = "true"))
	int32 sensorIndex;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Parameters", meta = (ExposeOnSpawn = "true"))
	FCameraDescription Description;

	// Render Command Fence and BufferData
	FRenderCommandFence RenderFence;
	UPROPERTY()
	TArray<FColor> ReadBufferData;

	// ROS topic for the camera data
	UPROPERTY()
	UTopic* CameraDataTopic;

	// Output image from ROS messages
	TSharedPtr<ROSMessages::sensor_msgs::Image> output_image = MakeShareable(new ROSMessages::sensor_msgs::Image);

public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	// ROS Instance
	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ROSS")
	UROSIntegrationGameInstance* rosInstance;

	// Camera and capture component
	UPROPERTY(EditAnywhere)
	class UCameraComponent* ourCamera;
	UPROPERTY(EditAnywhere)
	class USceneCaptureComponent2D* sceneCapture;

	// Render target
	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	UTextureRenderTarget2D* renderTarget;
};
