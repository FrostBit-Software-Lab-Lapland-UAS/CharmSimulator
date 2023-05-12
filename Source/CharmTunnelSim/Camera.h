// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "EnumContainer.h"
#include "Engine/Classes/Camera/CameraComponent.h"
#include "Engine/Classes/Components/SceneCaptureComponent2D.h"
#include "Engine/Classes/Engine/TextureRenderTarget2D.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/sensor_msgs/Image.h"
#include "Async/Async.h"
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

	void CaptureAndPublishImage();

	void ProcessAndPublishImage();

	FRenderCommandFence RenderFence;

	UPROPERTY()
	TArray<FColor> ReadBufferData;

	void ExecuteOnRenderThread(FRHICommandListImmediate& RHICmdList, FRHITexture2D* Texture, FIntRect Rect, TArray<FColor> BufferData);
	void CaptureRenderTarget(FRHICommandListImmediate& RHICmdList, FRHITexture2D* RenderTargetTexture, FIntRect Rect, TArray<FColor>& ReadBuffer);


	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Parameters", meta = (ExposeOnSpawn = "true"))
	FCameraDescription Description;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "Parameters", meta = (ExposeOnSpawn = "true"))
	int32 sensorIndex;

	// Textures need to be power of 2
    // Texture has to be a square
	uint32_t internResolution;
	float deltaCount;
	UPROPERTY()
	UTopic* CameraDataTopic;

	TSharedPtr<ROSMessages::sensor_msgs::Image> output_image = MakeShareable(new ROSMessages::sensor_msgs::Image);

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	int32 NumPixels;

	UPROPERTY(BlueprintReadWrite, EditAnywhere, Category = "ROSS")
	UROSIntegrationGameInstance* rosInstance;

	UPROPERTY(EditAnywhere, Category = "Output Information", meta = (ClampMin = "32", ClampMax = "4096", UIMin = "32", UIMax = "4096"))
	uint32 resolutionX;

	UPROPERTY(EditAnywhere, Category = "Output Information", meta = (ClampMin = "32", ClampMax = "4096", UIMin = "32", UIMax = "4096"))
	uint32 resolutionY;

	UPROPERTY(EditAnywhere, Category = "Output Information", meta = (ClampMin = "20.0", ClampMax = "170.0", UIMin = "20.0", UIMax = "179.9"))
	float field_of_view;

	UPROPERTY(EditAnywhere)
	class UCameraComponent* ourCamera;

	UPROPERTY(EditAnywhere)
	class USceneCaptureComponent2D* sceneCapture;

	UPROPERTY(BlueprintReadWrite, EditAnywhere)
	UTextureRenderTarget2D* renderTarget;
};
