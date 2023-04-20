// Fill out your copyright notice in the Description page of Project Settings.


#include "Camera.h"
#include "Math/UnrealMathUtility.h"
#include "RHI.h"
#include "RenderingThread.h"
#include "RHIResources.h"



// Sets default values
ACamera::ACamera()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
    PrimaryActorTick.TickGroup = TG_PostUpdateWork;

    RootComponent = CreateDefaultSubobject<USceneComponent>(TEXT("RootComponent"));
    ourCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("ViewportCamera"));
    ourCamera->SetupAttachment(RootComponent);
    sceneCapture = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("SceneCapture"));   // *****
    sceneCapture->SetupAttachment(ourCamera);
}

// Called when the game starts or when spawned
void ACamera::BeginPlay()
{
	Super::BeginPlay();

    rosInstance = Cast<UROSIntegrationGameInstance>(GetGameInstance());
    // IF ROS CONNECTION IS ALREADY MADE WE CREATE ROS TOPIC IN THE START
    if (rosInstance->bIsConnected) {
        // Initialize a topic
        ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
        ExampleTopic->Init(rosInstance->ROSIntegrationCore, Description.topicName, TEXT("sensor_msgs/Image"), 0);

        // (Optional) Advertise the topic
        ExampleTopic->Advertise();
    }

    ourCamera->FieldOfView = Description.field_of_view;
    sceneCapture->FOVAngle = Description.field_of_view;
    renderTarget = NewObject<UTextureRenderTarget2D>();
    renderTarget->InitAutoFormat(Description.resolutionX, Description.resolutionY);
    renderTarget->bGPUSharedFlag = true;
    //renderTarget->InitCustomFormat(internResolution, internResolution, EPixelFormat::PF_B8G8R8A8, true);  // some testing with EPixelFormat::PF_FloatRGBA, true=force Linear Gamma
    renderTarget->UpdateResourceImmediate();

    sceneCapture->CaptureSource = SCS_FinalColorLDR;     // SCS_FinalColorLDR allows for post processing on the image.  default = SCS_SceneColorHDR
    sceneCapture->TextureTarget = renderTarget;
    sceneCapture->bCaptureEveryFrame = true;
    sceneCapture->bAlwaysPersistRenderingState = true;  // This allows velocities for Motion Blur and Temporal AA to be computed.

    output_image->header.seq = 1;
    output_image->header.frame_id = "Camera";
    output_image->height = Description.resolutionY;
    output_image->width = Description.resolutionX;
    output_image->encoding = "rgb8";
    output_image->is_bigendian = false;
    output_image->step = 3 * Description.resolutionX; //Full row length in bytes

    deltaCount = 0;
}



constexpr float FRAME_RATE = 24.0f;
constexpr float FRAME_INTERVAL = 1.0f / FRAME_RATE;

// Called every frame
void ACamera::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

    // Check if enough time has passed since the last frame
    deltaCount += DeltaTime;
    if (deltaCount < FRAME_INTERVAL) {
        return;
    }
    else {
        deltaCount = 0;
    }

    // IF ROS INSTANCE IS CONNECTED AND WE DONT HAVE TOPIC MADE YET
    if (rosInstance->bIsConnected && !IsValid(ExampleTopic)) {
        // Initialize a topic
        ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
        ExampleTopic->Init(rosInstance->ROSIntegrationCore, Description.topicName, TEXT("sensor_msgs/Image"), 0);

        // (Optional) Advertise the topic
        ExampleTopic->Advertise();
    }

    CaptureAndPublishImage();
}


void ACamera::CaptureAndPublishImage()
{
    // Check if the render target is valid
    if (!renderTarget)
    {
        UE_LOG(LogTemp, Error, TEXT("Render target is not set."));
        return;
    }

    // Create a lambda function to capture the render target
    auto CaptureLambda = [this, RT = renderTarget, &ReadBuffer = ReadBufferData](FRHICommandListImmediate& RHICmdList)
    {
        // Get the render target texture
        FRHITexture2D* SrcRenderTarget = RT->GetRenderTargetResource()->GetRenderTargetTexture();
        if (!SrcRenderTarget)
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to create a snapshot of the render target."));
            return;
        }
        // Define the rectangle to capture the entire render target
        FIntRect Rect(0, 0, SrcRenderTarget->GetSizeX(), SrcRenderTarget->GetSizeY());
        // Capture the render target
        CaptureRenderTarget(RHICmdList, SrcRenderTarget, Rect, ReadBuffer);
    };

    // Enqueue the render command to capture the render target on the render thread
    ENQUEUE_RENDER_COMMAND(CaptureCommand)(
        [CaptureLambda](FRHICommandListImmediate& RHICmdList)
        {
            CaptureLambda(RHICmdList);
        });

    // Use AsyncTask to process the image and send it to ROS API on a separate thread.
    AsyncTask(ENamedThreads::AnyThread, [this]()
        {
            // Process the captured image data (ReadBufferData) and prepare the output_image object

            // Set the current timestamp for the output_image header
            output_image->header.time = FROSTime::Now();

            // Create a new array to store the RGB values of each pixel
            uint8* uArray = new uint8[ReadBufferData.Num() * 3];

            // Iterate through the captured image data and extract RGB values
            for (int i = 0; i < ReadBufferData.Num(); i++)
            {
                uArray[i + i * 2] = ReadBufferData[i].R;
                uArray[i + 1 + i * 2] = ReadBufferData[i].G;
                uArray[i + 2 + i * 2] = ReadBufferData[i].B;
            }
            // Set the output_image data to the processed RGB array
            output_image->data = uArray;

            // Send the processed image data to the ROS API

            // Check if ROS is connected and the ExampleTopic is valid
            if (rosInstance->bIsConnected && IsValid(ExampleTopic) && (ReadBufferData.Num() * 3) > 0) {
                // Publish the output_image to the ROS topic
                bool didPub = ExampleTopic->Publish(output_image);
            }

            // Cleanup: delete the allocated RGB array
            delete[] uArray;
        });
}

void ACamera::CaptureRenderTarget(FRHICommandListImmediate& RHICmdList, FRHITexture2D* RenderTargetTexture, FIntRect Rect, TArray<FColor>& ReadBuffer)
{
    check(IsInRenderingThread());

    if (!RenderTargetTexture)
    {
        UE_LOG(LogTemp, Error, TEXT("RenderTargetTexture is null."));
        return;
    }

    FReadSurfaceDataFlags ReadPixelFlags(RCM_UNorm);
    ReadPixelFlags.SetLinearToGamma(true);

    RHICmdList.ReadSurfaceData(RenderTargetTexture, Rect, ReadBuffer, ReadPixelFlags);
}
