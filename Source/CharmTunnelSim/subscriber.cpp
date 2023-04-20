// Fill out your copyright notice in the Description page of Project Settings.

#include "subscriber.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/std_msgs/String.h"


// Sets default values
Asubscriber::Asubscriber()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void Asubscriber::BeginPlay()
{
	Super::BeginPlay();
	// Initialize a topic
	UTopic* ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	ExampleTopic->Init(rosinst->ROSIntegrationCore, TEXT("/test"), TEXT("std_msgs/String"));

	// Create a std::function callback object
	std::function<void(TSharedPtr<FROSBaseMsg>)> SubscribeCallback = [](TSharedPtr<FROSBaseMsg> msg) -> void
	{
		auto Concrete = StaticCastSharedPtr<ROSMessages::std_msgs::String>(msg);
		if (Concrete.IsValid())
		{
			UE_LOG(LogROS, Display, TEXT("Incoming string was: %s"), (*(Concrete->_Data)));
		}
		return;
	};
	// (Optional) Advertise the topic
	ExampleTopic->Advertise();

	// Subscribe to the topic
	ExampleTopic->Subscribe(SubscribeCallback);
}


// Called every frame
void Asubscriber::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

