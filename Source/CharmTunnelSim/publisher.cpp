// Fill out your copyright notice in the Description page of Project Settings.

#include "publisher.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Public/std_msgs/String.h"


// Sets default values
Apublisher::Apublisher()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void Apublisher::BeginPlay()
{
	Super::BeginPlay();
	
}

void Apublisher::test() {
	// Initialize a topic
	UTopic* ExampleTopic = NewObject<UTopic>(UTopic::StaticClass());
	UROSIntegrationGameInstance* rosinst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	ExampleTopic->Init(rosinst->ROSIntegrationCore, TEXT("/test"), TEXT("std_msgs/String"));

	TSharedPtr<ROSMessages::std_msgs::String> StringMessage(new ROSMessages::std_msgs::String("This is an example"));
	ExampleTopic->Publish(StringMessage);
	UE_LOG(LogTemp, Log, TEXT("Incoming string was: asdasda"));
}

// Called every frame
void Apublisher::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

