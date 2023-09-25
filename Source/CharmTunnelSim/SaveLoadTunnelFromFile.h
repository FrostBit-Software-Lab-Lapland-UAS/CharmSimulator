// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "SaveLoadTunnelFromFile.generated.h"

/**
 * 
 */
UCLASS()
class CHARMTUNNELSIM_API USaveLoadTunnelFromFile : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()


public:

	UFUNCTION(BlueprintCallable, Category = "LoadFromFile")
		static FString LoadFromFile(FString FilePath, bool& bOutSuccess, FString& OutInfoMessage);

	UFUNCTION(BlueprintCallable, Category = "SaveToFile")
		static void SaveToFile(FString FilePath, FString String, bool& bOutSuccess, FString& OutInfoMessage);

	UFUNCTION(BlueprintCallable, BlueprintPure, Category = "VerticeCalculations")
		static bool IsPointInsideRectangle2D(const FVector2D& P, const FVector2D& A, const FVector2D& B, const FVector2D& C, const FVector2D& D);
	
};
