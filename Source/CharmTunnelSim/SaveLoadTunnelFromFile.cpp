// Fill out your copyright notice in the Description page of Project Settings.


#include "SaveLoadTunnelFromFile.h"
#include "HAL/PlatformFileManager.h"
#include "Misc/FileHelper.h"

FString USaveLoadTunnelFromFile::LoadFromFile(FString FilePath, bool& bOutSuccess, FString& OutInfoMessage)
{
	if (!FPlatformFileManager::Get().GetPlatformFile().FileExists(*FilePath))
	{
		bOutSuccess = false;
		OutInfoMessage = FString::Printf(TEXT("Read String from file failed - File doesnt exist - '%s'"), *FilePath);
		return "";
	}
	FString RetString = "";
	if (!FFileHelper::LoadFileToString(RetString, *FilePath))
	{
		bOutSuccess = false;
		OutInfoMessage = FString::Printf(TEXT("Read String from file failed - Was not able to read file. Is this a text file? - '%s'"), *FilePath);
		return "";
	}
	bOutSuccess = true;
	OutInfoMessage = FString::Printf(TEXT("Read String from file succeeded - '%s'"), *FilePath);
	return RetString;
}

void USaveLoadTunnelFromFile::SaveToFile(FString FilePath, FString String, bool& bOutSuccess, FString& OutInfoMessage)
{
	if (!FFileHelper::SaveStringToFile(String, *FilePath))
	{
		bOutSuccess = false;
		OutInfoMessage = FString::Printf(TEXT("Write data failed. Is path valid? - '%s'"), *FilePath);
		return;
	}

	bOutSuccess = true;
	OutInfoMessage = FString::Printf(TEXT("Write data succeeded - '%s'"), *FilePath);
	return;
}

// Function to compute the cross product of two 2D vectors
float cross(const FVector2D& u, const FVector2D& v) {
	return u.X * v.Y- u.Y * v.X;
}

bool USaveLoadTunnelFromFile::IsPointInsideRectangle2D(const FVector2D& P, const FVector2D& A, const FVector2D& B, const FVector2D& C, const FVector2D& D)
{
	return (cross(B - A, P - A) >= 0) &&
		(cross(D - B, P - B) >= 0) &&
		(cross(C - D, P - D) >= 0) &&
		(cross(A - C, P - C) >= 0);
}
