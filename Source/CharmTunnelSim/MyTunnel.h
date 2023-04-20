// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "Components/SplineComponent.h"
#include "MyTunnel.generated.h"


UCLASS()
class CHARMTUNNELSIM_API AMyTunnel : public AActor
{
	GENERATED_BODY()

public:
	AMyTunnel();

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
	UProceduralMeshComponent* ProceduralMesh;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Components")
	USplineComponent* Spline;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Tunnel")
	float Width = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Tunnel")
	float Height = 300.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Tunnel")
	int32 Subsections = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Tunnel")
	UMaterialInterface* TunnelMaterial;

	UFUNCTION(BlueprintCallable)
	void AddIntersection();

	void UpdateTunnel(float TunnelWidth, float TunnelHeight, float SurfaceVariation, UTexture2D* SurfaceTexture, int32 Resolution, int32 SubsectionCount);
	void GenerateMeshSection(int32 MeshSectionIndex, int32 SplineIndex, float TunnelWidth, float TunnelHeight, float SurfaceVariation, UTexture2D* SurfaceTexture, int32 Resolution, float StartAlpha, float EndAlpha);
	void GenerateSectionVertices(int32 SplineIndex, float StartAlpha, float EndAlpha, float TunnelWidth, float TunnelHeight, int32 Resolution, TArray<FVector>& Vertices, TArray<FVector>& Normals, TArray<FVector2D>& UVs, TArray<FProcMeshTangent>& Tangents);
	AMyTunnel* CreateIntersection(float BranchingAngle);

private:
	int32 SectionCount;
};


