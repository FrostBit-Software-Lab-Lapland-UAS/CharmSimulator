// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "EnumContainer.h"
#include "ProceduralIntersection.generated.h"

class AProceduralTunnel;

UCLASS()
class CHARMTUNNELSIM_API AProceduralIntersection : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AProceduralIntersection();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	float wallVerticeSize;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector> lastRightWallVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector> lastLeftWallVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector> lastStraightRoofVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector> lastRightRoofVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector> lastLeftRoofVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector> lastRightFloorVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector> lastStraightFloorVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector> lastLeftFloorVertices;


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TEnumAsByte<IntersectionType> intersectionType = IntersectionType::Right;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Meshes")
	UProceduralMeshComponent* IntersectionMesh;

	// PARAMS
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FVector2D localScale;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FVector2D surfaceVariation;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	AProceduralTunnel* parentTunnel;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	bool isUpdate;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	float groundVerticeSize;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 pointsInWidth = 25;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 pointsInHeight = 20;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 pointAdjustment = 2;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector> wallVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector> groundVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector2D> wallUV;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector2D> groundUV;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 loopAroundTunnelLastIndex;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FVector latestVertice;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 surfaceIndex;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 forwarLoopIndex;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 aroundLoopIndex;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FVector firstVertice;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	UCurveFloat* roundnessCurve; 
	float tunnelRoundValue = 100.0f; // HOW MUCH WE ADD ROUNDNESS TO TUNNEL
	UFUNCTION(BlueprintImplementableEvent, Category = "Deformation")
	float GetPixelValue(int32 x, int32 y);


	/////////////test stuff
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TEST", meta = (AllowPrivateAccess = "true"))
	UStaticMesh* cube;


	// FUNCTIONS
	UFUNCTION(BlueprintCallable)
	void ClearArrays();
	UFUNCTION(BlueprintCallable)
	void SetValues(FVector2D scale, IntersectionType type, AProceduralTunnel* parent, FVector2D variation, bool update);
	UFUNCTION(BlueprintCallable)
	void RightSideIntersection();
	UFUNCTION(BlueprintCallable)
	void LeftSideIntersection();
	UFUNCTION(BlueprintCallable)
	void RightLeftIntersection();
	UFUNCTION(BlueprintCallable)
	void AllSidesIntersection();
	void GenerateMeshes();
	void CalculateTangentsAndNormals();
	void CalculateTriangles();
	void AddChildTunnels();
	void GetSurfaceIndex();
	FVector GetVertice();
	FVector GetFloorVertice();
	FVector GetRightVertice();
	FVector GetLeftVertice();
	FVector GetRoofVertice();
	FVector GetRightLeftIntersectionRoof(); // This is so special that it can be implemented into above

	UFUNCTION(BlueprintImplementableEvent)
	void MakeMeshTriangles();
	UFUNCTION(BlueprintImplementableEvent)
	void MakeMeshTangentsAndNormals();
	UFUNCTION(BlueprintImplementableEvent)
	void MakeMesh();
	UFUNCTION(BlueprintImplementableEvent)
	void AddContinuationTunnels();
	


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	float maxDeform = 100.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	float roundingAmount = 15.0f;
};
