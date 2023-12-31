
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
	float verticalPointSize;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	float horizontalPointSize;

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
	TArray<FVector> wallVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector> groundVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector2D> wallUV;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FVector2D> groundUV;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FVector latestVertice;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 surfaceIndex;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 forwarLoopIndex;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 loopAroundTunnelCurrentIndex;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FVector firstVertice;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vertice points")
	int32 numberOfHorizontalPoints;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vertice points")
	int32 numberOfVerticalPoints;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vertice points")
	int32 loopAroundTunnelLastIndex;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	UCurveFloat* roundnessCurve; 
	float tunnelRoundValue = 100.0f; // HOW MUCH WE ADD ROUNDNESS TO TUNNEL
	UFUNCTION(BlueprintImplementableEvent, Category = "Deformation")
	float GetPixelValue(int32 x, int32 y);

	// FUNCTIONS
	UFUNCTION(BlueprintCallable)
	void ClearArrays();
	UFUNCTION(BlueprintCallable)
	void SetValues(FVector2D scale, IntersectionType type, AProceduralTunnel* parent, FVector2D variation, bool update);
	UFUNCTION(BlueprintCallable)
	void IntersectionGenerationLoop();
	void StoreVertice();
	int32 GetArrayIndex();
	int32 GetSurfaceIndex();
	FVector GetVertice();
	FVector TransformVertex(FVector vertex);
	FVector GetVerticeBySurface(int32 surface, IntersectionType type);
	FVector GetFloorVertice();
	FVector GetRightVertice();
	FVector GetLeftVertice();
	FVector GetRoofVertice();

	UFUNCTION(BlueprintImplementableEvent)
	void MakeMeshTriangles();
	UFUNCTION(BlueprintImplementableEvent)
	void MakeMeshTangentsAndNormals();
	UFUNCTION(BlueprintImplementableEvent)
	void MakeMesh();
	UFUNCTION(BlueprintImplementableEvent)
	void AddContinuationTunnels();
	
	float maxWallDeformation = 100.0f;
	float maxFloorDeformation = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	float roundingAmount = 15.0f;
};
