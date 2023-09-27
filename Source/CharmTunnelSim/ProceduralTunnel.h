// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Components/SplineComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Containers/Array.h"
#include "Curves/CurveFloat.h"
#include "Engine/StaticMeshActor.h"
#include "ProceduralMeshComponent.h"
#include "Engine/TextureRenderTarget2D.h"
#include "EnumContainer.h"
#include "ProceduralTunnel.generated.h"

class AProceduralIntersection;

UCLASS(Blueprintable)
class CHARMTUNNELSIM_API AProceduralTunnel : public AActor
{
	GENERATED_BODY()
	
	
public:	
	// Sets default values for this actor's properties
	AProceduralTunnel();

	// Spline component
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spline", meta=(AllowPrivateAccess = "true"))
	USplineComponent* SplineComponent;

	// Static mesh spline point indicator
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spline", meta=(AllowPrivateAccess = "true"))
	UStaticMeshComponent* SplinePointIndicator;

	// Scene root component
	USceneComponent* RootComponent;

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	//DEFAULT VALUES
	FVector wallStartVertice;
	float maxDistanceToSnapSplineEnds = 600.0f;
	float stepSizeOnSpline = 50.0f; ///100 original lower the number the higher the resolution
	float lastStepSizeOnSpline = 0.0f;
	TArray<AStaticMeshActor*> staticMeshes;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	bool isFirstTunnel = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	bool isReset;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	bool isUndo;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	bool isLoad;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	float isSelected;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Deformation")
	float floorDeformation;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Deformation")
	float wallDeformation;
	float noiseTextureXresolution = 500.0f;
	UFUNCTION(BlueprintImplementableEvent, Category = "Deformation")
	float GetPixelValue(int32 x, int32 y);
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	UCurveFloat* stopDeformCurve;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	float widthScale;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	float heightScale;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FVector2D localTunnelScale;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FVector2D surfaceVariation;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vertice points", meta = (ExposeOnSpawn = "true"))
	int32 numberOfHorizontalPoints;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vertice points", meta = (ExposeOnSpawn = "true"))
	int32 numberOfVerticalPoints;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Vertice points", meta = (ExposeOnSpawn = "true"))
	int32 loopAroundTunnelLastIndex;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tunnel foundation")
	TArray<FVector> wallVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tunnel foundation")
	TArray<int32> wallTriangles;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tunnel foundation")
	TArray<FVector2D> wallUV;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tunnel foundation")
	TArray<FVector> wallNormals;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tunnel foundation")
	TArray<FProcMeshTangent> wallTangents;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tunnel foundation")
	TArray<FVector> groundVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tunnel foundation")
	TArray<int32> groundTriangles;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tunnel foundation")
	TArray<FVector2D> groundUV;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tunnel foundation")
	TArray<FVector> groundNormals;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Tunnel foundation")
	TArray<FProcMeshTangent> groundTangents;
	float maxDeform = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Snapping")
	bool isEndConnected;

	AProceduralTunnel* connectedActor;
	AProceduralIntersection* parentIntersection;
	AProceduralTunnel* parentsParentTunnel;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Snapping")
	AProceduralTunnel* rightSideTunnel;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Snapping")
	AProceduralTunnel* leftSideTunnel;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Meshes")
	TArray<UProceduralMeshComponent*> TunnelMeshes;

	// FIST VERTICE DATA OF THE WHOLE TUNNEL SECTION
	TArray<FVector> firstFloorVertices;
	TArray<FVector> firstRightVertices;
	TArray<FVector> firstRoofVertices;
	TArray<FVector> firstLeftVertices;

	// LAST VERTICE DATA OF THE MESH SECTION
	TArray<FVector> lastFloorVertices;
	TArray<FVector> lastRightVertices;
	TArray<FVector> lastRoofVertices;
	TArray<FVector> lastLeftVertices;
	TArray<FVector> allFloorVertices;

	// LAST VERTICE DATA OF THE WHOLE TUNNEL SECTION
	TArray<FVector> tunnelLastFloorVertices;
	TArray<FVector> tunnelLastRightVertices;
	TArray<FVector> tunnelLastRoofVertices;
	TArray<FVector> tunnelLastLeftVertices;

	// LOOP VARIABLES
	int32 meshLoopFirstIndex;
	int32 indexOfLastMesh;
	int32 indexOfCurrentMesh;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 stepCountToMakeCurrentMesh;
	int32 stepIndexInsideMesh;
	int32 loopAroundTunnelCurrentIndex;

	int32 surfaceIndex;
	int32 verticeIndex;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	float horizontalPointSize;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	float verticalPointSize;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FMeshSectionEnd currentMeshEndData;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FMeshSectionEnd> meshEnds;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FMeshSectionEnd> meshEndsBeforeIntersection;
	int32 forwardStepInDeformTexture;
	FVector firstVertice;
	FVector latestVertice;
	FVector rightVector;
	FVector forwardVector;
	FVector startLocationOnSpline;
	TArray<float> deformValues;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	UCurveFloat* deformCurve;

	
	float tunnelRoundValue = 100.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 reCreateMeshCount;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 meshInRework = -1;
	

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TEnumAsByte<TunnelType> tunnelType = TunnelType::DefaultTunnel;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TEnumAsByte<IntersectionType> intersectionType = IntersectionType::Right;


	virtual void Tick(float DeltaTime) override;

	UFUNCTION(BlueprintCallable)
	void SnapToEndOfOtherSpline();
	UFUNCTION(BlueprintCallable)
	void ControlSplinePoints(bool bInterSectionAdded = false);
	UFUNCTION(BlueprintCallable)
	int32 SetUpProceduralGeneratorLoopParams();
	UFUNCTION(BlueprintCallable)
	void ProceduralGenerationLoop(int32 firstIndex, int32 lastIndex, bool isSinglePointUpdate, bool isIntersectionAdded, IntersectionType interType);
	UFUNCTION(BlueprintCallable)
	void SetValuesForGeneratingTunnel(float select, bool undo, FVector2D tunnelScale, FVector2D sVariation, bool reset, bool load);

	void AddOrRemoveSplinePoints(bool interSectionAdded);

	void InitializeProceduralGenerationLoopVariables(int32 firstIndex, int32 lastIndex, IntersectionType interType);
	void CalculateStepsInTunnelSection();
	void ResetCurrentMeshEndData();
	void GenerateVerticesAndUVs(bool isSinglePointUpdate, bool isIntersectionAdded, int32 lastIndex);
	void UsePreviousEndVerticesData(bool isSinglePointUpdate);
	void GenerateVerticesForCurrentLoop(bool isSinglePointUpdate, bool isIntersectionAdded, int32 lastIndex);

	int32 GetSurfaceIndex();
	int32 GetIndexOfVertice();

	bool GetIsFirstLoopAround();
	bool usePreviousEndVertices (bool isIntersectionAdded, bool isSinglePointUpdate);

	// Functions used to get intersections child tunnels start vertices to align with intersection
	FVector RightTunnelStart();
	FVector LeftTunnelStart();
	FVector StraightTunnelStart();
	FVector TransformVerticeToLocalSpace(AActor* actorFrom, FVector vector);

	// Basic functions to get vertice locations on different surfaces
	FVector GetVerticeOnGround();
	FVector GetVerticeOnRightWall(bool isFirstLoopARound, bool isIntersectionAdded);
	FVector GetVerticeOnRoof();
	FVector GetVerticeOnLeftWall(bool isFirstLoopARound, bool isIntersectionAdded);

	void InitializeStartVectorRightVectorAndValueInTexture();
	void ClearArrays();

	FVector GetVerticeForConnectedTunnel();
	FVector GetVerticeForStartOfChildTunnel();
	bool IsOnTheEndOfTunnel();
	FVector GetVerticeForDefaultTunnel(bool isFirstLoopAround, bool isIntersectionAdded);
	FVector AdjustLatestVerticeForOverlap(FVector latestVertice, int32 surfaceIndex);
	void SaveFirstVerticeIfNeeded(FVector latestVertice);
	void AddCreatedVerticeToArrays(FVector latestVertice, int32 surfaceIndex);
	bool IsFirstLoopOfWholeTunnel();
	void SaveFirstLoopVerticeData(int32 surfaceIndex, FVector latestVertice);
	bool IsOnTheEndOfCurrentMeshSection();
	void SaveEndMeshVerticeData(int32 surfaceIndex, FVector latestVertice);

	UFUNCTION(BlueprintCallable)
	void DestroyLastMesh();

	// Tunnel foundation functions
	UFUNCTION(BlueprintImplementableEvent)
	void MakeMeshTriangles();
	UFUNCTION(BlueprintImplementableEvent)
	void MakeMeshTangentsAndNormals();
	UFUNCTION(BlueprintImplementableEvent)
	void MakeMesh(int32 meshIndex, bool isHeightChanged, bool isFromLoad);
};
