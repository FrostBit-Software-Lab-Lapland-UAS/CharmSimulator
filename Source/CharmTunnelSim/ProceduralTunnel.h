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
	float maxDistance = 150.0f;
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
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Deformation")
	UTextureRenderTarget2D* wallDeformRenderTarget;
	float noiseTextureXresolution = 500.0f;
	UFUNCTION(BlueprintImplementableEvent, Category = "Deformation")
	float GetPixelValue(int32 x, int32 y);
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	UCurveFloat* stopDeformCurve;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	float widthScale;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	float heightScale;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	FVector2D localTunnelScale;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	FVector2D surfaceVariation;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	int32 pointsInWidth = 25;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	int32 pointsInHeight = 20;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	int32 splineLoopAdjustment = 2;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	TArray<FVector> wallVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	TArray<int32> wallTriangles;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	TArray<FVector2D> wallUV;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	TArray<FVector> wallNormals;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	TArray<FProcMeshTangent> wallTangents;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	TArray<FVector> groundVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	TArray<int32> groundTriangles;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	TArray<FVector2D> groundUV;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	TArray<FVector> groundNormals;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	TArray<FProcMeshTangent> groundTangents;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "DEFAULT VALUES")
	float maxDeform = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Snapping")
	bool isEndConnected;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Snapping")
	AProceduralTunnel* connectedActor;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Snapping")
	AProceduralIntersection* parentIntersection;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Snapping")
	AProceduralTunnel* parentsParentTunnel;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Snapping")
	AProceduralTunnel* rightSideTunnel;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Snapping")
	AProceduralTunnel* leftSideTunnel;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Meshes")
	TArray<UProceduralMeshComponent*> TunnelMeshes;

	// FIST VERTICE DATA
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FirstVertices")
	TArray<FVector> firstFloorVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FirstVertices")
	TArray<FVector> firstRightVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FirstVertices")
	TArray<FVector> firstRoofVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "FirstVertices")
	TArray<FVector> firstLeftVertices;

	// LAST VERTICE DATA
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LastVertices")
	TArray<FVector> lastFloorVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LastVertices")
	TArray<FVector> lastRightVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LastVertices")
	TArray<FVector> lastRoofVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LastVertices")
	TArray<FVector> lastLeftVertices;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "LastVertices")
	TArray<FVector> allFloorVertices;

	// LOOP VARIABLES
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 meshLoopFirstIndex;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 meshLoopLastIndex;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 meshLoopCurrentIndex;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 pointCapLoopLastIndex;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 pointCapLoopCurrentIndex;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 loopAroundTunnelLastIndex = 92;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 loopAroundTunnelCurrentIndex;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 surfaceIndex;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 arrayIndex;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	float groundVerticeSize;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	float wallVerticeSize;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FMeshSectionEnd currentMeshEndData;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FMeshSectionEnd> meshEnds;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<FMeshSectionEnd> meshEndsBeforeIntersection;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 forwardStepInDeformTexture;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FVector firstVertice;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FVector latestVertice;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FVector rightVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FVector forwardVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	FVector startLocationOnSpline;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TArray<float> deformValues;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	UCurveFloat* deformCurve;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	UCurveFloat* rotateCurve;

	
	float tunnelRoundValue = 100.0f;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 reCreateMeshCount;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 meshInRework = -1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	int32 countOfMeshesToRemake;
	

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TEnumAsByte<TunnelType> tunnelType = TunnelType::DefaultTunnel;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Procedural loop params")
	TEnumAsByte<IntersectionType> intersectionType = IntersectionType::Right;

	/////////////test stuff
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "TEST", meta = (AllowPrivateAccess = "true"))
	UStaticMesh* cube;


	// Called every frame
	virtual void Tick(float DeltaTime) override;
	// Get closest tunnel end
	UFUNCTION(BlueprintCallable)
	void SnapToEndOfOtherSpline();
	UFUNCTION(BlueprintCallable)
	void ControlSplinePoints(bool bInterSectionAdded = false);
	UFUNCTION(BlueprintCallable)
	int32 SetUpProceduralGeneratorLoopParams();
	UFUNCTION(BlueprintCallable)
	void ProceduralGenerationLoop(int32 firstIndex, int32 lastIndex, bool isSinglePointUpdate, bool isIntersectionAdded, IntersectionType interType);
	UFUNCTION(BlueprintCallable)
	void SetValues(float select, bool undo, FVector2D tunnelScale, FVector2D sVariation, bool reset, bool load);

	void AddOrRemoveSplinePoints(bool interSectionAdded);

	void InitializeProceduralGenerationLoopVariables(int32 firstIndex, int32 lastIndex, IntersectionType interType);
	void SetupSplineLocations();
	void ResetCurrentMeshEndData();
	void GenerateVerticesAndUVs(bool isSinglePointUpdate, bool isIntersectionAdded, int32 lastIndex);
	void UsePreviousEndVerticesData(bool isSinglePointUpdate);
	void GenerateVerticesForCurrentLoop(bool isSinglePointUpdate, bool isIntersectionAdded, int32 lastIndex);

	int32 GetSurfaceIndex();
	int32 CalculateArrayIndex();

	bool GetIsFirstLoopAround();
	bool usePreviousEndVertices (bool isIntersectionAdded, bool isSinglePointUpdate);


	FVector RightTunnelStart();
	FVector LeftTunnelStart();
	FVector StraightTunnelStart();

	FVector GetFloorVertice();
	FVector GetRightVertice(bool isFirstLoopARound, bool isIntersectionAdded);
	FVector GetRoofVertice();
	FVector GetLeftVertice(bool isFirstLoopARound, bool isIntersectionAdded);

	void InitializeStartVectorRightVectorAndValueInTexture();
	void ClearArrays();

	FVector GetVerticeForConnectedTunnel();
	FVector GetVerticeForIntersectionAddedTunnel();
	FVector GetLatestVerticeForIntersectionContinuationTunnels();
	bool IsConnectedToOtherTunnel(bool isSinglePointUpdate);
	bool IsIntersectionAddedToThisTunnel(bool isIntersectionAdded, int32 lastIndex);
	FVector GetVerticeForDefaultTunnel(bool isFirstLoopAround, bool isIntersectionAdded);
	FVector AdjustLatestVerticeForOverlap(FVector latestVertice, int32 surfaceIndex);
	void SaveFirstVerticeIfNeeded(FVector latestVertice);
	void AddCreatedVerticeToArrays(FVector latestVertice, int32 surfaceIndex);
	bool IsFirstLoopOfWholeTunnel();
	void SaveFirstLoopVerticeData(int32 surfaceIndex, FVector latestVertice);
	bool IsEndOfCurrentMesh();
	void SaveEndMeshVerticeData(int32 surfaceIndex, FVector latestVertice);
	void AddUVCoordinates(int32 surfaceIndex, int32 pointCapLoopCurrentIndex, int32 pointCapLoopLastIndex, int32 loopAroundTunnelCurrentIndex);

	UFUNCTION(BlueprintCallable)
	void DestroyLastMesh();

	UFUNCTION(BlueprintImplementableEvent)
	void MakeMeshTriangles();
	UFUNCTION(BlueprintImplementableEvent)
	void MakeMeshTangentsAndNormals();
	UFUNCTION(BlueprintImplementableEvent)
	void MakeMesh(int32 meshIndex, bool isHeightChanged, bool isFromLoad);
};
