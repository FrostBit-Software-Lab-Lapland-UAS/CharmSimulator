// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.


#include "ProceduralTunnel.h"
#include "ProceduralIntersection.h"
#include "Components/SplineComponent.h"
#include "Math/Vector.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"
#include "Math/UnrealMathUtility.h"
#include "Components/StaticMeshComponent.h"
#include "Math/UnrealMathVectorCommon.h"
using namespace std;

// Sets default values
AProceduralTunnel::AProceduralTunnel()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	RootComponent = CreateDefaultSubobject<USceneComponent>("Root Scene Component");
	RootComponent->SetMobility(EComponentMobility::Static);
	SetRootComponent(RootComponent);
	SplineComponent = CreateDefaultSubobject<USplineComponent>("Spline Component");
	SplineComponent->SetupAttachment(RootComponent);
	SplinePointIndicator = CreateDefaultSubobject<UStaticMeshComponent>("Mesh Component");
	SplinePointIndicator->SetupAttachment(SplineComponent);

	int32 PointIndex = 1; // The index of the second spline point

	// Set the location for the second spline point
	FVector NewLocation(300.f, 0.f, 0.f); // Replace this with the desired location
	SplineComponent->SetLocationAtSplinePoint(PointIndex, NewLocation, ESplineCoordinateSpace::Local);

	// Set the tangent for the second spline point 
	FVector SecondPointTangent(100.f, 0.f, 0.f); 
	SplineComponent->SetTangentAtSplinePoint(PointIndex, SecondPointTangent, ESplineCoordinateSpace::Local);

	// Set the spline point type 
	SplineComponent->SetSplinePointType(PointIndex, ESplinePointType::Curve);

	// Set the spline point type 
	SplineComponent->SetSplinePointType(0, ESplinePointType::CurveCustomTangent);

	// Set the tangent for the first spline point 
	FVector FirstPoint(1000.f, 0.f, 0.f); //300
	SplineComponent->SetTangentAtSplinePoint(0, SecondPointTangent, ESplineCoordinateSpace::Local);

	// Update the SplinePointIndicator's transform to match the second spline point
	if (SplinePointIndicator)
	{
		FTransform NewTransform = SplineComponent->GetTransformAtSplinePoint(PointIndex, ESplineCoordinateSpace::Local);
		SplinePointIndicator->SetWorldTransform(NewTransform);
	}

	// Force the spline to update its visualization in the editor (if needed)
	SplineComponent->UpdateSpline();
}

// Called when the game starts or when spawned
void AProceduralTunnel::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AProceduralTunnel::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

// DESTROY LAST PROCEDURAL MESH
void AProceduralTunnel::DestroyLastMesh() 
{
	int32 lastIndex = TunnelMeshes.Num() - 1;
	if (lastIndex >= 0) {
		TunnelMeshes[lastIndex]->DestroyComponent();
		TunnelMeshes.RemoveAt(lastIndex);
		lastIndex = meshEnds.Num() - 1;
		meshEnds.RemoveAt(lastIndex);
	}
}

// Function that connects the end of the current tunnel to the end of another tunnel spline
void AProceduralTunnel::SnapToEndOfOtherSpline()
{
	// Initialize variables to store the closest point, lowest distance, and closest tangent
	FVector closestPoint = FVector(0, 0, 0);
	FVector closestTangent;

	// Get the last spline point of the current tunnel
	int32 lastIndex = SplineComponent->GetNumberOfSplinePoints() - 1;
	FVector lastPointLocation = SplineComponent->GetLocationAtSplinePoint(lastIndex, ESplineCoordinateSpace::World);

	// Get all actors of class AProceduralTunnel in the world
	TArray<AActor*> FoundActors;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AProceduralTunnel::StaticClass(), FoundActors);

	// If we are
	bool connecToStart = false;

	// Loop through all the found tunnel actors
	for (AActor* tunnel : FoundActors)
	{
		// Cast the tunnel actor to a ProceduralTunnel object and get its spline component
		AProceduralTunnel* proceduralTunnel = Cast<AProceduralTunnel>(tunnel);
		USplineComponent* spline = proceduralTunnel->SplineComponent;

		// Ensure the spline being compared is not the current tunnel's spline
		if (spline != SplineComponent)
		{
			// Get the last spline point of the other tunnel
			int32 SplinePointIndexToSnap;
			if (proceduralTunnel->isFirstTunnel)
			{
				SplinePointIndexToSnap = 0;
			}
			else {
				SplinePointIndexToSnap = spline->GetNumberOfSplinePoints() - 1;
			}

			FVector comparedPointsLocation = spline->GetLocationAtSplinePoint(SplinePointIndexToSnap, ESplineCoordinateSpace::World);

			// Calculate the distance between the current tunnel end and the other tunnel end
			float distance = FVector::Distance(comparedPointsLocation, lastPointLocation);

			// Check if the distance is within the maximum allowed distance and update the closest point and tangent if necessary
			if (distance < maxDistance)
			{
				closestPoint = comparedPointsLocation;
				closestTangent = spline->GetTangentAtSplinePoint(SplinePointIndexToSnap, ESplineCoordinateSpace::World);
				connectedActor = proceduralTunnel;
				if (SplinePointIndexToSnap == 0) 
				{
					connecToStart = true;
				}
				else
				{
					connecToStart = false;
				}
				
			}
		}
		// If we are comparing self spline we snap to the start if to something
		else
		{
			FVector selfFirstPointLocation = spline->GetLocationAtSplinePoint(0, ESplineCoordinateSpace::World);

			// Calculate the distance between the current tunnel end and the other tunnel end
			float distance = FVector::Distance(selfFirstPointLocation, lastPointLocation);

			// Check if the distance is within the maximum allowed distance and update the closest point and tangent if necessary
			if (distance < maxDistance)
			{
				closestPoint = selfFirstPointLocation;
				closestTangent = spline->GetTangentAtSplinePoint(0, ESplineCoordinateSpace::World);
				connectedActor = proceduralTunnel;
				connecToStart = true;
			}
		}
	}

	// If a closest point was found, connect the ends of the tunnels and set the corresponding tangent
	if (closestPoint != FVector(0, 0, 0))
	{
		isEndConnected = true;
		connectedActor->isEndConnected = true; // Set the closest proceduralTunnel's isEndConnected to true
		SplineComponent->SetLocationAtSplinePoint(lastIndex, closestPoint, ESplineCoordinateSpace::World, true);
		if (connecToStart) {
			SplineComponent->SetTangentAtSplinePoint(lastIndex, closestTangent, ESplineCoordinateSpace::World, true);
		}
		else {
			SplineComponent->SetTangentAtSplinePoint(lastIndex, closestTangent * -1, ESplineCoordinateSpace::World, true);
		}
	}
	// If no closest point was found, set the end connection status to false
	else
	{
		isEndConnected = false;
	}
}


// WHEN TUNNEL IS DRAGGED FORWARD OF BACKWARD WE CONTROL THAT CHANGE HERE
void AProceduralTunnel::ControlSplinePoints(bool interSectionAdded)
{
	if(!isReset) 
	{
		if (!isEndConnected && !isUndo) // ADD SPLINE POINTS
		{ 
			AddOrRemoveSplinePoints(interSectionAdded);
		}
		else if (!isEndConnected && isUndo && SplineComponent->GetNumberOfSplinePoints() >= 2)
		{
			TunnelMeshes.Last()->DestroyComponent();
			TunnelMeshes.RemoveAt(TunnelMeshes.Num() - 1);
			meshEnds.RemoveAt(meshEnds.Num()-1);
		}
	} 
	else 
	{
		for(UProceduralMeshComponent* mesh : TunnelMeshes) 
		{
			mesh->DestroyComponent();
		}
		TunnelMeshes.Empty();
		meshEnds.Empty();
	}
}

// DOES WHAT IT SAYS IN NAME
void AProceduralTunnel::AddOrRemoveSplinePoints(bool interSectionAdded)
{
	float maxDistanceBetweenPoints = 750; //1000 original WAS 500 NOW
	float pointOffSet = 200;
	APlayerController* playerController = UGameplayStatics::GetPlayerController(GetWorld(), 0);
	if(playerController->WasInputKeyJustPressed(EKeys::LeftShift)) // IF LEFT SHIFT IS CLIKKED ADD NEW POINT WHERE WE ARE
	{ 										
		maxDistanceBetweenPoints = maxDistanceBetweenPoints / 2;
		int32 lastIndex = SplineComponent->GetNumberOfSplinePoints() - 1;
		FVector position = SplineComponent->GetLocationAtSplinePoint(lastIndex, ESplineCoordinateSpace::World);
		SplineComponent->AddSplinePointAtIndex(position, lastIndex, ESplineCoordinateSpace::World, true);
	}
	int32 numberOfPoints = SplineComponent->GetNumberOfSplinePoints();
	int32 lastIndex = numberOfPoints - 1;
	float currentDistance = SplineComponent->GetDistanceAlongSplineAtSplinePoint(lastIndex);
	float previousDistance = SplineComponent->GetDistanceAlongSplineAtSplinePoint(lastIndex - 1);
	float distanceBetweenPoints = currentDistance - previousDistance;
	int32 pointsToFitBetween = FMath::FloorToInt(distanceBetweenPoints / maxDistanceBetweenPoints);

	if (interSectionAdded) {
		pointsToFitBetween = pointsToFitBetween - 2;
	}

	if (pointsToFitBetween != 0) 
	{
		int32 splinePointIndex = SplineComponent->GetNumberOfSplinePoints() - 2;
		for (int32 i = 1; i <= pointsToFitBetween; i++)
		{
			float distance = SplineComponent->GetDistanceAlongSplineAtSplinePoint(SplineComponent->GetNumberOfSplinePoints() - 2);
			distance = distance + maxDistanceBetweenPoints;
			if(i == pointsToFitBetween) {
				distance = distance - pointOffSet;
			} 
			FVector location = SplineComponent->GetLocationAtDistanceAlongSpline(distance, ESplineCoordinateSpace::World);
			SplineComponent->AddSplinePointAtIndex(location, splinePointIndex + i, ESplineCoordinateSpace::World, true);
		}
	}
	else if (distanceBetweenPoints < pointOffSet && numberOfPoints > 3)
	{
		SplineComponent->RemoveSplinePoint(numberOfPoints -2);
		TunnelMeshes.Last()->DestroyComponent();
		TunnelMeshes.RemoveAt(TunnelMeshes.Num() - 1);
		meshEnds.RemoveAt(meshEnds.Num() - 1);
	}
}

// SET PARAMETERS FOR TUNNEL GENERATION LOOP
int32 AProceduralTunnel::SetUpProceduralGeneratorLoopParams()
{
	groundVerticeSize = (widthScale * 100.0f) / (float)(pointsInWidth - 1);
	wallVerticeSize = (heightScale * 100.0f) / (float)(pointsInHeight + 1);
	reCreateMeshCount = 2;
	meshInRework = -1;
	countOfMeshesToRemake = FMath::Clamp((SplineComponent->GetNumberOfSplinePoints() -2), 0, 2) + (fmax(0, ((SplineComponent->GetNumberOfSplinePoints() - 2) - TunnelMeshes.Num())));
	return countOfMeshesToRemake * -1;
}

void AProceduralTunnel::ProceduralGenerationLoop(int32 firstIndex, int32 lastIndex, bool isSinglePointUpdate, bool isIntersectionAdded, IntersectionType interType) {
	// Initialize variables for procedural generation loop
	InitializeProceduralGenerationLoopVariables(firstIndex, lastIndex, interType);

	// Loop through the tunnel sections
	for (int32 index = firstIndex; index <= lastIndex; index++) {
		meshLoopCurrentIndex = abs(index);
		ClearArrays();

		// Set up spline locations for this tunnel section
		SetupSplineLocations();

		// Check if there is enough space for at least one step
		if (pointCapLoopLastIndex > 0) {
			// Reset the current mesh end data
			ResetCurrentMeshEndData();

			// Generate vertices and UVs for the tunnel mesh
			GenerateVerticesAndUVs(isSinglePointUpdate, isIntersectionAdded, lastIndex);

			// Create the mesh triangles, tangents, and normals
			MakeMeshTriangles();
			MakeMeshTangentsAndNormals();

			// Build the mesh with the generated data
			MakeMesh(meshLoopCurrentIndex, isSinglePointUpdate, isLoad);

			// Append vertices if required for loading or resetting
			if (isLoad || isReset) {
				allFloorVertices.Append(groundVertices);
			}
		}
	}
}

// Initialize variables required for the procedural generation loop
void AProceduralTunnel::InitializeProceduralGenerationLoopVariables(int32 firstIndex, int32 lastIndex, IntersectionType interType) {
	intersectionType = interType;
	meshLoopFirstIndex = abs(firstIndex);
	meshLoopLastIndex = abs(lastIndex);

	// Check for a parent actor and set parentIntersection and parentsParentTunnel accordingly
	if (IsChildActor()) {
		parentIntersection = Cast<AProceduralIntersection>(GetParentActor());
		parentsParentTunnel = Cast<AProceduralTunnel>(parentIntersection->parentTunnel);
	}
}

// Calculate the start and end locations on the spline for this tunnel section
void AProceduralTunnel::SetupSplineLocations() {
	float latestLocation = SplineComponent->GetDistanceAlongSplineAtSplinePoint(SplineComponent->GetNumberOfSplinePoints() - (meshLoopCurrentIndex + 1));
	float previousLocation = SplineComponent->GetDistanceAlongSplineAtSplinePoint(SplineComponent->GetNumberOfSplinePoints() - (meshLoopCurrentIndex + 2));
	pointCapLoopLastIndex = FMath::Floor((latestLocation - previousLocation) / stepSizeOnSpline);
	float remainder = ((latestLocation - previousLocation) / stepSizeOnSpline) - (float)pointCapLoopLastIndex;
	lastStepSizeOnSpline = stepSizeOnSpline * remainder + stepSizeOnSpline;
}

// Reset the current mesh end data
void AProceduralTunnel::ResetCurrentMeshEndData() {
	currentMeshEndData = FMeshSectionEnd();
}

// Generate vertices and UVs for the tunnel mesh
void AProceduralTunnel::GenerateVerticesAndUVs(bool isSinglePointUpdate, bool isIntersectionAdded, int32 lastIndex) {
	for (pointCapLoopCurrentIndex = 0; pointCapLoopCurrentIndex <= pointCapLoopLastIndex; pointCapLoopCurrentIndex++) {
		// Clear arrays holding vertice data of start of tunnel
		if (IsFirstLoopOfWholeTunnel()) {
			firstFloorVertices.Empty();
			firstRightVertices.Empty();
			firstRoofVertices.Empty();
			firstLeftVertices.Empty();
		}
		if (usePreviousEndVertices(isIntersectionAdded, isSinglePointUpdate)) {
			UsePreviousEndVerticesData(isSinglePointUpdate);
		}
		else {
			GenerateVerticesForCurrentLoop(isSinglePointUpdate, isIntersectionAdded, lastIndex);
		}
	}
}

bool AProceduralTunnel::usePreviousEndVertices (bool isIntersectionAdded, bool isSinglePointUpdate) {
	int32 z = SplineComponent->GetNumberOfSplinePoints() - 3 - meshLoopCurrentIndex;
	if(isIntersectionAdded && pointCapLoopCurrentIndex == pointCapLoopLastIndex) {
		return false;
	} 
	else 
	{
		// FIRST LOOP AROUND && THERE IS END PART TO USE && WE ARE IN NOT TRYING NEGATIVES OR SOMETHING LIKE THAT LAST 2 OF CLAUSE
		if((pointCapLoopCurrentIndex == 0) && (meshEnds.Num() > 0) && (z >= 0) && (meshEnds.Num() - 1 >= z)) {
			return true;
		}
		// IF HEIGHT IS ADJUSTED AND WE ARE IN THE LAST MESHES END
		if(meshLoopCurrentIndex == meshLoopLastIndex && pointCapLoopCurrentIndex == pointCapLoopLastIndex && isSinglePointUpdate) {
			return true;	
		}
		return false;
	}
}

// Use the previous end vertices data for the current tunnel section
void AProceduralTunnel::UsePreviousEndVerticesData(bool isSinglePointUpdate) {
	// Calculate the index that provides the correct mesh end data from the array (previous meshes end)
	int32 meshEndIndex = SplineComponent->GetNumberOfSplinePoints() - 3 - meshLoopCurrentIndex; 

	// Check if we are at the end of the last mesh that we are generating and the height is adjusted
	// In this case, we want to use this mesh's previously saved end data
	if (meshLoopCurrentIndex == meshLoopLastIndex && pointCapLoopCurrentIndex == pointCapLoopLastIndex && isSinglePointUpdate) 
	{																													 
		meshEndIndex = meshEnds.Num() - 1 - meshLoopCurrentIndex;
		currentMeshEndData = meshEnds[meshEndIndex];
	}

	// Get the end data for the current tunnel section
	FMeshSectionEnd end = meshEnds[meshEndIndex]; 

	// Add end data to arrays that are used to generate the tunnel
	groundVertices.Append(end.GroundVertives);	  
	groundUV.Append(end.GroundUV);
	wallVertices.Append(end.WallVertices);
	wallUV.Append(end.WallUV);
}

void AProceduralTunnel::GenerateVerticesForCurrentLoop(bool isSinglePointUpdate, bool isIntersectionAdded, int32 lastIndex) {
	InitializeStartVectorRightVectorAndValueInTexture();

	// Loop through each point in the loop that goes around tunnel
	for (loopAroundTunnelCurrentIndex = 1; loopAroundTunnelCurrentIndex <= loopAroundTunnelLastIndex; loopAroundTunnelCurrentIndex++) {
		// Get the surface index and array index for this point
		surfaceIndex = GetSurfaceIndex();
		arrayIndex = CalculateArrayIndex();

		// Check if this is the first loop around and if it's a special tunnel type
		bool isFirstLoopAround = GetIsFirstLoopAround();

		if (isFirstLoopAround && tunnelType != TunnelType::DefaultTunnel) {
			// Get the latest vertice for intersection continuation tunnels
			latestVertice = GetLatestVerticeForIntersectionContinuationTunnels();
		}
		// Check if this tunnel is connected to another tunnel
		else if (IsConnectedToOtherTunnel(isSinglePointUpdate)) {
			// Get the vertice from the connected tunnel
			latestVertice = GetVerticeForConnectedTunnel();
		}
		// Check if this tunnel has an intersection added to it
		else if (IsIntersectionAddedToThisTunnel(isIntersectionAdded, lastIndex)) {
			// Get the vertice from the intersection
			latestVertice = GetVerticeForIntersectionAddedTunnel();
		}
		else {
			// Get the vertice for a default tunnel
			latestVertice = GetVerticeForDefaultTunnel(isFirstLoopAround, isIntersectionAdded);
		}

		// Adjust the latest vertice for overlap
		latestVertice = AdjustLatestVerticeForOverlap(latestVertice, surfaceIndex);

		// Save the first vertice if needed
		SaveFirstVerticeIfNeeded(latestVertice);

		// Add the vertice to the appropriate array
		AddCreatedVerticeToArrays(latestVertice, surfaceIndex);

		// Save the first loop vertice data if this is the first loop of the whole tunnel
		if (IsFirstLoopOfWholeTunnel()) {
			SaveFirstLoopVerticeData(surfaceIndex, latestVertice);
		}

		// Save the end mesh vertice data if this is the end of the current mesh
		if (IsEndOfCurrentMesh()) {
			SaveEndMeshVerticeData(surfaceIndex, latestVertice);
		}

		// Add UV coordinates for this point
		AddUVCoordinates(surfaceIndex, pointCapLoopCurrentIndex, pointCapLoopLastIndex, loopAroundTunnelCurrentIndex);
	}
}

// Returns the latest vertice for the intersection continuation tunnels
FVector AProceduralTunnel::GetLatestVerticeForIntersectionContinuationTunnels() {
	FVector vertice;
	switch (tunnelType) {
	case 0:
		vertice = RightTunnelStart();
		break;
	case 1:
		vertice = LeftTunnelStart();
		break;
	case 2:
		vertice = StraightTunnelStart();
		break;
	}
	return vertice;
}

// Check if the current tunnel is connected to another tunnel and not adjusting height
bool AProceduralTunnel::IsConnectedToOtherTunnel(bool isSinglePointUpdate) {
	// Check if the current tunnel is at its end and connected to another tunnel
	return !isSinglePointUpdate && isEndConnected && meshLoopCurrentIndex == meshLoopLastIndex && pointCapLoopCurrentIndex == pointCapLoopLastIndex && IsValid(connectedActor);
}

// Get the vertices from the connected tunnel mesh based on the current surface index.
FVector AProceduralTunnel::GetVerticeForConnectedTunnel() {
	TArray<FVector> verticeArrayToUse;

    switch (surfaceIndex) {
        case 0:
			if (connectedActor->isFirstTunnel) {
				verticeArrayToUse = connectedActor->firstFloorVertices;
			}
			else {
				verticeArrayToUse = connectedActor->lastFloorVertices;
			}
            break;
        case 1:
			if (connectedActor->isFirstTunnel) {
				verticeArrayToUse = connectedActor->firstRightVertices;
			}
			else {
				verticeArrayToUse = connectedActor->lastLeftVertices;
			}
            break;
        case 2:
			if (connectedActor->isFirstTunnel) {
				verticeArrayToUse = connectedActor->firstRoofVertices;
			}
			else {
				verticeArrayToUse = connectedActor->lastRoofVertices;
			}
            break;
        case 3:
			if (connectedActor->isFirstTunnel) {
				verticeArrayToUse = connectedActor->firstLeftVertices;
			}
			else {
				verticeArrayToUse = connectedActor->lastRightVertices;
			}
            break;
    }
	// Get the specific vertex to use from the vertex array.
	FVector verticeToUse;
	if (connectedActor->isFirstTunnel) {
		verticeToUse = verticeArrayToUse[arrayIndex];
	}
	else {
		verticeToUse = verticeArrayToUse[FMath::Clamp(verticeArrayToUse.Num() - (1 + arrayIndex), 0, verticeArrayToUse.Num() - 1)];
	}

	/*if (connectedActor == this) {
		return verticeToUse;
	}*/
    

	// Transform the vertex from the connected actor's transform to world transform.
    verticeToUse = UKismetMathLibrary::TransformLocation(connectedActor->GetTransform(), verticeToUse); 

	// Transform the vertex from world transform to local transform of this tunnel mesh.
    return UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), verticeToUse); 
}

// Check if the parent intersection is valid and there are mesh ends before the intersection
// This is to ensure that an intersection was actually added to this tunnel
bool AProceduralTunnel::IsIntersectionAddedToThisTunnel(bool isIntersectionAdded, int32 lastIndex) {
	return IsValid(parentIntersection) && meshEndsBeforeIntersection.Num() > 0 && meshLoopCurrentIndex == abs(lastIndex) && pointCapLoopCurrentIndex == pointCapLoopLastIndex;
}

// Get the vertice for a tunnel that has intersection added
FVector AProceduralTunnel::GetVerticeForIntersectionAddedTunnel() {
	// Get the last mesh section before the intersection
	if (meshEndsBeforeIntersection.Num() - 1 >= 0) {
		FMeshSectionEnd meshEnd = meshEndsBeforeIntersection[meshEndsBeforeIntersection.Num() - 1];
		TArray<FVector> verticeArrayToSelect;
		int32 verticeIndex;

		// Check if the surface is not the ground
		if (surfaceIndex != 0) {
			// Compute the index of the vertice on the wall surface
			verticeIndex = loopAroundTunnelCurrentIndex - (pointsInWidth + 2);
			verticeArrayToSelect = meshEnd.WallVertices;
		}
		else {
			// Compute the index of the vertice on the ground surface
			verticeIndex = loopAroundTunnelCurrentIndex - 1;
			verticeArrayToSelect = meshEnd.GroundVertives;
		}

		// Return the selected vertex
		return verticeArrayToSelect[verticeIndex];
	}

	// Return the zero vector if there are no mesh sections before the intersection
	return FVector::ZeroVector;
}

// Returns the appropriate vertice for a default tunnel segment based on the surface index, whether it's the first loop around the tunnel, and if there's an intersection added.
FVector AProceduralTunnel::GetVerticeForDefaultTunnel(bool isFirstLoopAround, bool isIntersectionAdded) {
	FVector vertice;
	switch (surfaceIndex) {
	case 0:
		vertice = GetFloorVertice();
		break;
	case 1:
		vertice = GetRightVertice(isFirstLoopAround, isIntersectionAdded);
		break;
	case 2:
		vertice = GetRoofVertice();
		break;
	case 3:
		vertice = GetLeftVertice(isFirstLoopAround, isIntersectionAdded);
		break;
	}
	return vertice;
}

// Adjusts the latest vertice position for overlap, ensuring a smooth transition between tunnel segments.
FVector AProceduralTunnel::AdjustLatestVerticeForOverlap(FVector vertice, int32 surface) {
	FVector previousLoopVertice = FVector(0.0f, 0.0f, 0.0f);
	// On the walls/roof
	if (surface != 0) {
		if (wallVertices.Num() - (pointsInHeight * 2 + pointsInWidth + 1) >= 0) {
			previousLoopVertice = wallVertices[wallVertices.Num() - (pointsInHeight * 2 + pointsInWidth + 1)];
		}
	}
	// On the floor
	else {
		if (groundVertices.Num() - (pointsInWidth + 1) >= 0) {
			previousLoopVertice = groundVertices[groundVertices.Num() - (pointsInWidth + 1)];
		}
	}
	// This is forward vector pointing at startLocationOnSpline
	forwardVector;

	// If a valid vertice is found for comparison, adjust the current vertice position if necessary.
	if (previousLoopVertice != FVector(0.0f, 0.0f, 0.0f)) {
		// Calculate the projections of the vectors onto the forward vector
		float currentVerticeProjection = FVector::DotProduct((vertice - startLocationOnSpline).GetSafeNormal(), forwardVector);
		float previousVerticeProjection = FVector::DotProduct((previousLoopVertice - startLocationOnSpline).GetSafeNormal(), forwardVector);

		// If previous vertices projection is larger than current vertices projection, we set current vertice to previous vertices location.
		if (previousVerticeProjection >= currentVerticeProjection) {
			vertice = previousLoopVertice;
		}
	}

	return vertice;
}

// Saves the first vertice of the loop if the current index of the loop around the tunnel is 1.
void AProceduralTunnel::SaveFirstVerticeIfNeeded(FVector vertice) {
	if (loopAroundTunnelCurrentIndex == 1) {
		firstVertice = vertice;
	}
}

// Adds the generated vertice to the appropriate array based on the surface type.
void AProceduralTunnel::AddCreatedVerticeToArrays(FVector vertice, int32 surface) {
	// If the surface type is 0 (floor), add the vertice to the groundVertices array.
	if (surface == 0) {
		groundVertices.Add(vertice);
	}
	// For all other surface types (1 - right wall, 2 - roof, 3 - left wall),
	// add the vertice to the wallVertices array.
	else {
		wallVertices.Add(vertice);
	}
}

// Determines if the current loop is the first loop of the entire tunnel.
bool AProceduralTunnel::IsFirstLoopOfWholeTunnel() {
	// Returns true if the conditions for the first loop of the whole tunnel are met:
	// - The spline point index (meshLoopCurrentIndex) is one less than the total number of spline points,
	//   or there are no tunnel meshes.
	// - The point cap loop index (pointCapLoopCurrentIndex) is 0.
	return ((SplineComponent->GetNumberOfSplinePoints() - 1) - meshLoopCurrentIndex == 1 || TunnelMeshes.Num() == 0) && pointCapLoopCurrentIndex == 0;
}

// Stores the vertices of the first loop in the tunnel based on the provided surface index.
void AProceduralTunnel::SaveFirstLoopVerticeData(int32 surface, FVector vertice) {
	// Selects the appropriate array to save the vertex data based on the surface index.
	switch (surface) {
	case 0:
		firstFloorVertices.Add(vertice);
		break;
	case 1:
		firstRightVertices.Add(vertice); 
		break;
	case 2:
		firstRoofVertices.Add(vertice);
		break;
	case 3:
		firstLeftVertices.Add(vertice);
		break;
	}
}

// Determines if the current loop iteration is at the end of the current mesh.
bool AProceduralTunnel::IsEndOfCurrentMesh() {
	// Returns true if the current loop index is equal to the last loop index, indicating the end of the current mesh.
	return pointCapLoopCurrentIndex == pointCapLoopLastIndex;
}

// Saves the end mesh vertice data based on the provided surface index and vertice position.
void AProceduralTunnel::SaveEndMeshVerticeData(int32 surface, FVector vertice) {
	// Switch on the surface index to determine which part of the tunnel to update.
	switch (surface) {
		case 0: // Floor
			lastFloorVertices.Add(vertice);
			currentMeshEndData.GroundVertives.Add(vertice);
			break;
		case 1: // Right wall
			lastRightVertices.Add(vertice);
			currentMeshEndData.WallVertices.Add(vertice);
			break;
		case 2: // Roof
			lastRoofVertices.Add(vertice);
			currentMeshEndData.WallVertices.Add(vertice);
			break;
		case 3: // Left wall
			lastLeftVertices.Add(vertice);
			currentMeshEndData.WallVertices.Add(vertice);
			break;
	}
}

// Adds UV coordinates to the mesh based on the provided surface index, loop index between points,
// last index between points, and current index around the tunnel.
void AProceduralTunnel::AddUVCoordinates(int32 surface, int32 loopIndexBetweenPoints, int32 lastIndexBetweenPoints, int32 currentIndexAroundTunnel) {
	float xValue = loopIndexBetweenPoints;
	float yValue;

	// If we are at the last point between points, set the xValue to 0.
	if (lastIndexBetweenPoints == 0) {
		xValue = 0.0f;
	}

	// If the surface is the floor (0), calculate yValue based on the current index around the tunnel
	// and the number of points in the width, then add the UV coordinates to the ground arrays.
	if (surface == 0) {
		yValue = static_cast<float>(currentIndexAroundTunnel) / pointsInWidth;
		groundUV.Add(FVector2D(xValue, yValue));
		currentMeshEndData.GroundUV.Add(FVector2D(xValue, yValue));
	}
	// If the surface is not the floor, calculate yValue based on the current index around the tunnel,
	// number of points in width, and number of points in height, then add the UV coordinates to the wall arrays.
	else {
		yValue = static_cast<float>(currentIndexAroundTunnel - pointsInWidth) / (pointsInWidth + (pointsInHeight * 2));
		wallUV.Add(FVector2D(xValue, yValue));
		currentMeshEndData.WallUV.Add(FVector2D(xValue, yValue));
	}
}

// GET VERTICES FOR THE START OF RIGHT TUNNEL
FVector AProceduralTunnel::RightTunnelStart() 
{
	FVector vertice;
	switch (surfaceIndex)
	{
	case 0:
		vertice = parentIntersection->lastRightFloorVertices[parentIntersection->lastRightFloorVertices.Num() - (1 + arrayIndex)];
		vertice = UKismetMathLibrary::TransformLocation(parentIntersection->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
		vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);
		return vertice;
		break;
	case 1:
		vertice = parentsParentTunnel->lastRightVertices[arrayIndex];
		vertice = UKismetMathLibrary::TransformLocation(parentsParentTunnel->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
		vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);
		return vertice;
		break;
	case 2:
		vertice = parentIntersection->lastRightRoofVertices[arrayIndex];
		vertice = UKismetMathLibrary::TransformLocation(parentIntersection->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
		vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);
		return vertice;
		break;
	case 3:
		if(parentIntersection->intersectionType == IntersectionType::RightLeft)
		{
			vertice = parentIntersection->lastRightWallVertices[arrayIndex];
			vertice = UKismetMathLibrary::TransformLocation(parentIntersection->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
			vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);
			return vertice;
			break;
		}
		else 
		{
			return GetLeftVertice(true, false);;
			break;
		}	
	default:
		return FVector(0,0,0);
		break;
	}
}

// GET VERTICES FOR THE START OF LEFT TUNNEL
FVector AProceduralTunnel::LeftTunnelStart() 
{
	FVector vertice;
	switch (surfaceIndex)
	{
	case 0:
		vertice = parentIntersection->lastLeftFloorVertices[arrayIndex];
		vertice = UKismetMathLibrary::TransformLocation(parentIntersection->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
		vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);
		return vertice;
		break;
	case 1:
		if(parentIntersection->intersectionType == IntersectionType::RightLeft)
		{
			vertice = parentIntersection->lastLeftWallVertices[parentIntersection->lastLeftWallVertices.Num() - (1 + arrayIndex)];
			vertice = UKismetMathLibrary::TransformLocation(parentIntersection->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
			vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);
			return vertice;
			break;
		}
		else 
		{
			return GetRightVertice(true, false);;
			break;
		}
	case 2:
		vertice = parentIntersection->lastLeftRoofVertices[parentIntersection->lastLeftRoofVertices.Num() - (1 + arrayIndex)];
		vertice = UKismetMathLibrary::TransformLocation(parentIntersection->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
		vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);
		return vertice;
		break;
	case 3:
		vertice = parentsParentTunnel->lastLeftVertices[arrayIndex];
		vertice = UKismetMathLibrary::TransformLocation(parentsParentTunnel->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
		vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);
		return vertice;
		break;
	default:
		return FVector(0,0,0);
		break;
	}
}

// GET VERTICES FOR THE START OF STRAIGHT TUNNEL
FVector AProceduralTunnel::StraightTunnelStart() 
{
	FVector vertice;
	switch (surfaceIndex)
	{
	case 0:
		vertice = parentIntersection->lastStraightFloorVertices[arrayIndex];
		vertice = UKismetMathLibrary::TransformLocation(parentIntersection->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
		vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);
		return vertice;
		break;
	case 1:
		if(IsValid(rightSideTunnel))
		{
			vertice = rightSideTunnel->firstLeftVertices[rightSideTunnel->firstLeftVertices.Num() - (1 + arrayIndex)];
			vertice = UKismetMathLibrary::TransformLocation(rightSideTunnel->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
			vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);
			return vertice;
			break;
		} 
		else
		{
			vertice = parentIntersection->lastRightWallVertices[arrayIndex];
			vertice = UKismetMathLibrary::TransformLocation(parentIntersection->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
			vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);
			return vertice;
			break;
		}
	case 2:
		vertice = parentIntersection->lastStraightRoofVertices[arrayIndex];
		vertice = UKismetMathLibrary::TransformLocation(parentIntersection->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
		vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);
		return vertice;
		break;
	case 3:
		if (IsValid(leftSideTunnel))
		{
			vertice = leftSideTunnel->firstRightVertices[leftSideTunnel->firstRightVertices.Num() - (1 + arrayIndex)];
			vertice = UKismetMathLibrary::TransformLocation(leftSideTunnel->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
			vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);
			return vertice;
			break;
		} 
		else 
		{
			vertice = parentIntersection->lastLeftWallVertices[arrayIndex];
			vertice = UKismetMathLibrary::TransformLocation(parentIntersection->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
			vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);
			return vertice;
			break;
		}
	default:
		return FVector(0,0,0);
		break;
	}
}

// GET VERTICE LOCATION IN FLOOR
FVector AProceduralTunnel::GetFloorVertice()
{
	// Calculate the distance along the spline
	float startDistance = SplineComponent->GetDistanceAlongSplineAtSplinePoint(SplineComponent->GetNumberOfSplinePoints() - (2 + meshLoopCurrentIndex));
	float currentDistance;

	// Determine the current distance based on the loop index
	if (pointCapLoopCurrentIndex == pointCapLoopLastIndex && meshLoopCurrentIndex == meshLoopLastIndex) {
		currentDistance = startDistance + ((float)(pointCapLoopCurrentIndex - 1) * stepSizeOnSpline + lastStepSizeOnSpline);
	} else {
		currentDistance = startDistance + (float)pointCapLoopCurrentIndex * stepSizeOnSpline;
	}

	// Calculate the position of the vertex on the side of the floor
	float sideWaysMultiplier = (float)(pointsInWidth / 2) * groundVerticeSize;
	FVector negativeRightVector = rightVector * -1;
	FVector stepLocationFromStart = SplineComponent->GetLocationAtDistanceAlongSpline(currentDistance, ESplineCoordinateSpace::Local);
	FVector startLocationOnSide = stepLocationFromStart + FVector(negativeRightVector.X*sideWaysMultiplier, negativeRightVector.Y*sideWaysMultiplier, negativeRightVector.Z*sideWaysMultiplier);

	// Calculate the deformation amount
	float deformAmount = GetPixelValue(forwardStepInDeformTexture, loopAroundTunnelCurrentIndex);
	float amountOfDeform = FMath::Lerp(0.0f, deformAmount, floorDeformation);
	float deform = FMath::RandRange(amountOfDeform * -20, amountOfDeform * 20);


	if (loopAroundTunnelCurrentIndex == 1) 
	{
		return latestVertice; //- FVector(0,0, amountOfDeform);
	} 
	else 
	{
		FVector stepToSide = startLocationOnSide + FVector(rightVector.X, rightVector.Y, 0) * groundVerticeSize * (loopAroundTunnelCurrentIndex - 1);
		return FVector(stepToSide.X, stepToSide.Y, stepToSide.Z - deform);
	}
}

// GET VERTICE LOCATION IN RIGHT WALL
FVector AProceduralTunnel::GetRightVertice(bool isFirstLoopARound, bool isIntersectionAdded)
{
	bool isEndOrStar = false;

	// Right wall have to start from where floor ends to prevent any cap between floor and wall meshes
	if (loopAroundTunnelCurrentIndex == pointsInWidth + splineLoopAdjustment)
	{
		// Save the starting vertice when entering first time
		wallStartVertice = latestVertice;
		return latestVertice;
	}

	FVector rVector = rightVector;
	float extraMovementToEnd = 0.0f;

	// Check if we need to rotate the starting position to align with the intersection.
	if ((tunnelType != TunnelType::DefaultTunnel && pointCapLoopCurrentIndex <= 4 && ((SplineComponent->GetNumberOfSplinePoints() - 1 - meshLoopCurrentIndex == 1) || TunnelMeshes.Num() == 0)))
	{
		isEndOrStar = true;
		float rotateAmount = 0.0f;
		float alpha = pointCapLoopCurrentIndex / 4.0f;
		rotateAmount = FMath::Lerp(-45.0f, 0.0f, alpha);
		switch (pointCapLoopCurrentIndex) {
		case 0:
			extraMovementToEnd = 100.0f;
			break;
		case 1:
			extraMovementToEnd = 50.0f;
			break;
		case 2:
			extraMovementToEnd = 30.0f;
			break;
		case 3:
			extraMovementToEnd = 10.0f;
			break;
		case 4:
			extraMovementToEnd = 0.0f;
			break;
		}
		FRotator rotator = FRotator(0.0f, rotateAmount, 0.0f);
		rVector = rotator.RotateVector(rightVector);
	}
	
	// Check if we need to rotate the end right wall vertices to align with the continuation tunnel of the intersection.
	if (meshLoopLastIndex == meshLoopCurrentIndex && pointCapLoopCurrentIndex <= pointCapLoopLastIndex && pointCapLoopCurrentIndex >= pointCapLoopLastIndex - 4 && isIntersectionAdded && (intersectionType == IntersectionType::Right || intersectionType == IntersectionType::All || intersectionType == IntersectionType::RightLeft))
	{
		isEndOrStar = true;
		float rotateAmount = 0.0f;
		int index = pointCapLoopCurrentIndex - pointCapLoopLastIndex + 4; // 0-4
		float alpha = index / 4.0f;
		rotateAmount = FMath::Lerp(0.0f, 45.0f, alpha);

		switch (index) {
		case 0:
			extraMovementToEnd = 0.0f;
			break;
		case 1:
			extraMovementToEnd = 10.0f;
			break;
		case 2:
			extraMovementToEnd = 30.0f;
			break;
		case 3:
			extraMovementToEnd = 50.0f;
			break;
		case 4:
			extraMovementToEnd = 100.0f;
			break;
		}
		FRotator rotator = FRotator(0.0f, rotateAmount, 0.0f);
		rVector = rotator.RotateVector(rightVector);
	}

	// Get the location on normalized range at what point are on wall ( 0 = start and 1 = end )
	float locationOnWall = (float)(loopAroundTunnelCurrentIndex - (pointsInWidth + splineLoopAdjustment)) / (float)(pointsInHeight - 1);

	float wVerticeSize = wallVerticeSize;

	// Adjust the vertice size for the first loop around 
	if (isFirstLoopARound && IsValid(parentIntersection))
	{
		wVerticeSize = parentIntersection->wallVerticeSize;
	}


	// Add roundness to the tunnel.
	float roundnessAmount = deformCurve->GetFloatValue(locationOnWall);
	FVector tunnelRoundnesss = rVector * (FMath::Lerp((tunnelRoundValue + extraMovementToEnd), 0.0f, roundnessAmount));

	float zLocation = (loopAroundTunnelCurrentIndex - (pointsInWidth + splineLoopAdjustment)) * wVerticeSize;
	FVector wallVertice = wallStartVertice + FVector(0.0f, 0.0f, zLocation);
	wallVertice += tunnelRoundnesss;

	// Dont add deformation to end or start of tunnel
	if (!isEndOrStar) {
		/* DEFORMATION */
		float deformAmount = GetPixelValue(forwardStepInDeformTexture, loopAroundTunnelCurrentIndex);
		//float deformArea = stopDeformCurve->GetFloatValue(locationOnWall);
		//float deformFromLerp = FMath::Lerp(0.0f, deformAmount, deformArea);				/// lerp out bottom of wall

		wallVertice += FMath::Lerp(0.0f, maxDeform, wallDeformation) * deformAmount * rightVector;
	}

	return wallVertice;
}

// GET VERTICE LOCATION IN ROOF
FVector AProceduralTunnel::GetRoofVertice()
{
	if (loopAroundTunnelCurrentIndex == pointsInWidth + pointsInHeight + splineLoopAdjustment)
	{
		wallStartVertice = latestVertice;
	}

	FVector wallVertice = wallStartVertice;

    // Apply any sideway movement to the starting location
    //if (loopAroundTunnelCurrentIndex != (pointsInWidth + pointsInHeight + splineLoopAdjustment))
    //{
        float sideWaysMovementSize = (loopAroundTunnelCurrentIndex - (pointsInWidth + pointsInHeight + 2)) * -groundVerticeSize;
		wallVertice += rightVector * sideWaysMovementSize;
    //}

    // Apply roundness to the starting location
    float roundingIndex = (loopAroundTunnelCurrentIndex - (pointsInWidth + pointsInHeight + splineLoopAdjustment)) / static_cast<float>(pointsInWidth);
    float roundnessAmount = deformCurve->GetFloatValue(roundingIndex);
    float tunnelRounding = FMath::Lerp(tunnelRoundValue, 0.0f, roundnessAmount);
	wallVertice.Z += tunnelRounding;

    // Apply deformation to the starting location
    float deformAmount = GetPixelValue(forwardStepInDeformTexture, loopAroundTunnelCurrentIndex);
	wallVertice.Z += FMath::Lerp(0.0f, maxDeform, wallDeformation) * deformAmount;

    return wallVertice;
}

// GET VERTICE LOCATION IN LEFT WALL
FVector AProceduralTunnel::GetLeftVertice(bool isFirstLoopARound, bool isIntersectionAdded)
{
	bool isEndOrStar = false;

	// If we have completed a full loop around the tunnel, return the first vertice.
	if (loopAroundTunnelCurrentIndex == loopAroundTunnelLastIndex)
	{
		return firstVertice;
	}

	// Save the starting vertice when entering first time
	if (loopAroundTunnelCurrentIndex == pointsInWidth * 2 + pointsInHeight + splineLoopAdjustment + 1)
	{
		wallStartVertice = latestVertice;
	}
	
	float extraMovementToEnd = 0.0f;
	FVector rVector = rightVector;
	float wVerticeSize = wallVerticeSize;

	// If this is the first loop around the tunnel and a parent intersection is valid, use the parent's wall vertice size.
	if (isFirstLoopARound && IsValid(parentIntersection))
	{
		wVerticeSize = parentIntersection->wallVerticeSize;
	}

	// Rotate start of wall vertices to align with parent of intersection.
	if ((tunnelType != TunnelType::DefaultTunnel && pointCapLoopCurrentIndex <= 4 && ((SplineComponent->GetNumberOfSplinePoints() - 1 - meshLoopCurrentIndex == 1) || TunnelMeshes.Num() == 0)))
	{
		if ((tunnelType == TunnelType::StraightTunnel && IsValid(leftSideTunnel)) || tunnelType != TunnelType::StraightTunnel)
		{
			isEndOrStar = true;
			wallStartVertice = FVector(firstVertice.X, firstVertice.Y, firstVertice.Z + ((float)(pointsInHeight)*wallVerticeSize));


			float rotateAmount = 0.0f;
			float alpha = pointCapLoopCurrentIndex / 4.0f;
			rotateAmount = FMath::Lerp(45.0f, 0.0f, alpha);

			switch (pointCapLoopCurrentIndex) {
			case 0:
				extraMovementToEnd = 100.0f;
				break;
			case 1:
				extraMovementToEnd = 50.0f;
				break;
			case 2:
				extraMovementToEnd = 30.0f;
				break;
			case 3:
				extraMovementToEnd = 10.0f;
				break;
			case 4:
				extraMovementToEnd = 0.0f;
				break;
			}

			// Rotate the rVector and update it.
			FRotator rotator = FRotator(0.0f, rotateAmount, 0.0F);
			rVector = rotator.RotateVector(rightVector);
		}
	}

	// Rotate the end right wall vertices to round up with continuation tunnel of intersection, if applicable.
	if (meshLoopLastIndex == meshLoopCurrentIndex && pointCapLoopCurrentIndex <= pointCapLoopLastIndex && pointCapLoopCurrentIndex >= pointCapLoopLastIndex - 4 && isIntersectionAdded &&
		(intersectionType == IntersectionType::Left || intersectionType == IntersectionType::All || intersectionType == IntersectionType::RightLeft))
	{
		isEndOrStar = true;

		float rotateAmount = 0.0f;
		int index = pointCapLoopCurrentIndex - pointCapLoopLastIndex + 4; // 0-4
		float alpha = index / 4.0f;
		rotateAmount = FMath::Lerp(0.0f, -45.0f, alpha);

		switch (index) {
		case 0:
			extraMovementToEnd = 0.0f;
			break;
		case 1:
			extraMovementToEnd = 10.0f;
			break;
		case 2:
			extraMovementToEnd = 30.0f;
			break;
		case 3:
			extraMovementToEnd = 50.0f;
			break;
		case 4:
			extraMovementToEnd = 100.0f;
			break;
		}

		// Rotate the rVector and update it.
		FRotator rotator = FRotator(0.0f, rotateAmount, 0.0f);
		rVector = rotator.RotateVector(rightVector);
	}

	// Get the location on normalized range at what point are on wall ( 0 = start and 1 = end )
	float locationOnWall = (float)(loopAroundTunnelCurrentIndex - (pointsInWidth * 2 + pointsInHeight + splineLoopAdjustment)) / (float)pointsInHeight;
	// Add roundness to the tunnel wall

	float roundnessAmount = deformCurve->GetFloatValue(locationOnWall);
	if (isIntersectionAdded && pointCapLoopCurrentIndex == pointCapLoopLastIndex)
	{
		roundnessAmount = FMath::Clamp((float)(roundnessAmount - 0.1f), 0.0f, 1.0f);
	}
	FVector tunnelRoundnesss = rVector * FMath::Lerp((tunnelRoundValue + extraMovementToEnd) * -1, 0.0f, roundnessAmount); // AMOUNT WE MOVE TO SIDE TO CREATE  THAT ROUND LOOK OF TUNNEL
	float zLocation = (loopAroundTunnelCurrentIndex - (pointsInWidth * 2 + pointsInHeight + splineLoopAdjustment)) * (wVerticeSize * -1.0f);


	FVector wallVertice = wallStartVertice + FVector(0.0f, 0.0f, zLocation);
	wallVertice += tunnelRoundnesss;

	if (!isEndOrStar) {
		// Calculate deformation of the wall
		float deformAmount = GetPixelValue(forwardStepInDeformTexture, loopAroundTunnelCurrentIndex);
		//float deformArea = stopDeformCurve->GetFloatValue(locationOnWall);
		//float deformFromLerp = FMath::Lerp(0.0f, deformAmount, deformArea);				/// lerp out bottom of wall
		wallVertice -= FMath::Lerp(0.0f, maxDeform, wallDeformation) * deformAmount * rightVector;
	}

	return wallVertice;
}

// RETURN VALUE OF ARRAY INDEX
int32 AProceduralTunnel::CalculateArrayIndex() 
{
	switch (surfaceIndex)
	{
		case 0: 
			return loopAroundTunnelCurrentIndex - 1;
			break;
		case 1: 
			return loopAroundTunnelCurrentIndex - (pointsInWidth + splineLoopAdjustment);
			break;
		case 2: 
			return loopAroundTunnelCurrentIndex - (pointsInWidth + pointsInHeight + splineLoopAdjustment);
			break;
		case 3: 
			return loopAroundTunnelCurrentIndex - (pointsInWidth*2 + pointsInHeight +splineLoopAdjustment + 1);
			break;
		default:
			return 0;
			break;
	}
}

// CALCULATE IF ITS FIRST LOOP IN TUNNEL
bool AProceduralTunnel::GetIsFirstLoopAround() 
{
	// Calculate the index of the last spline point in the tunnel mesh.
	int32 lastSplineIndex = SplineComponent->GetNumberOfSplinePoints() - 1;
	int32 meshLastIndex = lastSplineIndex - meshLoopCurrentIndex;

	// Check if this is the first loop around the tunnel.
	bool isFirstLoop = (meshLastIndex == 1 || TunnelMeshes.Num() == 0) && pointCapLoopCurrentIndex == 0;

	return isFirstLoop;
}

// Return the index of the surface we are currently on.
// 0 = Floor, 1 = Right, 2 = Roof, 3 = Left
int32 AProceduralTunnel::GetSurfaceIndex() 
{
	// If under or equal to 26
	if(loopAroundTunnelCurrentIndex <= pointsInWidth + 1) {
		return 0;
	} // If over 26 and under 47
	else if (loopAroundTunnelCurrentIndex > pointsInWidth + 1 && loopAroundTunnelCurrentIndex < pointsInWidth+pointsInHeight+splineLoopAdjustment)
	{
		return 1;
	} // If over or equal to 47 and under or equal to 72
	else if (loopAroundTunnelCurrentIndex >= pointsInWidth+pointsInHeight+splineLoopAdjustment && loopAroundTunnelCurrentIndex <= pointsInWidth*2+pointsInHeight+splineLoopAdjustment) 
	{
		return 2;
	} // Over 72
	else 
	{
		return 3;
	}
}

// SET START VECTOR, RIGHT VECTOR AND X VALUE OF DEFORMLIST 
void AProceduralTunnel::InitializeStartVectorRightVectorAndValueInTexture()
{
	// Calculate start distance and current distance on spline
	int32 splinePointIndex = SplineComponent->GetNumberOfSplinePoints() - (meshLoopCurrentIndex + 2);
	float startDistance = SplineComponent->GetDistanceAlongSplineAtSplinePoint(splinePointIndex); /// DISTANCE ON SPLINE AT START

	/// THIS IS DISTANCE ON SPLINE AT CURRENT LOCATION 
	float currentDistance = (pointCapLoopCurrentIndex == pointCapLoopLastIndex && meshLoopCurrentIndex == meshLoopLastIndex)
		? startDistance + ((float)(pointCapLoopCurrentIndex - 1) * stepSizeOnSpline + lastStepSizeOnSpline)
		: startDistance + (float)pointCapLoopCurrentIndex * stepSizeOnSpline;

	// Calculate distance on spline in steps
	float distanceOnSpline = currentDistance / stepSizeOnSpline; 

	// Calculate the start location on spline, right vector on distance, and forward vector
	startLocationOnSpline = SplineComponent->GetLocationAtDistanceAlongSpline(currentDistance, ESplineCoordinateSpace::Local);
	FVector rightVectorOnDistance = SplineComponent->GetRightVectorAtDistanceAlongSpline(currentDistance, ESplineCoordinateSpace::Local);
	const FRotator rot = FRotator(0.0f, -90, 0.0f);
	FVector result = rot.RotateVector(rightVectorOnDistance);
	forwardVector = result;

	// Calculate how many times we can fit our noise texture on the distance we have moved
	int32 whole = FMath::FloorToFloat(distanceOnSpline / noiseTextureXresolution);

	// Check if it's divisible by two
	float remainder = whole % 2;

	/// MOVE FORWARD ON NOISE TEXTURE
	if (remainder == 0 || whole == 0) {
		forwardStepInDeformTexture = FMath::FloorToInt(distanceOnSpline - (float)FMath::FloorToFloat(distanceOnSpline / noiseTextureXresolution) * noiseTextureXresolution);
	}
	/// MOVE BACKWARD ON NOISE TEXTURE
	else {
		int32 x = FMath::FloorToInt(distanceOnSpline - (float)FMath::FloorToFloat(distanceOnSpline / noiseTextureXresolution) * noiseTextureXresolution);
		forwardStepInDeformTexture = (int32)noiseTextureXresolution - x;
	}


	// Calculate the sideways multiplier
	float sideWaysMultiplier = (float)(pointsInWidth / 2) * groundVerticeSize;											 
	
	// Calculate the x, y, and z values for the latest vertice
	FVector negativeRightVector = rightVectorOnDistance * -1;																			 
	float x = negativeRightVector.X * sideWaysMultiplier;
	float y = negativeRightVector.Y * sideWaysMultiplier;
	float z = negativeRightVector.Z * sideWaysMultiplier;
	latestVertice = startLocationOnSpline + FVector(x, y, z);

	// Set the right vector
	rightVector = FVector(rightVectorOnDistance.X, rightVectorOnDistance.Y, 0.0f);
}

// CLEAR ARRAYS
void AProceduralTunnel::ClearArrays () {
	wallVertices.Empty();
	wallTriangles.Empty();
	wallUV.Empty();
	groundVertices.Empty();
	groundTriangles.Empty();
	groundUV.Empty();
	lastRightVertices.Empty();
	lastRoofVertices.Empty();
	lastLeftVertices.Empty();
	lastFloorVertices.Empty();
}

// SET VALUES
void AProceduralTunnel::SetValues(float selected, bool undo, FVector2D tunnelScale, FVector2D sVariation, bool reset, bool load) 
{
	isSelected = selected;
	isUndo = undo;
	localTunnelScale = tunnelScale;
	widthScale = tunnelScale.X;
	heightScale = tunnelScale.Y;
	surfaceVariation = sVariation;
	floorDeformation = sVariation.X;
	wallDeformation = sVariation.Y;
	isReset = reset;
	isLoad = load;
}


