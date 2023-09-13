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

	if (IsValid(parentIntersection)) {
		numberOfVerticalPoints = parentIntersection->numberOfVerticalPoints;
		numberOfHorizontalPoints = parentIntersection->numberOfHorizontalPoints;
		// -1 because when tunnel round loop is created it starts with index 0
		loopAroundTunnelLastIndex = numberOfVerticalPoints + numberOfHorizontalPoints - 1;
	}

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

// Destroy the last mesh
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


// This will control the addition of new spline points on drag event and remove when undoing or reseting
void AProceduralTunnel::ControlSplinePoints(bool interSectionAdded)
{
	
	if(!isReset) 
	{
		// When end is connected or we are undoing tunnel we dont want to enter here
		if (!isEndConnected && !isUndo)
		{ 
			AddOrRemoveSplinePoints(interSectionAdded);
		}
		// If we are undoing tunnel and tunnel is long enough we will destroy last mesh part of this tunnel
		else if (!isEndConnected && isUndo && SplineComponent->GetNumberOfSplinePoints() >= 2)
		{
			TunnelMeshes.Last()->DestroyComponent();
			TunnelMeshes.RemoveAt(TunnelMeshes.Num() - 1);
			meshEnds.RemoveAt(meshEnds.Num()-1);
		}
	} 
	// When resetting remove all the mesh components so that when this tunnel part is build again it will have default starting size
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

// Adds or removes spline points depending on the distance of dragged spline point and previous spline point
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

// Returns negative value of how many meshes we need to remake
int32 AProceduralTunnel::SetUpProceduralGeneratorLoopParams()
{
	float tunnelWidth = widthScale * 100.0f;
	float tunnelHeight = heightScale * 100.0f;
	horizontalPointSize = tunnelWidth / (float)(numberOfHorizontalPoints - 1);
	verticalPointSize = tunnelHeight / (float)(numberOfVerticalPoints);

	reCreateMeshCount = 2;
	meshInRework = -1;
	int32 countOfMeshesToRemake = FMath::Clamp((SplineComponent->GetNumberOfSplinePoints() -2), 0, 2) + (fmax(0, ((SplineComponent->GetNumberOfSplinePoints() - 2) - TunnelMeshes.Num())));
	return countOfMeshesToRemake * -1;
}

// Regenerate section of tunnel. One section is space between 2 back to back spline points
void AProceduralTunnel::ProceduralGenerationLoop(int32 firstIndex, int32 lastIndex, bool isSinglePointUpdate, bool isIntersectionAdded, IntersectionType interType) {
	// Initialize variables for procedural generation loop
	InitializeProceduralGenerationLoopVariables(firstIndex, lastIndex, interType);

	// Loop through the tunnel sections
	for (int32 index = firstIndex; index <= lastIndex; index++) {
		indexOfCurrentMesh = abs(index);
		ClearArrays();

		// Calulate step count and last step size on selected tunnel section
		CalculateStepsInTunnelSection();

		// Check if there is enough space for at least one step
		if (stepCountToMakeCurrentMesh > 0) {
			// Reset the current mesh end data
			ResetCurrentMeshEndData();

			// Generate vertices and UVs for the tunnel mesh
			GenerateVerticesAndUVs(isSinglePointUpdate, isIntersectionAdded, lastIndex);

			// Create the mesh triangles, tangents, and normals
			MakeMeshTriangles();
			MakeMeshTangentsAndNormals();

			// Build the mesh with the generated data
			MakeMesh(indexOfCurrentMesh, isSinglePointUpdate, isLoad);

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
	indexOfLastMesh = abs(lastIndex);

	// Check for a parent actor and set parentIntersection and parentsParentTunnel accordingly
	if (IsChildActor()) {
		parentIntersection = Cast<AProceduralIntersection>(GetParentActor());
		parentsParentTunnel = Cast<AProceduralTunnel>(parentIntersection->parentTunnel);
	}
}

// Calculate how many steps we can fit between selected tunnel section also calculate how big the last step can be
void AProceduralTunnel::CalculateStepsInTunnelSection() {
	float latestLocation = SplineComponent->GetDistanceAlongSplineAtSplinePoint(SplineComponent->GetNumberOfSplinePoints() - (indexOfCurrentMesh + 1));
	float previousLocation = SplineComponent->GetDistanceAlongSplineAtSplinePoint(SplineComponent->GetNumberOfSplinePoints() - (indexOfCurrentMesh + 2));
	stepCountToMakeCurrentMesh = FMath::Floor((latestLocation - previousLocation) / stepSizeOnSpline);
	float remainder = ((latestLocation - previousLocation) / stepSizeOnSpline) - (float)stepCountToMakeCurrentMesh;
	lastStepSizeOnSpline = stepSizeOnSpline * remainder + stepSizeOnSpline;
}

// Reset the current mesh end data
void AProceduralTunnel::ResetCurrentMeshEndData() {
	currentMeshEndData = FMeshSectionEnd();
}

// Generate vertices and UVs for the tunnel mesh
void AProceduralTunnel::GenerateVerticesAndUVs(bool isSinglePointUpdate, bool isIntersectionAdded, int32 lastIndex) {
	for (stepIndexInsideMesh = 0; stepIndexInsideMesh <= stepCountToMakeCurrentMesh; stepIndexInsideMesh++) {
		// Clear arrays holding vertice data of start of tunnel
		if (IsFirstLoopOfWholeTunnel()) {
			firstFloorVertices.Empty();
			firstRightVertices.Empty();
			firstRoofVertices.Empty();
			firstLeftVertices.Empty();
		}
		// If true we will accuire previous mesh sections end vertice data to make seamless connection between mesh sections 
		if (usePreviousEndVertices(isIntersectionAdded, isSinglePointUpdate)) {
			UsePreviousEndVerticesData(isSinglePointUpdate);
		}
		else {
			// This is default way of creting vertices around the tunnel
			GenerateVerticesForCurrentLoop(isSinglePointUpdate, isIntersectionAdded, lastIndex);
		}
	}
}

// Returns boolean if we should use previous mesh section end data to make seamless connection between sections
bool AProceduralTunnel::usePreviousEndVertices (bool isIntersectionAdded, bool isSinglePointUpdate) {
	int32 meshEndIndex = SplineComponent->GetNumberOfSplinePoints() - 3 - indexOfCurrentMesh;
	// If there is intersection added and we are in the end of current mesh 
	if(isIntersectionAdded && stepIndexInsideMesh == stepCountToMakeCurrentMesh) {
		return false;
	} 
	else 
	{
		// If we are in the first step on creating current mesh section. And there is mesh end we can use
		if((stepIndexInsideMesh == 0) && (meshEnds.Num() > 0) && (meshEndIndex >= 0) && (meshEnds.Num() - 1 >= meshEndIndex)) {
			return true;
		}
		// If we are in the end of last mesh to create and single point adjustment is made
		if(indexOfCurrentMesh == indexOfLastMesh && stepIndexInsideMesh == stepCountToMakeCurrentMesh && isSinglePointUpdate) {
			return true;	
		}
		return false;
	}
}

// Use the previous meshs sections end vertices data for the current tunnel section
void AProceduralTunnel::UsePreviousEndVerticesData(bool isSinglePointUpdate) {
	// Calculate the index that provides the correct mesh end data from the array (previous meshes end)
	int32 meshEndIndex = SplineComponent->GetNumberOfSplinePoints() - 3 - indexOfCurrentMesh; 

	// Check if we are at the end of the last mesh that we are generating and the height is adjusted
	// In this case, we want to use this mesh's previously saved end data
	if (indexOfCurrentMesh == indexOfLastMesh && stepIndexInsideMesh == stepCountToMakeCurrentMesh && isSinglePointUpdate) 
	{																													 
		meshEndIndex = meshEnds.Num() - 1 - indexOfCurrentMesh;
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

// Generate vector locations around the tunnel
void AProceduralTunnel::GenerateVerticesForCurrentLoop(bool isSinglePointUpdate, bool isIntersectionAdded, int32 lastIndex) {
	// Initialize start location and right vector 
	InitializeStartVectorRightVectorAndValueInTexture();

	// Loop through each point in the loop that goes around tunnel
	for (loopAroundTunnelCurrentIndex = 0; loopAroundTunnelCurrentIndex <= loopAroundTunnelLastIndex; loopAroundTunnelCurrentIndex++) {
		// Get the surface index and array index for this point
		surfaceIndex = GetSurfaceIndex();
		// Get index of current vertice. Can be used to retrieve right vertice location from other tunnel in conncetion event
		verticeIndex = GetIndexOfVertice();

		// Check if this is the first loop around and if it's a special tunnel type
		bool isFirstLoopAround = GetIsFirstLoopAround();

		// Get vertice for child tunnel of intersection from the parent intersection when we are in first loop of tunnel. 
		// This is needed to create seamless cap between intersection and child tunnel
		if (isFirstLoopAround && tunnelType != TunnelType::DefaultTunnel) {
			latestVertice = GetVerticeForStartOfChildTunnel();
		}
		// If the tunnel is connected to other tunnel and we are in the end of the tunnel and we are not adjusting singlepoint
		else if (IsOnTheEndOfTunnel() && indexOfLastMesh == 0 && !isSinglePointUpdate && IsValid(connectedActor) && isEndConnected) {
			latestVertice = GetVerticeForConnectedTunnel();
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
		if (IsOnTheEndOfCurrentMeshSection()) {
			SaveEndMeshVerticeData(surfaceIndex, latestVertice);
		}
	}
}

// Returns vertice for tunnel that is child tunnel of intersection.
FVector AProceduralTunnel::GetVerticeForStartOfChildTunnel() {
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

// Returns true if we are in the end of the tunnel
bool AProceduralTunnel::IsOnTheEndOfTunnel() {
	return indexOfCurrentMesh == indexOfLastMesh && IsOnTheEndOfCurrentMeshSection();
}

// Returns true if we are in the end of current mesh section.
bool AProceduralTunnel::IsOnTheEndOfCurrentMeshSection() {
	// Returns true if the current loop index is equal to the last loop index, indicating the end of the current mesh.
	return stepIndexInsideMesh == stepCountToMakeCurrentMesh;
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
		verticeToUse = verticeArrayToUse[verticeIndex];
	}
	else {
		verticeToUse = verticeArrayToUse[FMath::Clamp(verticeArrayToUse.Num() - (1 + verticeIndex), 0, verticeArrayToUse.Num() - 1)];
	}    

	// Transform the vertex from the connected actor's transform to world transform.
    verticeToUse = UKismetMathLibrary::TransformLocation(connectedActor->GetTransform(), verticeToUse); 

	// Transform the vertex from world transform to local transform of this tunnel mesh.
    return UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), verticeToUse); 
}

// Returns the appropriate vertice for a default tunnel segment based on the surface index, whether it's the first loop around the tunnel, and if there's an intersection added.
FVector AProceduralTunnel::GetVerticeForDefaultTunnel(bool isFirstLoopAround, bool isIntersectionAdded) {
	FVector vertice;
	switch (surfaceIndex) {
	case 0:
		vertice = GetVerticeOnGround();
		break;
	case 1:
		vertice = GetVerticeOnRightWall(isFirstLoopAround, isIntersectionAdded);
		break;
	case 2:
		vertice = GetVerticeOnRoof();
		break;
	case 3:
		vertice = GetVerticeOnLeftWall(isFirstLoopAround, isIntersectionAdded);
		break;
	}
	return vertice;
}

// Adjusts the latest vertice position for overlap, ensuring a smooth transition between tunnel segments.
FVector AProceduralTunnel::AdjustLatestVerticeForOverlap(FVector vertice, int32 surface) {
	FVector previousLoopVertice = FVector(0.0f, 0.0f, 0.0f);
	// On the walls/roof
	if (surface != 0) {
		int32 previousVertice = numberOfHorizontalPoints + numberOfVerticalPoints * 2;
		if (wallVertices.Num() - previousVertice >= 0) {
			previousLoopVertice = wallVertices[wallVertices.Num() - previousVertice];
		}
	}
	// On the floor
	else {
		if (groundVertices.Num() - numberOfHorizontalPoints >= 0) {
			previousLoopVertice = groundVertices[groundVertices.Num() - numberOfHorizontalPoints];
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
	if (loopAroundTunnelCurrentIndex == 0) {
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
	// - The spline point index (indexOfCurrentMesh) is one less than the total number of spline points,
	//   or there are no tunnel meshes.
	// - The point cap loop index (stepIndexInsideMesh) is 0.
	return ((SplineComponent->GetNumberOfSplinePoints() - 1) - indexOfCurrentMesh == 1 || TunnelMeshes.Num() == 0) && stepIndexInsideMesh == 0;
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

// Saves the end mesh vertice data based on the provided surface index and vertice position.
void AProceduralTunnel::SaveEndMeshVerticeData(int32 surface, FVector vertice) {
	// Switch on the surface index to determine which part of the tunnel to update.
	switch (surface) {
		case 0: // Ground
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

// Get vertices for intersections child tunnel pointing to right
FVector AProceduralTunnel::RightTunnelStart() 
{
	FVector vertice;
	switch (surfaceIndex)
	{
	case 0:
		vertice = parentIntersection->lastRightFloorVertices[parentIntersection->lastRightFloorVertices.Num() - 1 - verticeIndex];
		return TransformVerticeToLocalSpace(parentIntersection, vertice);
		break;
	case 1:
		vertice = parentsParentTunnel->lastRightVertices[verticeIndex];
		return TransformVerticeToLocalSpace(parentsParentTunnel, vertice);
		break;
	case 2:
		vertice = parentIntersection->lastRightRoofVertices[verticeIndex];
		return TransformVerticeToLocalSpace(parentIntersection, vertice);
		break;
	case 3:
		if(parentIntersection->intersectionType == IntersectionType::RightLeft)
		{
			vertice = parentIntersection->lastRightWallVertices[verticeIndex];
			return TransformVerticeToLocalSpace(parentIntersection, vertice);
			break;
		}
		else 
		{
			return GetVerticeOnLeftWall(true, false);
			break;
		}	
	default:
		return FVector(0,0,0);
		break;
	}
}

// Get vertices for intersections child tunnel pointing to left
FVector AProceduralTunnel::LeftTunnelStart() 
{
	FVector vertice;
	switch (surfaceIndex)
	{
	case 0:
		vertice = parentIntersection->lastLeftFloorVertices[verticeIndex];
		return TransformVerticeToLocalSpace(parentIntersection, vertice);
		break;
	case 1:
		if(parentIntersection->intersectionType == IntersectionType::RightLeft)
		{
			vertice = parentIntersection->lastLeftWallVertices[parentIntersection->lastLeftWallVertices.Num() - (1 + verticeIndex)];
			return TransformVerticeToLocalSpace(parentIntersection, vertice);
			break;
		}
		else 
		{
			return GetVerticeOnRightWall(true, false);;
			break;
		}
	case 2:
		vertice = parentIntersection->lastLeftRoofVertices[parentIntersection->lastLeftRoofVertices.Num() - (1 + verticeIndex)];
		return TransformVerticeToLocalSpace(parentIntersection, vertice);
		break;
	case 3:
		vertice = parentsParentTunnel->lastLeftVertices[verticeIndex];
		return TransformVerticeToLocalSpace(parentsParentTunnel, vertice);
		break;
	default:
		return FVector(0,0,0);
		break;
	}
}

// Get vertices for intersections child tunnel pointing forward
FVector AProceduralTunnel::StraightTunnelStart()
{
	FVector vertice;
	switch (surfaceIndex)
	{
	case 0:
		vertice = parentIntersection->lastStraightFloorVertices[verticeIndex];
		return TransformVerticeToLocalSpace(parentIntersection, vertice);
		break;
	case 1:
		if (IsValid(rightSideTunnel))
		{
			vertice = rightSideTunnel->firstLeftVertices[rightSideTunnel->firstLeftVertices.Num() - (1 + verticeIndex)];
			return TransformVerticeToLocalSpace(rightSideTunnel, vertice);
			break;
		}
		else
		{
			vertice = parentIntersection->lastRightWallVertices[verticeIndex];
			return TransformVerticeToLocalSpace(parentIntersection, vertice);
			break;
		}
	case 2:
		vertice = parentIntersection->lastStraightRoofVertices[verticeIndex];
		return TransformVerticeToLocalSpace(parentIntersection, vertice);
		break;
	case 3:
		if (IsValid(leftSideTunnel))
		{
			vertice = leftSideTunnel->firstRightVertices[leftSideTunnel->firstRightVertices.Num() - (1 + verticeIndex)];
			return TransformVerticeToLocalSpace(leftSideTunnel, vertice);
			break;
		}
		else
		{
			vertice = parentIntersection->lastLeftWallVertices[verticeIndex];
			return TransformVerticeToLocalSpace(parentIntersection, vertice);
			break;
		}
	default:
		return FVector(0, 0, 0);
		break;
	}
}

FVector AProceduralTunnel::TransformVerticeToLocalSpace(AActor* actorFrom, FVector vector) {
	vector = UKismetMathLibrary::TransformLocation(actorFrom->GetTransform(), vector); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
	vector = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vector);
	return vector;
}

// Get vertice for ground 
FVector AProceduralTunnel::GetVerticeOnGround()
{
	if (loopAroundTunnelCurrentIndex == 0)
	{
		wallStartVertice = latestVertice;
		return latestVertice;
	}

	// Calculate the deformation amount
	float deformAmount = GetPixelValue(forwardStepInDeformTexture, loopAroundTunnelCurrentIndex);
	float amountOfDeform = FMath::Lerp(0.0f, deformAmount, floorDeformation);
	float deform = FMath::RandRange(amountOfDeform * -20, amountOfDeform * 20);

	// Add sideways movement to start location
	float stepSize = horizontalPointSize * (float)loopAroundTunnelCurrentIndex;
	FVector stepToSide = wallStartVertice + rightVector * stepSize;
	return FVector(stepToSide.X, stepToSide.Y, stepToSide.Z - deform);
}

// Get vertice for right wall
FVector AProceduralTunnel::GetVerticeOnRightWall(bool isFirstLoopARound, bool isIntersectionAdded)
{
	bool isEndOrStar = false;

	// Right wall have to start from where floor ends to prevent any cap between floor and wall meshes
	if (loopAroundTunnelCurrentIndex == numberOfHorizontalPoints)
	{
		// Save the starting vertice when entering first time
		wallStartVertice = latestVertice;
		return latestVertice;
	}

	FVector rVector = rightVector;
	float extraMovementToEnd = 0.0f;

	// Check if we need to rotate the starting position to align with the intersection.
	bool isUnderFiveSteps = stepIndexInsideMesh <= 4;
	bool isStraightTunnelAfterLeftIntersection = tunnelType == TunnelType::StraightTunnel && parentIntersection->intersectionType == IntersectionType::Left;
	bool isLeftTunnelAfterRightLeftIntersection = tunnelType == TunnelType::LeftTunnel && parentIntersection->intersectionType == IntersectionType::RightLeft;
	int32 defaultSplinePointCount = 2; // When tunnel is added it has 2 spline points as default
	bool isFirstMesh = SplineComponent->GetNumberOfSplinePoints() - indexOfCurrentMesh - defaultSplinePointCount == 0; // This should be 2 - 0 - 2 = 0
	
	if (tunnelType != TunnelType::DefaultTunnel && 
		isUnderFiveSteps && 
		!isStraightTunnelAfterLeftIntersection &&
		!isLeftTunnelAfterRightLeftIntersection &&
		(isFirstMesh || TunnelMeshes.Num() == 0))
	{
		isEndOrStar = true;
		float rotateAmount = 0.0f;
		float alpha = stepIndexInsideMesh / 4.0f;
		rotateAmount = FMath::Lerp(-45.0f, 0.0f, alpha);
		switch (stepIndexInsideMesh) {
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
	if (indexOfLastMesh == indexOfCurrentMesh && 
		stepIndexInsideMesh <= stepCountToMakeCurrentMesh && 
		stepIndexInsideMesh >= stepCountToMakeCurrentMesh - 4 && 
		isIntersectionAdded && 
		(intersectionType == IntersectionType::Right || 
			intersectionType == IntersectionType::All || 
			intersectionType == IntersectionType::RightLeft))
	{
		isEndOrStar = true;
		float rotateAmount = 0.0f;
		int index = stepIndexInsideMesh - stepCountToMakeCurrentMesh + 4; // 0-4
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
	float locationOnWall = (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints) / (float)(numberOfVerticalPoints);

	float wVerticeSize = verticalPointSize;

	// Adjust the vertice size for the first loop around 
	if (isFirstLoopARound && IsValid(parentIntersection))
	{
		wVerticeSize = parentIntersection->verticalPointSize;
	}


	// Add roundness to the tunnel.
	float roundnessAmount = deformCurve->GetFloatValue(locationOnWall);
	FVector tunnelRoundnesss = rVector * (FMath::Lerp((tunnelRoundValue + extraMovementToEnd), 0.0f, roundnessAmount));

	float zLocation = (loopAroundTunnelCurrentIndex - numberOfHorizontalPoints) * wVerticeSize;
	FVector wallVertice = wallStartVertice + FVector(0.0f, 0.0f, zLocation);
	wallVertice += tunnelRoundnesss;

	// Dont add deformation to end or start of tunnel
	if (!isEndOrStar) {
		float deformAmount = GetPixelValue(forwardStepInDeformTexture, loopAroundTunnelCurrentIndex);
		wallVertice += FMath::Lerp(0.0f, maxDeform, wallDeformation) * deformAmount * rightVector;
	}

	return wallVertice;
}

// Get vertice for roof
FVector AProceduralTunnel::GetVerticeOnRoof()
{
	// Save start vertice when entering first time
	if (loopAroundTunnelCurrentIndex == numberOfHorizontalPoints + numberOfVerticalPoints)
	{
		wallStartVertice.Z += verticalPointSize * numberOfVerticalPoints;
		return wallStartVertice;
	}

	FVector wallVertice = wallStartVertice;

    float sideWaysMovementSize = (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints - numberOfVerticalPoints) * -horizontalPointSize;
	wallVertice += rightVector * sideWaysMovementSize;

    // Apply roundness to the starting location
    float roundingIndex = (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints - numberOfVerticalPoints) / (float)(numberOfHorizontalPoints - 1);
    float roundnessAmount = deformCurve->GetFloatValue(roundingIndex);
    float tunnelRounding = FMath::Lerp(tunnelRoundValue, 0.0f, roundnessAmount);
	// Divide rounding by 2 to add more natural roundness to roof
	wallVertice.Z += tunnelRounding / 2.0f;

    // Apply deformation to the starting location
    float deformAmount = GetPixelValue(forwardStepInDeformTexture, loopAroundTunnelCurrentIndex);
	wallVertice.Z += FMath::Lerp(0.0f, maxDeform, wallDeformation) * deformAmount;

    return wallVertice;
}

// Get vertice for left wall
FVector AProceduralTunnel::GetVerticeOnLeftWall(bool isFirstLoopARound, bool isIntersectionAdded)
{
	bool isEndOrStar = false;
	float extraMovementToEnd = 0.0f;
	FVector rVector = rightVector;
	float wVerticeSize = verticalPointSize;

	// If we have completed a full loop around the tunnel, return the first vertice.
	// This is to create seamless wall and ground connection
	if (loopAroundTunnelCurrentIndex == loopAroundTunnelLastIndex)
	{
		return firstVertice;
	}

	// Save the starting vertice when entering first time
	if (loopAroundTunnelCurrentIndex == numberOfHorizontalPoints * 2 + numberOfVerticalPoints)
	{
		wallStartVertice = latestVertice;
	}
	

	// If this is the first loop around the tunnel and a parent intersection is valid, use the parent's wall vertice size.
	if (isFirstLoopARound && IsValid(parentIntersection))
	{
		wVerticeSize = parentIntersection->verticalPointSize;
	}

	bool isStraightTunnelAfterRightIntersection = tunnelType == TunnelType::StraightTunnel && parentIntersection->intersectionType == IntersectionType::Right;
	bool isRightTunnelAfterRightLeftIntersection = tunnelType == TunnelType::RightTunnel && parentIntersection->intersectionType == IntersectionType::RightLeft;

	// Rotate start of wall vertices to align with parent of intersection.
	if (tunnelType != TunnelType::DefaultTunnel && stepIndexInsideMesh <= 4 && 
		!isStraightTunnelAfterRightIntersection &&
		!isRightTunnelAfterRightLeftIntersection &&
		((SplineComponent->GetNumberOfSplinePoints() - 1 - indexOfCurrentMesh == 1) || TunnelMeshes.Num() == 0))
	{
		if ((tunnelType == TunnelType::StraightTunnel && IsValid(leftSideTunnel)) || tunnelType != TunnelType::StraightTunnel)
		{
			isEndOrStar = true;
			wallStartVertice = FVector(firstVertice.X, firstVertice.Y, firstVertice.Z + ((float)(numberOfVerticalPoints)*verticalPointSize));


			float rotateAmount = 0.0f;
			float alpha = stepIndexInsideMesh / 4.0f;
			rotateAmount = FMath::Lerp(45.0f, 0.0f, alpha);

			switch (stepIndexInsideMesh) {
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
	if (indexOfLastMesh == indexOfCurrentMesh && 
		stepIndexInsideMesh <= stepCountToMakeCurrentMesh && 
		stepIndexInsideMesh >= stepCountToMakeCurrentMesh - 4 && 
		isIntersectionAdded &&
		(intersectionType == IntersectionType::Left || 
			intersectionType == IntersectionType::All ||
			intersectionType == IntersectionType::RightLeft))
	{
		isEndOrStar = true;

		float rotateAmount = 0.0f;
		int index = stepIndexInsideMesh - stepCountToMakeCurrentMesh + 4; // 0-4
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
	float locationOnWall = (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints * 2 - numberOfVerticalPoints + 1) / (float)numberOfVerticalPoints;
	// Add roundness to the tunnel wall

	float roundnessAmount = deformCurve->GetFloatValue(locationOnWall);
	if (isIntersectionAdded && stepIndexInsideMesh == stepCountToMakeCurrentMesh)
	{
		roundnessAmount = FMath::Clamp((float)(roundnessAmount - 0.1f), 0.0f, 1.0f);
	}
	FVector tunnelRoundnesss = rVector * FMath::Lerp((tunnelRoundValue + extraMovementToEnd) * -1, 0.0f, roundnessAmount); // AMOUNT WE MOVE TO SIDE TO CREATE  THAT ROUND LOOK OF TUNNEL
	float zLocation = (loopAroundTunnelCurrentIndex - numberOfHorizontalPoints * 2 - numberOfVerticalPoints + 1) * (wVerticeSize * -1.0f);


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

// Returns index of current vertice. This index can be used for getting other tunnels correct index in connection situation
int32 AProceduralTunnel::GetIndexOfVertice() 
{
	switch (surfaceIndex)
	{
		case 0: 
			return loopAroundTunnelCurrentIndex;
			break;
		case 1: 
			return loopAroundTunnelCurrentIndex - numberOfHorizontalPoints;
			break;
		case 2: 
			return loopAroundTunnelCurrentIndex - numberOfHorizontalPoints - numberOfVerticalPoints;
			break;
		case 3: 
			return loopAroundTunnelCurrentIndex - numberOfHorizontalPoints * 2 - numberOfVerticalPoints;
			break;
		default:
			return 0;
			break;
	}
}

// Returns true if this is first loop around tunnel
bool AProceduralTunnel::GetIsFirstLoopAround() 
{
	// Calculate the index of the last spline point in the tunnel mesh.
	int32 lastSplineIndex = SplineComponent->GetNumberOfSplinePoints() - 1;
	int32 meshLastIndex = lastSplineIndex - indexOfCurrentMesh;

	// Check if this is the first loop around the tunnel.
	bool isFirstLoop = (meshLastIndex == 1 || TunnelMeshes.Num() == 0) && stepIndexInsideMesh == 0;

	return isFirstLoop;
}

// Return the index of the surface we are currently on.
// 0 = Floor, 1 = Right, 2 = Roof, 3 = Left
int32 AProceduralTunnel::GetSurfaceIndex() 
{
	if(loopAroundTunnelCurrentIndex < numberOfHorizontalPoints) {
		return 0;
	} 
	else if (loopAroundTunnelCurrentIndex < numberOfHorizontalPoints + numberOfVerticalPoints)
	{
		return 1;
	}
	else if (loopAroundTunnelCurrentIndex < numberOfHorizontalPoints * 2 + numberOfVerticalPoints)
	{
		return 2;
	}
	else 
	{
		return 3;
	}
}

// Set start vector location, right vector and x step value for deformationlist
void AProceduralTunnel::InitializeStartVectorRightVectorAndValueInTexture()
{
	// Get spline point index from where our curren mesh section starts
	int32 splinePointIndex = SplineComponent->GetNumberOfSplinePoints() - (indexOfCurrentMesh + 2);
	// Distance on spline at start spline point
	float startDistance = SplineComponent->GetDistanceAlongSplineAtSplinePoint(splinePointIndex); 

	/// Get current distance on spline. If at the end of tunnel last step is taken in lastStepSizeOnSpline because it can differ from normal step size
	float currentDistance = IsOnTheEndOfTunnel()
		? startDistance + ((float)(stepIndexInsideMesh - 1) * stepSizeOnSpline + lastStepSizeOnSpline)
		: startDistance + (float)stepIndexInsideMesh * stepSizeOnSpline;

	// Calculate distance on spline in steps
	float distanceOnSpline = currentDistance / stepSizeOnSpline; 

	// Get location in local space at current distance
	startLocationOnSpline = SplineComponent->GetLocationAtDistanceAlongSpline(currentDistance, ESplineCoordinateSpace::Local);
	// Get right vector in local space at current distance
	FVector rightVectorOnDistance = SplineComponent->GetRightVectorAtDistanceAlongSpline(currentDistance, ESplineCoordinateSpace::Local);
	// Rotatotr to get forward vector from right vector
	const FRotator rot = FRotator(0.0f, -90, 0.0f);
	// Get forward vector from right vector
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
	float sideWaysMultiplier = (float)(numberOfHorizontalPoints - 1) / 2.0f * horizontalPointSize;
	
	// Get vector used as first vertice when generating tunnel
	FVector negativeRightVector = rightVectorOnDistance * -1;																			 
	float x = negativeRightVector.X * sideWaysMultiplier;
	float y = negativeRightVector.Y * sideWaysMultiplier;
	float z = negativeRightVector.Z * sideWaysMultiplier;
	latestVertice = startLocationOnSpline + FVector(x, y, z);

	// Set the right vector
	rightVector = FVector(rightVectorOnDistance.X, rightVectorOnDistance.Y, 0.0f);
}

// Clear arrays that hold data for generating tunnel
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

// These are values used when tunnel construction is called
void AProceduralTunnel::SetValuesForGeneratingTunnel(float selected, bool undo, FVector2D tunnelScale, FVector2D sVariation, bool reset, bool load) 
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


