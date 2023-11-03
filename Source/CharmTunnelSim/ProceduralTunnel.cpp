

#include "ProceduralTunnel.h"
#include "ProceduralIntersection.h"
#include "Components/SplineComponent.h"
#include "Math/Vector.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"
#include "Math/UnrealMathUtility.h"
#include "Components/StaticMeshComponent.h"
#include "Math/UnrealMathVectorCommon.h"
#include "Algo/Reverse.h"

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
	else {
		numberOfVerticalPoints = 20;
		numberOfHorizontalPoints = 20;
		// -1 because when tunnel round loop is created it starts with index 0
		loopAroundTunnelLastIndex = 80 - 1;
	}

	RootComponent = CreateDefaultSubobject<USceneComponent>("Root Scene Component");
	RootComponent->SetMobility(EComponentMobility::Static);
	SetRootComponent(RootComponent);
	SplineComponent = CreateDefaultSubobject<USplineComponent>("Spline Component");
	SplineComponent->SetupAttachment(RootComponent);

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
	if (IsValid(connectedActor)) {
		connectedActor->isEndConnected = false;
		connectedActor = nullptr;
	}
	if (SplineComponent->GetNumberOfSplinePoints() > 2) {
		SplineComponent->RemoveSplinePoint(SplineComponent->GetNumberOfSplinePoints() - 1, true);
		TunnelMeshes.Last()->DestroyComponent();
		TunnelMeshes.RemoveAt(TunnelMeshes.Num() - 1);
		meshEnds.RemoveAt(meshEnds.Num() - 1);
	}
	else {
		SplineComponent->SetLocationAtSplinePoint(1, FVector(200, 0, 0), ESplineCoordinateSpace::Local, true);
	}
}

// Constants for snapping behavior. 
const float DEFAULT_SNAPPING_DISTANCE = 600.0f;
const float LERP_FACTOR = 0.2f;

void AProceduralTunnel::SnapToEndOfOtherSpline()
{
	// Variables to store the closest tunnel details
	FVector closestPoint = FVector::ZeroVector;
	FVector closestTangent;
	AProceduralTunnel* closestTunnel = nullptr;
	bool connectToStart = false;

	// Get the end spline point of the current tunnel
	int32 lastIndex = SplineComponent->GetNumberOfSplinePoints() - 1;
	FVector lastPointLocation = SplineComponent->GetLocationAtSplinePoint(lastIndex, ESplineCoordinateSpace::World);

	// Retrieve all tunnel actors in the world
	TArray<AActor*> FoundActors;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), AProceduralTunnel::StaticClass(), FoundActors);

	// Initialize to the maximum allowed distance to find the closest tunnel within range.
	float shortestDistance = maxDistanceToSnapSplineEnds;

	// Loop through all tunnels to find the closest valid one to connect to
	for (AActor* tunnelActor : FoundActors)
	{
		// Safely cast the actor to AProceduralTunnel
		AProceduralTunnel* tunnel = Cast<AProceduralTunnel>(tunnelActor);

		// Skip if casting failed or if we're checking the current tunnel itself
		if (!tunnel || tunnel->SplineComponent == SplineComponent) continue;

		// Determine which point of the tunnel to consider for snapping
		USplineComponent* spline = tunnel->SplineComponent;
		int32 splineIndexToSnap = (tunnel->tunnelType == TunnelType::StartTunnel) ? 0 : spline->GetNumberOfSplinePoints() - 1;

		// Compute the distance and direction to the potential connecting point
		FVector comparedPointLocation = spline->GetLocationAtSplinePoint(splineIndexToSnap, ESplineCoordinateSpace::World);
		float distance = FVector::Distance(comparedPointLocation, lastPointLocation);
		FVector currentEndDirection = SplineComponent->GetDirectionAtSplinePoint(lastIndex, ESplineCoordinateSpace::World);
		FVector directionToOtherEnd = (comparedPointLocation - lastPointLocation).GetSafeNormal();
		float dotProduct = FVector::DotProduct(currentEndDirection, directionToOtherEnd);

		// Update the closest tunnel details if this tunnel is a better match
		if (distance < shortestDistance && dotProduct >= 0)
		{
			closestPoint = comparedPointLocation;
			closestTangent = spline->GetTangentAtSplinePoint(splineIndexToSnap, ESplineCoordinateSpace::World);
			closestTunnel = tunnel;
			connectToStart = (splineIndexToSnap == 0);
			shortestDistance = distance;  // Update the shortest distance found
		}
	}

	// If we've found a tunnel to connect to
	if (closestTunnel)
	{
		// Update connection status and adjust spline points for the snap
		isEndConnected = true;
		connectedActor = closestTunnel;
		closestTunnel->isEndConnected = true;
		SplineComponent->SetLocationAtSplinePoint(lastIndex, closestPoint, ESplineCoordinateSpace::World, true);

		// Adjust the tangent to ensure a smooth connection
		FVector adjustedTangent = connectToStart ? closestTangent : -closestTangent;
		SplineComponent->SetTangentAtSplinePoint(lastIndex, adjustedTangent, ESplineCoordinateSpace::World, true);

		// Additional tweak to the penultimate point for a smoother transition
		FVector direction = SplineComponent->GetDirectionAtSplinePoint(lastIndex, ESplineCoordinateSpace::World);
		FVector secondPointLocation = SplineComponent->GetLocationAtSplinePoint(lastIndex - 1, ESplineCoordinateSpace::World);
		if (SplineComponent->GetNumberOfSplinePoints() > 2)
		{
			FVector adjustedLocation = FMath::Lerp(closestPoint - direction * DEFAULT_SNAPPING_DISTANCE, secondPointLocation, LERP_FACTOR);
			SplineComponent->SetLocationAtSplinePoint(lastIndex - 1, adjustedLocation, ESplineCoordinateSpace::World, true);
		}
	}
	else  // No suitable tunnel was found
	{
		connectedActor = nullptr;
		isEndConnected = false;
	}
}

// This will control the addition of new spline points on drag event and remove when undoing or reseting
void AProceduralTunnel::ControlSplinePoints()
{
	if(!isReset) 
	{
		if (!isUndo)
		{ 
			AddOrRemoveSplinePoints();
		}
		// If we are undoing destroy last mesh part of this tunnel
		else 
		{
			DestroyLastMesh();
		}
	} 
	// When resetting remove all the mesh components so the whole tunnel is rebuild
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

void AProceduralTunnel::AddOrRemoveSplinePoints()
{
	// Define the base parameters for point spacing and offset
	float maxDistanceBetweenPoints = 750.0f;
	float pointOffSet = 200.0f;

	// Calculate the distances for last and second last spline points
	int32 numberOfPoints = SplineComponent->GetNumberOfSplinePoints();
	int32 lastIndex = numberOfPoints - 1;
	float currentDistance = SplineComponent->GetDistanceAlongSplineAtSplinePoint(lastIndex);
	float previousDistance = SplineComponent->GetDistanceAlongSplineAtSplinePoint(lastIndex - 1);
	float distanceBetweenPoints = currentDistance - previousDistance;

	// Calculate how many points can fit in the gap
	int32 pointsToFitBetween = FMath::FloorToInt(distanceBetweenPoints / maxDistanceBetweenPoints);

	FVector SecondPointForward = SplineComponent->GetDirectionAtSplinePoint(lastIndex, ESplineCoordinateSpace::World);
	FVector FirstPointForward = SplineComponent->GetDirectionAtSplinePoint(lastIndex - 1, ESplineCoordinateSpace::World);
	float DotProduct = FVector::DotProduct(SecondPointForward, FirstPointForward);

	if (DotProduct < 0.0f)
	{
		// Remove the second last spline point and associated tunnel mesh and mesh end
		SplineComponent->RemoveSplinePoint(numberOfPoints - 2);

		// Safety check before destruction and removal
		if (TunnelMeshes.Num() > 0)
		{
			TunnelMeshes.Last()->DestroyComponent();
			TunnelMeshes.RemoveAt(TunnelMeshes.Num() - 1);
		}

		// Safety check before removal
		if (meshEnds.Num() > 0)
		{
			meshEnds.RemoveAt(meshEnds.Num() - 1);
		}
	}
	// If there's space for more points between the last two points
	else if (pointsToFitBetween > 0)
	{
		for (int32 i = 1; i <= pointsToFitBetween; i++)
		{
			// Previous spline point index
			int32 splinePointIndex = SplineComponent->GetNumberOfSplinePoints() - 2;

			// Calculate the location for the new spline point
			float distance = SplineComponent->GetDistanceAlongSplineAtSplinePoint(splinePointIndex) + maxDistanceBetweenPoints;

			// Adjust the distance for the last new point
			if (i == pointsToFitBetween)
			{
				distance -= pointOffSet;
			}

			FVector location = SplineComponent->GetLocationAtDistanceAlongSpline(distance, ESplineCoordinateSpace::World);
			SplineComponent->AddSplinePointAtIndex(location, splinePointIndex + 1, ESplineCoordinateSpace::World, true);
		}
	}
	// If the distance between the last two points is too small and there are more than three points in total
	else if (distanceBetweenPoints < pointOffSet && numberOfPoints > 3)
	{
		// Remove the second last spline point and associated tunnel mesh and mesh end
		SplineComponent->RemoveSplinePoint(numberOfPoints - 2);

		// Safety check before destruction and removal
		if (TunnelMeshes.Num() > 0)
		{
			TunnelMeshes.Last()->DestroyComponent();
			TunnelMeshes.RemoveAt(TunnelMeshes.Num() - 1);
		}

		// Safety check before removal
		if (meshEnds.Num() > 0)
		{
			meshEnds.RemoveAt(meshEnds.Num() - 1);
		}
	}
}


int32 AProceduralTunnel::CalculateRecreationStartIndex()
{
	float tunnelWidth = widthScale * 100.0f;
	float tunnelHeight = heightScale * 100.0f;
	horizontalPointSize = tunnelWidth / (float)(numberOfHorizontalPoints - 1);
	//UE_LOG(LogTemp, Warning, TEXT("The float value is: %f"), horizontalPointSize);
	verticalPointSize = tunnelHeight / (float)(numberOfVerticalPoints);

	// Last mesh index is alway spline point count - 2
	int32 lastMeshIndex = SplineComponent->GetNumberOfSplinePoints() - 2;
	// This gives as base mesh index from which to start recreating tunnel meshes
	// 0 means that we only want to recreate last mesh of tunnel
	// But we also want to clamp base value to be in range of 0 - 2
	// Draging the end of tunnel does not affect meshes after third mesh (index 2)
	int32 baseMeshIndex = FMath::Clamp(lastMeshIndex, 0, 2);
	// This gives us extra offset if user has clicked tunnel end further 
	// by clicking user can move tunnel end far away from original location
	// by doing this there is much more caps between spline points where is no mesh created
	// below example of this kind of situation where x = spline point # = mesh and ? = missing mesh
	// 
	//  Start	  6   5   4   3   2   1   0 End
	//  x # x # x # x # x ? x ? x ? x ? x ? x
	//							  ^ baseMeshIndex = 2
	//            ^ baseMeshIndex(2) + tunnelMeshOffset(4) = 6 
	//
	int32 tunnelMeshOffset = fmax(0, (lastMeshIndex - TunnelMeshes.Num()));
	
	int32 indexOfMeshFromStartRecreation = baseMeshIndex + tunnelMeshOffset;
	// This needs to be returned as negative because recreation loop is done counterwise 
	// Check the visualization above 
	indexOfMeshFromStartRecreation *= -1;

	return indexOfMeshFromStartRecreation;
}

// Regenerate section of tunnel. One section is space between 2 back to back spline points
void AProceduralTunnel::ProceduralGenerationLoop(int32 firstIndex, int32 lastIndex, bool isMeshPartUpdate) {
	// Initialize variables for procedural generation loop
	InitializeProceduralGenerationLoopVariables(firstIndex, lastIndex);

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
			GenerateVerticesAndUVs(isMeshPartUpdate, lastIndex);

			// Create the mesh triangles, tangents, and normals
			MakeMeshTriangles();
			MakeMeshTangentsAndNormals();

			// Build the mesh with the generated data
			MakeMesh(indexOfCurrentMesh);
		}
	}
}

// Initialize variables required for the procedural generation loop
void AProceduralTunnel::InitializeProceduralGenerationLoopVariables(int32 firstIndex, int32 lastIndex) {
	meshLoopFirstIndex = abs(firstIndex);
	indexOfLastMesh = abs(lastIndex);

	if (IsValid(parentIntersection)) {
		parentIntersection = Cast<AProceduralIntersection>(GetParentActor());
		if (IsValid(parentIntersection->parentTunnel)) {
			parentsParentTunnel = Cast<AProceduralTunnel>(parentIntersection->parentTunnel);
		}
	}
}

// Calculate how many steps we can fit between selected tunnel section also calculate how big the last step can be
void AProceduralTunnel::CalculateStepsInTunnelSection() {
	// End spline point distance on spline
	float locationOnSplineWhereMeshEnds = SplineComponent->GetDistanceAlongSplineAtSplinePoint(SplineComponent->GetNumberOfSplinePoints() - (indexOfCurrentMesh + 1));
	// Starting spline point distance on spline
	float locationOnSplineWhereMeshStarts = SplineComponent->GetDistanceAlongSplineAtSplinePoint(SplineComponent->GetNumberOfSplinePoints() - (indexOfCurrentMesh + 2));
	stepCountToMakeCurrentMesh = FMath::Floor((locationOnSplineWhereMeshEnds - locationOnSplineWhereMeshStarts) / horizontalPointSize);
	float remainder = ((locationOnSplineWhereMeshEnds - locationOnSplineWhereMeshStarts) / horizontalPointSize) - (float)stepCountToMakeCurrentMesh;
	lastStepSizeOnSpline = horizontalPointSize * remainder + horizontalPointSize;
}

// Reset the current mesh end data
void AProceduralTunnel::ResetCurrentMeshEndData() {
	currentMeshEndData = FMeshSectionEnd();
}

// Generate vertices and UVs for the tunnel mesh
void AProceduralTunnel::GenerateVerticesAndUVs(bool isMeshPartUpdate, int32 lastIndex) {
	for (stepIndexInsideMesh = 0; stepIndexInsideMesh <= stepCountToMakeCurrentMesh; stepIndexInsideMesh++) {
		// Clear arrays holding vertice data of start of tunnel
		if (IsFirstLoopOfWholeTunnel()) {
			tunnelStartMeshData.Reset();
		}
		// If true we will accuire previous mesh sections end vertice data to make seamless connection between mesh sections 
		if (ShouldUseMeshEndData(isMeshPartUpdate)) {
			GetMeshEndData(isMeshPartUpdate);
		} 
		else if (IsOnTheEndOfTunnel() && indexOfLastMesh == 0 && !isMeshPartUpdate && IsValid(connectedActor) && isEndConnected) {
			if (connectedActor->tunnelType == TunnelType::StartTunnel) {
				TArray<FVector> ground = TransformVectors(connectedActor->tunnelStartMeshData.GroundVertives, connectedActor, this);
				TArray<FVector> walls = TransformVectors(connectedActor->tunnelStartMeshData.WallVertices, connectedActor, this);
				groundVertices.Append(ground);
				wallVertices.Append(walls);
				currentMeshEndData.GroundVertives = ground;
				currentMeshEndData.WallVertices = walls;
			}
			else {
				TArray<FVector> ground = TransformVectors(connectedActor->meshEnds[connectedActor->meshEnds.Num()-1].GroundVertives, connectedActor, this);
				TArray<FVector> walls = TransformVectors(connectedActor->meshEnds[connectedActor->meshEnds.Num() - 1].WallVertices, connectedActor, this);
				Algo::Reverse(ground);
				Algo::Reverse(walls);
				groundVertices.Append(ground);
				wallVertices.Append(walls);
				currentMeshEndData.GroundVertives = ground; 
				currentMeshEndData.WallVertices = walls;
			}
		}
		else {
			// This is default way of creting vertices around the tunnel
			GenerateVerticesForCurrentLoop(lastIndex);
		}
	}
}

// Transforms mesh end data vectors from other actors local space to other actor local space
TArray<FVector> AProceduralTunnel::TransformVectors(const TArray<FVector>& Vectors, AActor* SourceActor, AActor* TargetActor)
{
	TArray<FVector> TransformedVectors;

	if (!SourceActor || !TargetActor)
	{
		// Log an error or handle this case as per your needs.
		UE_LOG(LogTemp, Error, TEXT("Source or Target actor is null!"));
		return TransformedVectors; // Return an empty array.
	}

	// Get the transform of the source and target actors.
	FTransform SourceTransform = SourceActor->GetTransform();
	FTransform TargetTransform = TargetActor->GetTransform();

	// Reserve space in TransformedVectors to avoid reallocations.
	TransformedVectors.Reserve(Vectors.Num());

	// Loop through each vector in the input array.
	for (const FVector& Vec : Vectors)
	{
		// Transform the vertex from the connected actor's transform to world transform.
		FVector WorldVec = UKismetMathLibrary::TransformLocation(SourceTransform, Vec);

		// Transform the vertex from world transform to local transform of this tunnel mesh.
		FVector TargetLocalVec = UKismetMathLibrary::InverseTransformLocation(TargetTransform, WorldVec);

		// Add the transformed vector to the output array.
		TransformedVectors.Add(TargetLocalVec);
	}

	return TransformedVectors;
}

// Returns boolean if we should use previous mesh section end data to make seamless connection between sections
bool AProceduralTunnel::ShouldUseMeshEndData (bool isMeshPartUpdate) {
	int32 meshEndIndex = SplineComponent->GetNumberOfSplinePoints() - 3 - indexOfCurrentMesh;
	// If there is intersection added and we are in the end of current mesh 
	// return false because we want to recreate end before connecting with intersection
	if(IsValid(intersection) && stepIndexInsideMesh == stepCountToMakeCurrentMesh && indexOfCurrentMesh == 0) {
		return false;
	} 
	else 
	{
		// If we are in the first step on creating current mesh section. And there is mesh end we can use
		if((stepIndexInsideMesh == 0) && (meshEnds.Num() > 0) && (meshEndIndex >= 0) && (meshEnds.Num() - 1 >= meshEndIndex)) {
			return true;
		}
		// If we are in the end of last mesh to create and mesh part is udpated
		if(indexOfCurrentMesh == indexOfLastMesh && stepIndexInsideMesh == stepCountToMakeCurrentMesh && isMeshPartUpdate) {
			return true;	
		}
		return false;
	}
}

// Use the previous meshs sections end vertices data for the current tunnel section
void AProceduralTunnel::GetMeshEndData(bool isMeshPartUpdate) {
	// Calculate the index that provides the correct mesh end data from the array (previous meshes end)
	int32 meshEndIndex = SplineComponent->GetNumberOfSplinePoints() - 3 - indexOfCurrentMesh; 

	// Check if we are at the end of the last mesh that we are generating and the height is adjusted
	// In this case, we want to use this mesh's previously saved end data
	if (indexOfCurrentMesh == indexOfLastMesh && stepIndexInsideMesh == stepCountToMakeCurrentMesh && isMeshPartUpdate) 
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
void AProceduralTunnel::GenerateVerticesForCurrentLoop(int32 lastIndex) {
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
		if (isFirstLoopAround && tunnelType != TunnelType::StartTunnel) {
			latestVertice = GetVerticeForStartOfChildTunnel();
		}
		else {
			// Get the vertice for a default tunnel
			latestVertice = GetVerticeForDefaultTunnel(isFirstLoopAround);
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

// Returns the appropriate vertice for a default tunnel segment based on the surface index, whether it's the first loop around the tunnel, and if there's an intersection added.
FVector AProceduralTunnel::GetVerticeForDefaultTunnel(bool isFirstLoopAround) {
	FVector vertice;
	switch (surfaceIndex) {
	case 0:
		vertice = GetVerticeOnGround();
		break;
	case 1:
		vertice = GetVerticeOnRightWall(isFirstLoopAround);
		break;
	case 2:
		vertice = GetVerticeOnRoof();
		break;
	case 3:
		vertice = GetVerticeOnLeftWall(isFirstLoopAround);
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
	if (surface == 0) {
		tunnelStartMeshData.GroundVertives.Add(vertice);
	}
	else {
		tunnelStartMeshData.WallVertices.Add(vertice);
	}
}

// Saves the end mesh vertice data based on the provided surface index and vertice position.
void AProceduralTunnel::SaveEndMeshVerticeData(int32 surface, FVector vertice) {
	// Switch on the surface index to determine which part of the tunnel to update.
	if (surface == 0) {
		currentMeshEndData.GroundVertives.Add(vertice);
	}
	else {
		currentMeshEndData.WallVertices.Add(vertice);
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
		if (IsValid(parentsParentTunnel)) {
			// Get data of last mesh end
			vertice = parentsParentTunnel->meshEnds[parentsParentTunnel->meshEnds.Num() - 1].WallVertices[verticeIndex];
			return TransformVerticeToLocalSpace(parentsParentTunnel, vertice);
		}
		else {
			return GetVerticeOnRightWall(true);
		}
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
			return GetVerticeOnLeftWall(true);
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
			return GetVerticeOnRightWall(true);;
			break;
		}
	case 2:
		vertice = parentIntersection->lastLeftRoofVertices[parentIntersection->lastLeftRoofVertices.Num() - (1 + verticeIndex)];
		return TransformVerticeToLocalSpace(parentIntersection, vertice);
		break;
	case 3:
		if (IsValid(parentsParentTunnel)) {
			vertice = parentsParentTunnel->meshEnds[parentsParentTunnel->meshEnds.Num() - 1].WallVertices[loopAroundTunnelCurrentIndex - numberOfHorizontalPoints];
			return TransformVerticeToLocalSpace(parentsParentTunnel, vertice);
		}
		else {
			return GetVerticeOnLeftWall(true);
		}
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
			vertice = rightSideTunnel->tunnelStartMeshData.WallVertices[rightSideTunnel->tunnelStartMeshData.WallVertices.Num() - (1 + verticeIndex)];
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
			vertice = leftSideTunnel->tunnelStartMeshData.WallVertices[numberOfVerticalPoints - verticeIndex - 1];
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

	// Apply deformation to the starting location
	float pixelValue = GetPixelValue(forwardStepInDeformTexture, loopAroundTunnelCurrentIndex);
	// Pixel value is in range 0-1 and we want to change it to range between -1 - 1
	float directionOfDeform = FMath::Lerp(-1.0f, 1.0f, pixelValue);
	float deform = FMath::Lerp(0.0f, maxFloorDeformation, floorDeformation) * directionOfDeform;

	// Add sideways movement to start location
	float stepSize = horizontalPointSize * (float)loopAroundTunnelCurrentIndex;
	FVector stepToSide = wallStartVertice + rightVector * stepSize;
	return FVector(stepToSide.X, stepToSide.Y, stepToSide.Z + deform);
}

// Constants for rotation lerping values
const float ROTATE_LERP_START_NEGATIVE = -45.0f;
const float ROTATE_LERP_START_POSITIVE = 45.0f;
const float ROTATE_LERP_END = 0.0f;
const float maxStepCountToRound = 20;
const float minStepCountToRound = 2;
const float horizontalPointMaxSize = 225;

// Predefined values for extra movement based on index
const TArray<float> EXTRA_MOVEMENTS = { 100.0f, 50.0f, 30.0f, 10.0f, 0.0f };

FVector AProceduralTunnel::GetVerticeOnRightWall(bool isFirstLoopARound)
{
	bool isEndOrStar = false;

	// If it's the start of the right wall, just use the latest vertex
	if (loopAroundTunnelCurrentIndex == numberOfHorizontalPoints)
	{
		wallStartVertice = latestVertice;
		return latestVertice;
	}

	FVector rVector = rightVector;
	float extraMovementToEnd = 0.0f;
	// Calculate how many steps we need to round in end or in the start of tunnel
	float alphaValue = FMath::Clamp(horizontalPointSize / horizontalPointMaxSize, 0, 1);
	int32 numberOfStepsToRound = FMath::RoundToInt32(FMath::Lerp(maxStepCountToRound, minStepCountToRound, alphaValue));

	// Rotate the start position of the vertex if certain conditions are met
	if (ShouldRotateStartPositionRightWall(numberOfStepsToRound))
	{
		isEndOrStar = true;
		float alpha = float(stepIndexInsideMesh) / float(numberOfStepsToRound);
		float rotateAmount = FMath::Lerp(ROTATE_LERP_START_NEGATIVE, ROTATE_LERP_END, alpha);
		extraMovementToEnd = stopDeformCurve->GetFloatValue(alpha);;
		rVector = RotateVectorByAmount(rightVector, rotateAmount);
	}

	// Rotate the end position of the vertex for valid intersections
	if (IsValid(intersection) && ShouldRotateEndPositionRightWall(numberOfStepsToRound))
	{
		isEndOrStar = true;
		float startValue = stepIndexInsideMesh - (stepCountToMakeCurrentMesh - numberOfStepsToRound);
		float alpha = startValue / float(numberOfStepsToRound);
		float rotateAmount = FMath::Lerp(ROTATE_LERP_END, ROTATE_LERP_START_POSITIVE, alpha);
		extraMovementToEnd = stopDeformCurve->GetFloatValue(1 - alpha);//FMath::Lerp(0, 100, alpha);

		rVector = RotateVectorByAmount(rightVector, rotateAmount);
	}

	// Calculate the relative location on the wall
	float locationOnWall = (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints) / (float)(numberOfVerticalPoints);

	// Adjust the vertex size based on if it's the first loop and if there's a valid parent intersection
	float wVerticeSize = isFirstLoopARound && IsValid(parentIntersection) ? parentIntersection->verticalPointSize : verticalPointSize;

	// Calculate the roundness of the tunnel using the deform curve
	float roundnessAmount = deformCurve->GetFloatValue(locationOnWall);
	FVector tunnelRoundness = rVector * (FMath::Lerp(tunnelRoundValue + extraMovementToEnd, 0.0f, roundnessAmount));

	// Determine the vertical location on the wall and calculate the final wall vertex
	FVector wallVertice = wallStartVertice + FVector(0.0f, 0.0f, (loopAroundTunnelCurrentIndex - numberOfHorizontalPoints) * wVerticeSize);
	wallVertice += tunnelRoundness;

	// Apply deformation to the vertex unless it's at the start or end of the tunnel
	//if (!isEndOrStar) {
		float pixelValue = GetPixelValue(forwardStepInDeformTexture, loopAroundTunnelCurrentIndex);
		float directionOfDeform = FMath::Lerp(-1.0f, 1.0f, pixelValue);
		wallVertice += FMath::Lerp(0.0f, maxWallDeformation, wallDeformation) * directionOfDeform * rightVector;
	//}

	return wallVertice;
}

// Checks whether the start position of the vertex should be rotated based on several conditions
bool AProceduralTunnel::ShouldRotateStartPositionRightWall(int32 stepCountToRound)
{
	bool isStraightTunnelAfterLeftIntersection = tunnelType == TunnelType::StraightTunnel && parentIntersection->intersectionType == IntersectionType::Left;
	bool isLeftTunnelAfterRightLeftIntersection = tunnelType == TunnelType::LeftTunnel && parentIntersection->intersectionType == IntersectionType::RightLeft;

	bool isUnderStepCount = stepIndexInsideMesh <= stepCountToRound;
	bool isNotStartTunnel = tunnelType != TunnelType::StartTunnel;
	bool isParentIntersectionCorrect = !isStraightTunnelAfterLeftIntersection && !isLeftTunnelAfterRightLeftIntersection;
	bool isFirstMesh = (SplineComponent->GetNumberOfSplinePoints() - 1 - indexOfCurrentMesh == 1) || TunnelMeshes.Num() == 0;
	bool conditionFour = (tunnelType == TunnelType::StraightTunnel && IsValid(rightSideTunnel)) || tunnelType != TunnelType::StraightTunnel;

	return isUnderStepCount && isNotStartTunnel && isParentIntersectionCorrect && isFirstMesh && conditionFour;
}

// Checks whether the end position of the vertex should be rotated based on several conditions
bool AProceduralTunnel::ShouldRotateEndPositionRightWall(int32 stepCountToRound)
{
	return indexOfCurrentMesh == 0 &&
		stepIndexInsideMesh <= stepCountToMakeCurrentMesh &&
		stepIndexInsideMesh >= stepCountToMakeCurrentMesh - stepCountToRound &&
		intersection->intersectionType != IntersectionType::Left;
}

// Returns a rotated vector based on a given vector and rotation amount
FVector AProceduralTunnel::RotateVectorByAmount(const FVector& vector, float rotateAmount)
{
	FRotator rotator = FRotator(0.0f, rotateAmount, 0.0f);
	return rotator.RotateVector(vector);
}

// Get vertice for roof
FVector AProceduralTunnel::GetVerticeOnRoof()
{
	// Save start vertice when entering first time
	if (loopAroundTunnelCurrentIndex == numberOfHorizontalPoints + numberOfVerticalPoints)
	{
		wallStartVertice.Z += verticalPointSize * numberOfVerticalPoints;
		//return wallStartVertice;
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
    float pixelValue = GetPixelValue(forwardStepInDeformTexture, loopAroundTunnelCurrentIndex);
	// Pixel value is in range 0-1 and we want to change it to range between -1 - 1
	float directionOfDeform = FMath::Lerp(-1.0f, 1.0f, pixelValue);
	wallVertice.Z += FMath::Lerp(0.0f, maxWallDeformation, wallDeformation) * directionOfDeform;

    return wallVertice;
}

// Get vertice for the left wall of the tunnel
FVector AProceduralTunnel::GetVerticeOnLeftWall(bool isFirstLoopARound)
{
	bool isEndOrStar = false;
	float extraMovementToEnd = 0.0f;
	FVector rVector = rightVector;

	// If we've completed a loop around the tunnel, return the first vertice
	if (loopAroundTunnelCurrentIndex == loopAroundTunnelLastIndex)
	{
		return firstVertice;
	}

	// If this is the start of the wall, save the starting vertice
	if (loopAroundTunnelCurrentIndex == numberOfHorizontalPoints * 2 + numberOfVerticalPoints)
	{
		wallStartVertice = latestVertice;
	}

	// Adjust the vertice size if this is the first loop and a valid parent intersection exists
	float wVerticeSize = isFirstLoopARound && IsValid(parentIntersection) ? parentIntersection->verticalPointSize : verticalPointSize;

	// Calculate how many steps we need to round in end or in the start of tunnel
	float alphaValue = FMath::Clamp(horizontalPointSize / horizontalPointMaxSize, 0, 1);
	int32 numberOfStepsToRound = FMath::RoundToInt32(FMath::Lerp(maxStepCountToRound, minStepCountToRound, alphaValue));

	// Decide if we need to rotate the starting position of the wall based on tunnel conditions
	if (ShouldRotateStartPositionLeftWall(numberOfStepsToRound))
	{
		wallStartVertice = FVector(firstVertice.X, firstVertice.Y, firstVertice.Z + ((float)(numberOfVerticalPoints)*verticalPointSize));
		isEndOrStar = true;

		float alpha = float(stepIndexInsideMesh) / float(numberOfStepsToRound);
		float rotateAmount = FMath::Lerp(ROTATE_LERP_START_POSITIVE, ROTATE_LERP_END, alpha);
		extraMovementToEnd = stopDeformCurve->GetFloatValue(alpha);;
		rVector = RotateVectorByAmount(rightVector, rotateAmount);
	}

	// Decide if we need to rotate the ending position of the wall based on tunnel conditions
	if (ShouldRotateEndPositionLeftWall(numberOfStepsToRound))
	{
		isEndOrStar = true;

		float startValue = stepIndexInsideMesh - (stepCountToMakeCurrentMesh - numberOfStepsToRound);
		float alpha = startValue / float(numberOfStepsToRound);
		float rotateAmount = FMath::Lerp(ROTATE_LERP_END, ROTATE_LERP_START_NEGATIVE, alpha);
		extraMovementToEnd = stopDeformCurve->GetFloatValue(1 - alpha);
		rVector = RotateVectorByAmount(rightVector, rotateAmount);
	}

	// Calculate location on wall and apply roundness based on the deform curve
	float locationOnWall = (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints * 2 - numberOfVerticalPoints + 1) / (float)numberOfVerticalPoints;
	float roundnessAmount = deformCurve->GetFloatValue(locationOnWall);
	if (IsValid(intersection) && stepIndexInsideMesh == stepCountToMakeCurrentMesh)
	{
		roundnessAmount = FMath::Clamp(roundnessAmount - 0.1f, 0.0f, 1.0f);
	}
	FVector tunnelRoundness = rVector * FMath::Lerp((tunnelRoundValue + extraMovementToEnd) * -1, 0.0f, roundnessAmount);

	// Calculate wall vertice position
	float zLocation = (loopAroundTunnelCurrentIndex - numberOfHorizontalPoints * 2 - numberOfVerticalPoints + 1) * (wVerticeSize * -1.0f);
	FVector wallVertice = wallStartVertice + FVector(0.0f, 0.0f, zLocation) + tunnelRoundness;

	// If it's not the end or start, apply deformation based on the deform texture
	//if (!isEndOrStar)
	//{
		float pixelValue = GetPixelValue(forwardStepInDeformTexture, loopAroundTunnelCurrentIndex);
		float directionOfDeform = FMath::Lerp(-1.0f, 1.0f, pixelValue);
		wallVertice += FMath::Lerp(0.0f, maxWallDeformation, wallDeformation) * directionOfDeform * rightVector;
	//}

	return wallVertice;
}

// Check if we need to rotate the starting position of the left wall based on various tunnel conditions
bool AProceduralTunnel::ShouldRotateStartPositionLeftWall(int32 stepCountToRound)
{
	bool isStraightTunnelAfterRightIntersection = tunnelType == TunnelType::StraightTunnel && parentIntersection->intersectionType == IntersectionType::Right;
	bool isRightTunnelAfterRightLeftIntersection = tunnelType == TunnelType::RightTunnel && parentIntersection->intersectionType == IntersectionType::RightLeft;
	
	bool isUnderStepCount = stepIndexInsideMesh <= stepCountToRound;
	bool isNotStartTunnel = tunnelType != TunnelType::StartTunnel;
	bool isParentIntersectionCorrect = !isStraightTunnelAfterRightIntersection && !isRightTunnelAfterRightLeftIntersection;
	bool isFirstMesh = (SplineComponent->GetNumberOfSplinePoints() - 1 - indexOfCurrentMesh == 1) || TunnelMeshes.Num() == 0;
	bool conditionFour = (tunnelType == TunnelType::StraightTunnel && IsValid(leftSideTunnel)) || tunnelType != TunnelType::StraightTunnel;

	return isUnderStepCount && isNotStartTunnel && isParentIntersectionCorrect && isFirstMesh && conditionFour;
}

// Check if we need to rotate the ending position of the left wall based on various tunnel conditions
bool AProceduralTunnel::ShouldRotateEndPositionLeftWall(int32 stepCountToRound)
{
	bool conditionOne = IsValid(intersection) && indexOfCurrentMesh == 0;
	bool conditionTwo = stepIndexInsideMesh <= stepCountToMakeCurrentMesh && stepIndexInsideMesh >= stepCountToMakeCurrentMesh - stepCountToRound;
	bool conditionThree = intersection->intersectionType != IntersectionType::Right;

	return conditionOne && conditionTwo && conditionThree;
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
		? startDistance + ((float)(stepIndexInsideMesh - 1) * horizontalPointSize + lastStepSizeOnSpline)
		: startDistance + (float)stepIndexInsideMesh * horizontalPointSize;

	// Calculate distance on spline in steps
	float distanceOnSpline = currentDistance / horizontalPointSize; 

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
}

// These are values used when tunnel construction is called
void AProceduralTunnel::SetValuesForGeneratingTunnel(float selected, bool undo, FVector2D tunnelScale, FVector2D sVariation, bool reset) 
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
}