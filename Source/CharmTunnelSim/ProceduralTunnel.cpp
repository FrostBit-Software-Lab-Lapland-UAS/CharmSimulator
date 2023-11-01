

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
			if (proceduralTunnel->tunnelType == TunnelType::StartTunnel)
			{
				SplinePointIndexToSnap = 0;
			}
			else {
				SplinePointIndexToSnap = spline->GetNumberOfSplinePoints() - 1;
			}

			FVector comparedPointsLocation = spline->GetLocationAtSplinePoint(SplinePointIndexToSnap, ESplineCoordinateSpace::World);

			// Calculate the distance between the current tunnel end and the other tunnel end
			float distance = FVector::Distance(comparedPointsLocation, lastPointLocation);

			// Get the forward direction of the current tunnel's end
			FVector currentEndForward = SplineComponent->GetDirectionAtSplinePoint(lastIndex, ESplineCoordinateSpace::World);

			// Calculate the direction from the current tunnel end to the other tunnel end
			FVector directionToOtherEnd = (comparedPointsLocation - lastPointLocation).GetSafeNormal();

			// Calculate the dot product
			float dotProduct = FVector::DotProduct(currentEndForward, directionToOtherEnd);

			// Check if the distance is within the maximum allowed distance and update the closest point and tangent if necessary
			if ((distance < maxDistanceToSnapSplineEnds && dotProduct >= 0) || comparedPointsLocation == lastPointLocation)
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

			// Get the forward direction of the current tunnel's end
			FVector currentEndForward = SplineComponent->GetDirectionAtSplinePoint(lastIndex, ESplineCoordinateSpace::World);

			// Calculate the direction from the current tunnel end to the other tunnel end
			FVector directionToOtherEnd = (selfFirstPointLocation - lastPointLocation).GetSafeNormal();

			// Calculate the dot product
			float dotProduct = FVector::DotProduct(currentEndForward, directionToOtherEnd);

			// Check if the distance is within the maximum allowed distance and update the closest point and tangent if necessary
			if (distance < maxDistanceToSnapSplineEnds && dotProduct > 0)
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
		FVector direction = SplineComponent->GetDirectionAtSplinePoint(lastIndex, ESplineCoordinateSpace::World);
		FVector secondPointLocation = SplineComponent->GetLocationAtSplinePoint(lastIndex - 1, ESplineCoordinateSpace::World);
		if (SplineComponent->GetNumberOfSplinePoints() > 2) {
			SplineComponent->SetLocationAtSplinePoint(lastIndex - 1, FMath::Lerp(closestPoint - direction * 600, secondPointLocation, 0.2f), ESplineCoordinateSpace::World, true);
		}
	}
	// If no closest point was found, set the end connection status to false
	else
	{
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

// Adds or removes spline points depending on the distance of dragged spline point and previous spline point
void AProceduralTunnel::AddOrRemoveSplinePoints()
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

// Returns negative value of how many meshes we need to remakes
int32 AProceduralTunnel::GetMeshIndexToStartRecreation()
{
	float tunnelWidth = widthScale * 100.0f;
	float tunnelHeight = heightScale * 100.0f;
	horizontalPointSize = tunnelWidth / (float)(numberOfHorizontalPoints - 1);
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

// Get vertice for right wall
FVector AProceduralTunnel::GetVerticeOnRightWall(bool isFirstLoopARound)
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
	
	if (tunnelType != TunnelType::StartTunnel &&
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
	if (IsValid(intersection)) {
		if (indexOfCurrentMesh == 0 &&
			stepIndexInsideMesh <= stepCountToMakeCurrentMesh &&
			stepIndexInsideMesh >= stepCountToMakeCurrentMesh - 4 &&
			intersection->intersectionType != IntersectionType::Left)
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
		// Apply deformation to the starting location
		float pixelValue = GetPixelValue(forwardStepInDeformTexture, loopAroundTunnelCurrentIndex);
		// Pixel value is in range 0-1 and we want to change it to range between -1 - 1
		float directionOfDeform = FMath::Lerp(-1.0f, 1.0f, pixelValue);
		wallVertice += FMath::Lerp(0.0f, maxWallDeformation, wallDeformation) * directionOfDeform * rightVector;
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

// Get vertice for left wall
FVector AProceduralTunnel::GetVerticeOnLeftWall(bool isFirstLoopARound)
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
	if (tunnelType != TunnelType::StartTunnel && stepIndexInsideMesh <= 4 &&
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
	if (IsValid(intersection)) {
		if (indexOfCurrentMesh == 0 &&
			stepIndexInsideMesh <= stepCountToMakeCurrentMesh &&
			stepIndexInsideMesh >= stepCountToMakeCurrentMesh - 4 &&
			intersection->intersectionType != IntersectionType::Right)
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
	}

	// Get the location on normalized range at what point are on wall ( 0 = start and 1 = end )
	float locationOnWall = (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints * 2 - numberOfVerticalPoints + 1) / (float)numberOfVerticalPoints;
	// Add roundness to the tunnel wall

	float roundnessAmount = deformCurve->GetFloatValue(locationOnWall);
	if (IsValid(intersection) && stepIndexInsideMesh == stepCountToMakeCurrentMesh)
	{
		roundnessAmount = FMath::Clamp((float)(roundnessAmount - 0.1f), 0.0f, 1.0f);
	}
	FVector tunnelRoundnesss = rVector * FMath::Lerp((tunnelRoundValue + extraMovementToEnd) * -1, 0.0f, roundnessAmount); // AMOUNT WE MOVE TO SIDE TO CREATE  THAT ROUND LOOK OF TUNNEL
	float zLocation = (loopAroundTunnelCurrentIndex - numberOfHorizontalPoints * 2 - numberOfVerticalPoints + 1) * (wVerticeSize * -1.0f);


	FVector wallVertice = wallStartVertice + FVector(0.0f, 0.0f, zLocation);
	wallVertice += tunnelRoundnesss;

	if (!isEndOrStar) {
		// Apply deformation to the starting location
		float pixelValue = GetPixelValue(forwardStepInDeformTexture, loopAroundTunnelCurrentIndex);
		// Pixel value is in range 0-1 and we want to change it to range between -1 - 1
		float directionOfDeform = FMath::Lerp(-1.0f, 1.0f, pixelValue);
		wallVertice += FMath::Lerp(0.0f, maxWallDeformation, wallDeformation) * directionOfDeform * rightVector;
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