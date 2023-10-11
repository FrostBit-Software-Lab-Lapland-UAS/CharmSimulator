// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#include "ProceduralIntersection.h"
#include "ProceduralTunnel.h"
#include "Components/StaticMeshComponent.h"
#include "Kismet/KismetMathLibrary.h"


// Sets default values
AProceduralIntersection::AProceduralIntersection()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AProceduralIntersection::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AProceduralIntersection::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

// Set values
void AProceduralIntersection::SetValues(FVector2D scale, IntersectionType type, AProceduralTunnel* parent, FVector2D variation, bool update)
{
	localScale = scale;
	surfaceVariation = variation;
	intersectionType = type;
	if (IsValid(parent)) {
		UE_LOG(LogTemp, Warning, TEXT("Intersections parent is valid"));
		parentTunnel = parent;
	}
	if (!update) {
		if (IsValid(parent)) {
			numberOfHorizontalPoints = parent->numberOfHorizontalPoints;
			numberOfVerticalPoints = parent->numberOfVerticalPoints;
		}
		
		switch (intersectionType)
		{
		case IntersectionType::Right:
		case IntersectionType::Left:
			loopAroundTunnelLastIndex = numberOfHorizontalPoints * 2 + numberOfVerticalPoints - 1;
			break;
		case IntersectionType::RightLeft:
		case IntersectionType::All:
			loopAroundTunnelLastIndex = numberOfHorizontalPoints * 2 - 1;
			break;
		}
	}
	isUpdate = update;

	horizontalPointSize = (localScale.X * 100.0f) / (float)(numberOfHorizontalPoints - 1);
	verticalPointSize = (localScale.Y * 100.0f) / (float)(numberOfVerticalPoints);
}

// Clear arrays
void AProceduralIntersection::ClearArrays()
{
	lastRightWallVertices.Empty();
	lastLeftWallVertices.Empty();
	lastStraightRoofVertices.Empty();
	lastRightRoofVertices.Empty();
	lastLeftRoofVertices.Empty();
	lastRightFloorVertices.Empty();
	lastStraightFloorVertices.Empty();
	lastLeftFloorVertices.Empty();
	wallVertices.Empty();
	groundVertices.Empty();
	wallUV.Empty();
	groundUV.Empty();
	//lastStraightRoofVertices.Empty();
}

// Generate Intersection to right and straight
void AProceduralIntersection::IntersectionGenerationLoop() 
{
	// Forward loop is the size of points in width
	for (forwarLoopIndex = 0; forwarLoopIndex < numberOfHorizontalPoints; forwarLoopIndex++)
	{
		latestVertice = FVector((float)forwarLoopIndex * horizontalPointSize, 0.0f, 0.0f);
		// Around tunnel loop
		for (loopAroundTunnelCurrentIndex = 0; loopAroundTunnelCurrentIndex <= loopAroundTunnelLastIndex; loopAroundTunnelCurrentIndex++)
		{
			surfaceIndex = GetSurfaceIndex();
			latestVertice = GetVertice();
			StoreVertice();
		}
	}
	
	// End wall of this type intersection needs to be done invidually
	if (intersectionType == IntersectionType::RightLeft) {
		int32 numberOfPointsInEndWall = numberOfHorizontalPoints * numberOfVerticalPoints;
		FVector startVertice = lastStraightRoofVertices[0];

		int32 previousRow = -1;
		for (int32 endWallCurrentIndex = 0; endWallCurrentIndex < numberOfPointsInEndWall; endWallCurrentIndex++) {
			// This is return the row we currently are
			int32 currentRow = FMath::Floor(endWallCurrentIndex / numberOfHorizontalPoints);

			if (previousRow != -1 && previousRow != currentRow) {
				lastLeftWallVertices.Add(latestVertice);
			}

			// If there is only one row left
			if (endWallCurrentIndex >= numberOfPointsInEndWall - numberOfHorizontalPoints) {
				int32 lastRowIndex = endWallCurrentIndex - (numberOfPointsInEndWall - numberOfHorizontalPoints);
				if (currentRow != previousRow) {
					lastRightWallVertices.Add(lastStraightFloorVertices[lastStraightFloorVertices.Num() - lastRowIndex - 1]);
					previousRow = currentRow;
				}
				if (endWallCurrentIndex == numberOfPointsInEndWall - 1) {
					lastLeftWallVertices.Add(lastStraightFloorVertices[lastStraightFloorVertices.Num() - lastRowIndex - 1]);
				}
				wallVertices.Add(lastStraightFloorVertices[lastStraightFloorVertices.Num() - lastRowIndex - 1]);
			}
			// If we have moved to new row. This is also triggered in the beginning of the loop
			else if (currentRow != previousRow) {
				// Change startVertice z location one step down
				startVertice.Z -= verticalPointSize;
				// Add roundness to start vertice
				float locationOnWall = (float)(currentRow + 1) / (float)(numberOfVerticalPoints);
				float roundnessAmount = roundnessCurve->GetFloatValue(locationOnWall);
				float tunnelRounding = FMath::Lerp(tunnelRoundValue, 0.0f, roundnessAmount);

				startVertice.X = lastStraightRoofVertices[0].X + tunnelRounding + horizontalPointSize;
				// Set latest vertice to be our moved start vertice
				latestVertice = startVertice;
				// Store vertice
				wallVertices.Add(latestVertice);
				lastRightWallVertices.Add(latestVertice);
				// Set previous row to be current row
				previousRow = currentRow;
			}
			else {
				latestVertice.Y -= horizontalPointSize;
				wallVertices.Add(latestVertice);
			}
		}
	}
	MakeMeshTriangles();
	MakeMeshTangentsAndNormals();
	MakeMesh();
	if(!isUpdate) 
	{
		AddContinuationTunnels();
	}
}

void AProceduralIntersection::StoreVertice()
{

	if (surfaceIndex == 0) {
		groundVertices.Add(latestVertice);
	}
	else {
		wallVertices.Add(latestVertice);
	}

	switch (intersectionType)
	{
	case IntersectionType::Right:
		if (surfaceIndex == 2 && loopAroundTunnelCurrentIndex == 0) {
			lastRightRoofVertices.Add(latestVertice);
		}
		if (surfaceIndex == 2 && forwarLoopIndex == numberOfHorizontalPoints - 1) {
			lastStraightRoofVertices.Add(latestVertice);
		}
		if (surfaceIndex == 3 && forwarLoopIndex == numberOfHorizontalPoints - 1) {
			lastLeftWallVertices.Add(latestVertice);
		}
		if (surfaceIndex == 0 && loopAroundTunnelCurrentIndex == numberOfHorizontalPoints + numberOfVerticalPoints) {
			lastLeftFloorVertices.Add(latestVertice);
		}
		if (surfaceIndex == 0 && forwarLoopIndex == numberOfHorizontalPoints - 1) {
			lastStraightFloorVertices.Add(latestVertice);
		}
		if (surfaceIndex == 0 && loopAroundTunnelCurrentIndex == loopAroundTunnelLastIndex) {
			lastRightFloorVertices.Add(latestVertice);
		}
		break;
	case IntersectionType::Left:
		if (surfaceIndex == 0 && loopAroundTunnelCurrentIndex == 0) {
			lastLeftFloorVertices.Add(latestVertice);
		}
		if (surfaceIndex == 0 && forwarLoopIndex == numberOfHorizontalPoints - 1) {
			lastStraightFloorVertices.Add(latestVertice);
		}
		if (surfaceIndex == 1 && loopAroundTunnelCurrentIndex == numberOfHorizontalPoints) {
			lastRightFloorVertices.Add(latestVertice);
		}
		if (surfaceIndex == 1 && forwarLoopIndex == numberOfHorizontalPoints - 1) {
			lastRightWallVertices.Add(latestVertice);
		}
		if (surfaceIndex == 2 && loopAroundTunnelCurrentIndex == loopAroundTunnelLastIndex) {
			lastLeftRoofVertices.Add(latestVertice);
		}
		if (surfaceIndex == 2 && forwarLoopIndex == numberOfHorizontalPoints - 1) {
			lastStraightRoofVertices.Add(latestVertice);
		}
		break;
	case IntersectionType::RightLeft:
	case IntersectionType::All:
		if (surfaceIndex == 0 && loopAroundTunnelCurrentIndex == 0) {
			lastLeftFloorVertices.Add(latestVertice);
		}
		if (surfaceIndex == 0 && forwarLoopIndex == numberOfHorizontalPoints - 1) {
			lastStraightFloorVertices.Add(latestVertice);
		}
		if (surfaceIndex == 0 && loopAroundTunnelCurrentIndex == numberOfHorizontalPoints - 1) {
			lastRightFloorVertices.Add(latestVertice);
		}
		if (surfaceIndex == 2 && loopAroundTunnelCurrentIndex == loopAroundTunnelLastIndex) {
			lastLeftRoofVertices.Add(latestVertice);
		}
		if (surfaceIndex == 2 && forwarLoopIndex == numberOfHorizontalPoints - 1) {
			lastStraightRoofVertices.Add(latestVertice);
		}
		if (surfaceIndex == 2 && loopAroundTunnelCurrentIndex == numberOfHorizontalPoints ) {
			lastRightRoofVertices.Add(latestVertice);
		}
		break;
	}
}

// Return the index of the surface we are currently on.
// 0 = Floor, 1 = Right, 2 = Roof, 3 = Left
int32 AProceduralIntersection::GetSurfaceIndex()
{
	switch (intersectionType)
	{
	case IntersectionType::Right:
		if (loopAroundTunnelCurrentIndex < numberOfHorizontalPoints) return 2;
		else if (loopAroundTunnelCurrentIndex < numberOfHorizontalPoints + numberOfVerticalPoints) return 3;
		else if (loopAroundTunnelCurrentIndex < numberOfHorizontalPoints * 2 + numberOfVerticalPoints) return 0;
		break;
	case IntersectionType::Left:
		if (loopAroundTunnelCurrentIndex < numberOfHorizontalPoints) return 0;
		else if (loopAroundTunnelCurrentIndex < numberOfHorizontalPoints + numberOfVerticalPoints) return 1;
		else if (loopAroundTunnelCurrentIndex < numberOfHorizontalPoints * 2 + numberOfVerticalPoints) return 2;
		break;
	case IntersectionType::RightLeft:
	case IntersectionType::All:
		if (loopAroundTunnelCurrentIndex < numberOfHorizontalPoints) return 0;
		else if (loopAroundTunnelCurrentIndex < numberOfHorizontalPoints * 2) return 2;
		break;
	}
	return -1; // This is never done
}

// Returns index used to get correct vertice from parent tunnel 
int32 AProceduralIntersection::GetArrayIndex()
{
	switch (intersectionType)
	{
	case IntersectionType::Right:
		if (surfaceIndex == 0) return loopAroundTunnelCurrentIndex - numberOfHorizontalPoints - numberOfVerticalPoints;
		else return loopAroundTunnelCurrentIndex + numberOfVerticalPoints; // Roof
		break;
	case IntersectionType::Left:
		if (surfaceIndex == 0) return loopAroundTunnelCurrentIndex; // Ground
		else return loopAroundTunnelCurrentIndex - numberOfHorizontalPoints; 
		break;
	case IntersectionType::RightLeft:
	case IntersectionType::All:
		if (surfaceIndex == 0) return loopAroundTunnelCurrentIndex; // Ground
		else return loopAroundTunnelCurrentIndex - numberOfHorizontalPoints + numberOfVerticalPoints; // Roof 
		break;
	}
	return 0;
}

// Returns vertice used for creating tunnel shape
FVector AProceduralIntersection::GetVertice()
{
	FVector vertice;

	if (forwarLoopIndex == 0 && IsValid(parentTunnel))
	{
		int32 arrayIndex = GetArrayIndex();
		TArray<FVector>* targetArray = nullptr;
		if (surfaceIndex == 0) {
			targetArray = &(parentTunnel->meshEnds[parentTunnel->meshEnds.Num() - 1].GroundVertives);
		}
		else {
			targetArray = &(parentTunnel->meshEnds[parentTunnel->meshEnds.Num() - 1].WallVertices);
		}

		if (targetArray && targetArray->Num() > arrayIndex)
		{
			vertice = (*targetArray)[arrayIndex];
			vertice = TransformVertex(vertice);
		}
	}
	else
	{
		vertice = GetVerticeBySurface(surfaceIndex, intersectionType);
	}

	return vertice;
}

// Transform vertice to local space from parents space
FVector AProceduralIntersection::TransformVertex(FVector vertex)
{
	vertex = UKismetMathLibrary::TransformLocation(parentTunnel->GetTransform(), vertex);
	return UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertex);
}

// Get correct vertice for different types of intersections
FVector AProceduralIntersection::GetVerticeBySurface(int32 surface, IntersectionType type)
{
	switch (surface)
	{
	case 0:
		return GetFloorVertice(); 
	case 1:
		return GetRightVertice();
	case 2:
		return GetRoofVertice();
	case 3:
		return GetLeftVertice();
	}
	return FVector(0.0f, 0.0f, 0.0f);
}

// Get the floor vertice for the intersection
FVector AProceduralIntersection::GetFloorVertice()
{
	float sideWaysMovementAmount = 0.0f;

	// Calculate the amount of sideways movement based on the intersection type.
	switch (intersectionType)
	{
	case IntersectionType::Right:
		sideWaysMovementAmount = horizontalPointSize * (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints - numberOfVerticalPoints);
		if (loopAroundTunnelCurrentIndex == numberOfHorizontalPoints + numberOfVerticalPoints) {
			return firstVertice = latestVertice;
		}
		break;
	case IntersectionType::Left:
	case IntersectionType::RightLeft:
	case IntersectionType::All:
		sideWaysMovementAmount = horizontalPointSize * loopAroundTunnelCurrentIndex;
		if (loopAroundTunnelCurrentIndex == 0)
		{
			FVector verticeOffset = FVector(0.0f, (float)(numberOfHorizontalPoints - 1) / 2.0f * horizontalPointSize, 0.0f);
			return firstVertice = latestVertice - verticeOffset;
		}
		break;
	}

	// Calculate the position of the vertice.
	float deformAmount = GetPixelValue(forwarLoopIndex, loopAroundTunnelCurrentIndex);
	float amountOfDeform = FMath::Lerp(0.0f, deformAmount, surfaceVariation.X);
	float deform = FMath::RandRange(amountOfDeform * -20, amountOfDeform * 20);

	return firstVertice + FVector(0.0f, sideWaysMovementAmount, -deform);
}

// Get the right vertice for the left side intersection
FVector AProceduralIntersection::GetRightVertice()
{
	// If this is the first vertice in the intersection, set it as the starting point.
	if(loopAroundTunnelCurrentIndex == numberOfHorizontalPoints)
	{
		firstVertice = latestVertice;
		return latestVertice;
	}

	// Calculate the location of the vertice along the wall and the amount of roundness based on that location.
	float locationOnWall = (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints) / (float)numberOfVerticalPoints;
	float roundnessAmount = roundnessCurve->GetFloatValue(locationOnWall);
	float tunnelRounding = FMath::Lerp(tunnelRoundValue, 0.0f, roundnessAmount);

	// Calculate the amount of deform based on the texture value at this point.
	float maxValue = FMath::Lerp(0.0f, maxDeform, surfaceVariation.Y);
	float deform = GetPixelValue(forwarLoopIndex, loopAroundTunnelCurrentIndex) * maxValue;

	// Calculate the position of the vertice.
	FVector vertice = firstVertice + FVector(0.0f, 0.0f, verticalPointSize * (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints));
	vertice += FVector(0.0f, tunnelRounding, 0.0f);
	vertice += FVector(0.0f, deform, 0.0f);

	return vertice;
}

// This function returns the position of the roof vertice in the intersection.
FVector AProceduralIntersection::GetRoofVertice()
{
	// Calculate the location of the vertice on the roof based on the intersection type.
	float roundness;
	float locationOnYRoof;
	float locationOnXRoof = (float)forwarLoopIndex / (float)(numberOfHorizontalPoints - 1);
	float roofXRoundness = roundnessCurve->GetFloatValue(locationOnXRoof);
	float roofYRoundness;
	switch (intersectionType) 
	{
	case IntersectionType::Right:
		locationOnYRoof = (float)(loopAroundTunnelCurrentIndex + 1) / (float)numberOfHorizontalPoints;
		roofYRoundness = roundnessCurve->GetFloatValue(locationOnYRoof);
		if (locationOnYRoof >= 0.5) roundness = roofYRoundness;
		else roundness = FMath::Min(roofYRoundness, roofXRoundness);
		break;
	case IntersectionType::Left:
		locationOnYRoof = (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints - numberOfVerticalPoints + 1 ) / (float)numberOfHorizontalPoints;
		roofYRoundness = roundnessCurve->GetFloatValue(locationOnYRoof);
		if (locationOnYRoof < 0.5) roundness = roofYRoundness;
		else roundness = FMath::Min(roofYRoundness, roofXRoundness);
		break;
	case IntersectionType::RightLeft:
		locationOnYRoof = (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints + 1) / (float)numberOfHorizontalPoints;
		roofYRoundness = roundnessCurve->GetFloatValue(locationOnYRoof);
		if (locationOnXRoof >= 0.5) roundness = roofXRoundness;
		else roundness = FMath::Min(roofYRoundness, roofXRoundness);
		break;
	case IntersectionType::All:
		locationOnYRoof = (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints + 1) / (float)numberOfHorizontalPoints;
		roofYRoundness = roundnessCurve->GetFloatValue(locationOnYRoof);
		roundness = FMath::Min(roofYRoundness, roofXRoundness);
		break;
	}

	float tunnelRounding = FMath::Lerp(tunnelRoundValue, 0.0f, roundness);

	// Calculate the position of the vertex based on the intersection type.
	if (intersectionType == IntersectionType::Right) 
	{ 
		FVector selectedVector;

		if (loopAroundTunnelCurrentIndex == 0) 
		{
			// If we are at the start of the intersection, set the starting point.
			float yLocation = ((float)(numberOfHorizontalPoints - 1) / 2.0f) * horizontalPointSize;
			selectedVector = latestVertice + FVector(0.0f, yLocation, ((float)numberOfVerticalPoints) * verticalPointSize);
			firstVertice = selectedVector;
		}
		else {
			// Otherwise, move on the Y axis.
			selectedVector = firstVertice - FVector(0.0f, (float)(loopAroundTunnelCurrentIndex * horizontalPointSize), 0.0f);
		}

		// Add roundness to the vertice position. Divide rounding by 2 to add more natural roundness to roof
		selectedVector.Z += tunnelRounding / 2; // ADD ROUNDNESS
		
		// Calculate the amount of deform and add it to the vertice position.
		float maxValue = FMath::Lerp(0.0f, maxDeform, surfaceVariation.Y); 
		float deform = GetPixelValue(forwarLoopIndex, loopAroundTunnelCurrentIndex) * maxValue; 
		return selectedVector + FVector(0.0f,0.0f, deform); 

	} 
	else if (intersectionType == IntersectionType::Left) // LEFT SIDE INTERSECTION
	{
		if(loopAroundTunnelCurrentIndex == numberOfHorizontalPoints + numberOfVerticalPoints)
		{
			firstVertice.Z += verticalPointSize * numberOfVerticalPoints;
			return firstVertice;
		}

		FVector selectedVector = firstVertice;
		float sideWaysMovementSize = (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints - numberOfVerticalPoints) * horizontalPointSize;
		selectedVector.Y -= sideWaysMovementSize;

		// Add roundness to the vertex position.
		selectedVector.Z += tunnelRounding / 2; // ADD ROUNDNESS
		
		// Calculate the amount of deform and add it to the vertex position.
		float maxValue = FMath::Lerp(0.0f, maxDeform, surfaceVariation.Y);
		float deform = GetPixelValue(forwarLoopIndex, loopAroundTunnelCurrentIndex) * maxValue;
		return selectedVector + FVector(0.0f,0.0f, deform);

	} 
	else // ALL SIDE INTERSECTION
	{
		if(loopAroundTunnelCurrentIndex == numberOfHorizontalPoints)
		{
			firstVertice = latestVertice + FVector(0.0f, 0.0f, numberOfVerticalPoints * verticalPointSize);
		}

		FVector selectedVector = firstVertice;
		float sideWaysMovementSize = (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints) * horizontalPointSize;
		selectedVector.Y -= sideWaysMovementSize;

		selectedVector.Z += tunnelRounding / 2; // ADD ROUNDNESS

		return selectedVector;
	}
}

// This function returns the position of the left vertice in the right side intersection.
FVector AProceduralIntersection::GetLeftVertice()
{
	// Set the position of the first vertex if we are on the last vertice of the wall.
	if (loopAroundTunnelCurrentIndex == numberOfHorizontalPoints) {
		firstVertice = latestVertice;
	}

	// Calculate the location of the vertice on the wall.
	float locationOnWall = (float)(loopAroundTunnelCurrentIndex - numberOfHorizontalPoints + 1) / (float)numberOfVerticalPoints;

	// Calculate the amount of roundness and tunnel rounding.
	float roundnessAmount = roundnessCurve->GetFloatValue(locationOnWall);
	float tunnelRounding = FMath::Lerp(tunnelRoundValue, 0.0f, roundnessAmount);

	// Calculate the amount of deform.
	float maxValue = FMath::Lerp(0.0f, maxDeform, surfaceVariation.Y);
	float deform = GetPixelValue(forwarLoopIndex, loopAroundTunnelCurrentIndex) * maxValue;

	// Calculate the position of the vertice.
	FVector vertice = firstVertice - FVector(0.0f, 0.0f, verticalPointSize * (loopAroundTunnelCurrentIndex - numberOfHorizontalPoints + 1));
	vertice.Y -= tunnelRounding; // ADD ROUNDNESS
	vertice.Y -= deform;      // ADD DEFORM
	// Prevent deformation on last index. Because this would affect the grounds deformation
	if (loopAroundTunnelCurrentIndex == numberOfHorizontalPoints + numberOfVerticalPoints - 1) {
		if (IsValid(parentTunnel)) {
			
			vertice.Z = TransformVertex(parentTunnel->meshEnds[parentTunnel->meshEnds.Num() - 1].WallVertices[parentTunnel->meshEnds[parentTunnel->meshEnds.Num() - 1].WallVertices.Num() - 1]).Z;
			vertice.Y = TransformVertex(parentTunnel->meshEnds[parentTunnel->meshEnds.Num() - 1].WallVertices[parentTunnel->meshEnds[parentTunnel->meshEnds.Num() - 1].WallVertices.Num() - 1]).Y;
		}
		else if (groundVertices.Num() > 0) {
			vertice.Z = groundVertices[0].Z;
			vertice.Y = groundVertices[0].Y;
		}
		
	}

	// Return the position vector of the vertice.
	return vertice;
}