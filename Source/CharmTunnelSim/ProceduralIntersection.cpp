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
	if (!update) {
		parentTunnel = parent;
		pointsInRoof = parent->pointsInRoof;
		pointsInGround = parent->pointsInGround;
		pointsInRightWall = parent->pointsInRightWall;
		pointsInLeftWall = parent->pointsInLeftWall;
		switch (intersectionType)
		{
		case IntersectionType::Right:
			loopAroundTunnelLastIndex = pointsInGround + pointsInLeftWall + pointsInRoof - 1;
			break;
		case IntersectionType::Left:
			loopAroundTunnelLastIndex = pointsInGround + pointsInRightWall + pointsInRoof - 1;
			break;
		case IntersectionType::RightLeft:
		case IntersectionType::All:
			loopAroundTunnelLastIndex = pointsInGround + pointsInRoof - 1;
			break;
		}
	}
	isUpdate = update;

	groundVerticeSize = (localScale.X * 100.0f) / (float)(pointsInRoof);
	wallVerticeSize = (localScale.Y * 100.0f) / (float)(pointsInLeftWall);
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
	for (forwarLoopIndex = 0; forwarLoopIndex < pointsInGround; forwarLoopIndex++)
	{
		latestVertice = FVector((float)forwarLoopIndex * groundVerticeSize, 0.0f, 0.0f);
		// Around tunnel loop
		for (loopAroundTunnelCurrentIndex = 0; loopAroundTunnelCurrentIndex <= loopAroundTunnelLastIndex; loopAroundTunnelCurrentIndex++)
		{
			surfaceIndex = GetSurfaceIndex();
			if (forwarLoopIndex == pointsInGround - 1 && surfaceIndex != 0) {

			}
			else {
				latestVertice = GetVertice();
				StoreVertice();
			}			
		}
	}
	
	// End wall of this type intersection needs to be done invidually
	if (intersectionType == IntersectionType::RightLeft) {
		int32 numberOfPointsInEndWall = pointsInRoof * pointsInLeftWall;
		FVector startVertice = lastStraightRoofVertices[0];
		startVertice.X += groundVerticeSize;

		int32 previousRow = -1;
		for (int32 endWallCurrentIndex = 0; endWallCurrentIndex < numberOfPointsInEndWall; endWallCurrentIndex++) {
			// This is return the row we currently are
			int32 currentRow = FMath::Floor(endWallCurrentIndex / pointsInRoof); 

			// If there is only one row left
			if (endWallCurrentIndex >= numberOfPointsInEndWall - pointsInRoof) {
				int32 lastRowIndex = endWallCurrentIndex - (numberOfPointsInEndWall - pointsInRoof);
				if (currentRow != previousRow) {
					lastRightWallVertices.Add(lastStraightFloorVertices[lastStraightFloorVertices.Num() - lastRowIndex - 2]);
					previousRow = currentRow;
				}
				if (endWallCurrentIndex == numberOfPointsInEndWall - 1) {
					lastLeftWallVertices.Add(lastStraightFloorVertices[lastStraightFloorVertices.Num() - lastRowIndex - 2]);
				}
				wallVertices.Add(lastStraightFloorVertices[lastStraightFloorVertices.Num() - lastRowIndex - 2]);
			}
			// If we have moved to new row. This is also triggered in the beginning of the loop
			else if (currentRow != previousRow) {
				// Change startVertice z location one step down
				startVertice.Z -= wallVerticeSize;
				// Add roundness to start vertice
				float locationOnWall = (float)(currentRow + 1) / (float)(pointsInLeftWall);
				float roundnessAmount = roundnessCurve->GetFloatValue(locationOnWall);
				float tunnelRounding = FMath::Lerp(tunnelRoundValue, 0.0f, roundnessAmount);

				startVertice.X = lastStraightRoofVertices[0].X + tunnelRounding + groundVerticeSize;
				// Set latest vertice to be our moved start vertice
				latestVertice = startVertice;
				// Store vertice
				wallVertices.Add(latestVertice);
				lastRightWallVertices.Add(latestVertice);
				// Set previous row to be current row
				previousRow = currentRow;
			}
			else {
				latestVertice.Y -= groundVerticeSize;
				if (endWallCurrentIndex % (pointsInRoof - 1) == 0) {
					lastLeftWallVertices.Add(latestVertice);
				}
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
	//Currently no need for using uv's
	/*
	float divider;
	float divident;

	switch (intersectionType)
	{
	case IntersectionType::Right:
		divider = surfaceIndex != 0 ? loopAroundTunnelLastIndex -pointsInGround : loopAroundTunnelLastIndex - pointsInRoof - pointsInWalls;
		divident = surfaceIndex != 0 ? loopAroundTunnelCurrentIndex : loopAroundTunnelCurrentIndex - pointsInRoof - pointsInWalls;
		break;
	case IntersectionType::Left:
		divider = surfaceIndex == 0 ? loopAroundTunnelLastIndex - pointsInWalls - pointsInRoof : loopAroundTunnelLastIndex - pointsInGround;
		divident = surfaceIndex == 0 ? loopAroundTunnelCurrentIndex : loopAroundTunnelCurrentIndex - pointsInGround;
		break;
	case IntersectionType::All:
	case IntersectionType::RightLeft:
		divider = surfaceIndex == 0 ? loopAroundTunnelLastIndex - pointsInRoof : loopAroundTunnelLastIndex - pointsInGround;
		divident = surfaceIndex == 0 ? loopAroundTunnelCurrentIndex : loopAroundTunnelCurrentIndex - pointsInGround;
		break;
	}

	float x = (float)forwarLoopIndex / (float)pointsInGround;
	float y = divident / divider;
	*/
	if (surfaceIndex == 0) {
		//groundUV.Add(FVector2D(x, y));
		groundVertices.Add(latestVertice);
	}
	else {
		//wallUV.Add(FVector2D(x, y));
		wallVertices.Add(latestVertice);
	}

	switch (intersectionType)
	{
	case IntersectionType::Right:
		if (surfaceIndex == 2 && loopAroundTunnelCurrentIndex == 0) {
			lastRightRoofVertices.Add(latestVertice);
		}
		if (surfaceIndex == 2 && forwarLoopIndex == pointsInGround - 2) {
			lastStraightRoofVertices.Add(latestVertice);
		}
		if (surfaceIndex == 3 && forwarLoopIndex == pointsInGround - 2) {
			lastLeftWallVertices.Add(latestVertice);
		}
		if (surfaceIndex == 0 && forwarLoopIndex == pointsInGround - 1) {
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
		if (surfaceIndex == 0 && forwarLoopIndex == pointsInGround - 1) {
			lastStraightFloorVertices.Add(latestVertice);
		}
		if (surfaceIndex == 1 && forwarLoopIndex == pointsInGround - 2) {
			lastRightWallVertices.Add(latestVertice);
		}
		if (surfaceIndex == 2 && loopAroundTunnelCurrentIndex == loopAroundTunnelLastIndex) {
			lastLeftRoofVertices.Add(latestVertice);
		}
		if (surfaceIndex == 2 && forwarLoopIndex == pointsInGround - 2) {
			lastStraightRoofVertices.Add(latestVertice);
		}
		break;
	case IntersectionType::RightLeft:
	case IntersectionType::All:
		if (surfaceIndex == 0 && loopAroundTunnelCurrentIndex == 0) {
			lastLeftFloorVertices.Add(latestVertice);
		}
		if (surfaceIndex == 0 && forwarLoopIndex == pointsInGround - 1) {
			lastStraightFloorVertices.Add(latestVertice);
		}
		if (surfaceIndex == 0 && loopAroundTunnelCurrentIndex == pointsInGround - 1) {
			lastRightFloorVertices.Add(latestVertice);
		}
		if (surfaceIndex == 2 && loopAroundTunnelCurrentIndex == loopAroundTunnelLastIndex) {
			lastLeftRoofVertices.Add(latestVertice);
		}
		if (surfaceIndex == 2 && forwarLoopIndex == pointsInGround - 2) {
			lastStraightRoofVertices.Add(latestVertice);
		}
		if (surfaceIndex == 2 && loopAroundTunnelCurrentIndex == pointsInGround ) {
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
		if (loopAroundTunnelCurrentIndex < pointsInRoof) return 2;
		else if (loopAroundTunnelCurrentIndex < pointsInRoof + pointsInLeftWall) return 3;
		else if (loopAroundTunnelCurrentIndex < pointsInRoof + pointsInLeftWall + pointsInGround) return 0;
		break;
	case IntersectionType::Left:
		if (loopAroundTunnelCurrentIndex < pointsInGround) return 0;
		else if (loopAroundTunnelCurrentIndex < pointsInGround + pointsInRightWall) return 1;
		else if (loopAroundTunnelCurrentIndex < pointsInGround + pointsInRightWall + pointsInRoof) return 2;
		break;
	case IntersectionType::RightLeft:
	case IntersectionType::All:
		if (loopAroundTunnelCurrentIndex < pointsInGround) return 0;
		else if (loopAroundTunnelCurrentIndex < pointsInGround + pointsInLeftWall) return 2;
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
		if (surfaceIndex == 2) return loopAroundTunnelCurrentIndex; // Roof
		else if (surfaceIndex == 3) return loopAroundTunnelCurrentIndex - pointsInRoof; // Left 
		else if (surfaceIndex == 0) return loopAroundTunnelCurrentIndex - pointsInRoof - pointsInLeftWall; // Ground
		break;
	case IntersectionType::Left:
		if (surfaceIndex == 0) return loopAroundTunnelCurrentIndex; // Ground
		else if (surfaceIndex == 1) return loopAroundTunnelCurrentIndex - pointsInGround; // Right 
		else if (surfaceIndex == 2) return loopAroundTunnelCurrentIndex - pointsInGround - pointsInRightWall; // Roof
		break;
	case IntersectionType::RightLeft:
	case IntersectionType::All:
		if (surfaceIndex == 0) return loopAroundTunnelCurrentIndex; // Ground
		else if (surfaceIndex == 2) return loopAroundTunnelCurrentIndex - pointsInGround; // Roof 
		break;
	}
	return 0;
}

// Returns vertice used for creating tunnel shape
FVector AProceduralIntersection::GetVertice()
{
	FVector vertice;

	if (forwarLoopIndex == 0)
	{
		int32 arrayIndex = GetArrayIndex();
		TArray<FVector>* targetArray = nullptr;

		switch (surfaceIndex)
		{
		case 0:
			targetArray = &(parentTunnel->lastFloorVertices);
			break;
		case 1:
			targetArray = &(parentTunnel->lastRightVertices);
			break;
		case 2:
			targetArray = &(parentTunnel->lastRoofVertices);
			break;
		case 3:
			targetArray = &(parentTunnel->lastLeftVertices);
			break;
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
		sideWaysMovementAmount = groundVerticeSize * (float)(loopAroundTunnelCurrentIndex - pointsInRoof - pointsInLeftWall);
		UE_LOG(LogTemp, Warning, TEXT("The float value is: %f"), sideWaysMovementAmount);

		if (forwarLoopIndex == pointsInGround - 1)
		{
			FVector verticeOffset = FVector(0.0f, (float)pointsInRoof / 2.0f * groundVerticeSize, 0.0f);
			SetAndReturnFirstVertice(FVector((float)forwarLoopIndex * groundVerticeSize, 0.0f, 0.0f) - verticeOffset);
		}
		else if (loopAroundTunnelCurrentIndex == pointsInRoof + pointsInLeftWall) {
			SetAndReturnFirstVertice(latestVertice);
		}
		break;
	case IntersectionType::Left:
	case IntersectionType::RightLeft:
	case IntersectionType::All:
		sideWaysMovementAmount = groundVerticeSize * loopAroundTunnelCurrentIndex;
		if (loopAroundTunnelCurrentIndex == 0)
		{
			FVector verticeOffset = FVector(0.0f, (float)pointsInRoof / 2.0f * groundVerticeSize, 0.0f);
			SetAndReturnFirstVertice(latestVertice - verticeOffset);
		}
		break;
	}

	// Calculate the position of the vertice.
	float deformAmount = GetPixelValue(forwarLoopIndex, loopAroundTunnelCurrentIndex);
	float amountOfDeform = FMath::Lerp(0.0f, deformAmount, surfaceVariation.X);
	float deform = FMath::RandRange(amountOfDeform * -20, amountOfDeform * 20);

	return firstVertice + FVector(0.0f, sideWaysMovementAmount, -deform);
}

FVector AProceduralIntersection::SetAndReturnFirstVertice(FVector value)
{
	firstVertice = value;
	return value;
}

// Get the right vertice for the left side intersection
FVector AProceduralIntersection::GetRightVertice()
{
	// If this is the first vertice in the intersection, set it as the starting point.
	if(loopAroundTunnelCurrentIndex == pointsInGround)
	{
		firstVertice = latestVertice;
		return latestVertice;
	}

	// Calculate the location of the vertice along the wall and the amount of roundness based on that location.
	float locationOnWall = (float)(loopAroundTunnelCurrentIndex - pointsInGround) / (float)pointsInRightWall;
	float roundnessAmount = roundnessCurve->GetFloatValue(locationOnWall);
	float tunnelRounding = FMath::Lerp(tunnelRoundValue, 0.0f, roundnessAmount);

	// Calculate the amount of deform based on the texture value at this point.
	float maxValue = FMath::Lerp(0.0f, maxDeform, surfaceVariation.Y);
	float deform = GetPixelValue(forwarLoopIndex, loopAroundTunnelCurrentIndex) * maxValue;

	// Calculate the position of the vertice.
	FVector vertice = firstVertice + FVector(0.0f, 0.0f, wallVerticeSize * (float)(loopAroundTunnelCurrentIndex - pointsInGround));
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
	float locationOnXRoof = (float)forwarLoopIndex / (float)(pointsInGround - 1);
	float roofXRoundness = roundnessCurve->GetFloatValue(locationOnXRoof);
	float roofYRoundness;
	switch (intersectionType) 
	{
	case IntersectionType::Right:
		locationOnYRoof = (float)(loopAroundTunnelCurrentIndex + 1) / (float)pointsInRoof;
		roofYRoundness = roundnessCurve->GetFloatValue(locationOnYRoof);
		if (locationOnYRoof >= 0.5) roundness = roofYRoundness;
		else roundness = FMath::Min(roofYRoundness, roofXRoundness);
		break;
	case IntersectionType::Left:
		locationOnYRoof = (float)(loopAroundTunnelCurrentIndex - pointsInGround - pointsInRightWall + 1 ) / (float)pointsInRoof;
		roofYRoundness = roundnessCurve->GetFloatValue(locationOnYRoof);
		if (locationOnYRoof < 0.5) roundness = roofYRoundness;
		else roundness = FMath::Min(roofYRoundness, roofXRoundness);
		break;
	case IntersectionType::RightLeft:
		locationOnYRoof = (float)(loopAroundTunnelCurrentIndex - pointsInGround + 1) / (float)pointsInRoof;
		roofYRoundness = roundnessCurve->GetFloatValue(locationOnYRoof);
		if (locationOnXRoof >= 0.5) roundness = roofXRoundness;
		else roundness = FMath::Min(roofYRoundness, roofXRoundness);
		break;
	case IntersectionType::All:
		locationOnYRoof = (float)(loopAroundTunnelCurrentIndex - pointsInGround + 1) / (float)pointsInRoof;
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
			float yLocation = ((float)(pointsInRoof) / 2.0f) * groundVerticeSize - groundVerticeSize;
			selectedVector = latestVertice + FVector(0.0f, yLocation, ((float)pointsInLeftWall) * wallVerticeSize);
			firstVertice = selectedVector;
		}
		else {
			// Otherwise, move on the Y axis.
			selectedVector = firstVertice - FVector(0.0f, (float)(loopAroundTunnelCurrentIndex * groundVerticeSize), 0.0f);
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
		if(loopAroundTunnelCurrentIndex == pointsInGround + pointsInRightWall)
		{
			firstVertice = latestVertice;
		}

		FVector selectedVector = firstVertice;
		float sideWaysMovementSize = (float)(loopAroundTunnelCurrentIndex - pointsInGround - pointsInRightWall + 1) * groundVerticeSize;
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
		if(loopAroundTunnelCurrentIndex == pointsInGround)
		{
			firstVertice = latestVertice + FVector(0.0f, 0.0f, pointsInLeftWall * wallVerticeSize);
		}

		FVector selectedVector = firstVertice;
		float sideWaysMovementSize = (float)(loopAroundTunnelCurrentIndex - pointsInGround + 1) * groundVerticeSize;
		selectedVector.Y -= sideWaysMovementSize;

		selectedVector.Z += tunnelRounding / 2; // ADD ROUNDNESS

		return selectedVector;
	}
}

// This function returns the position of the left vertice in the right side intersection.
FVector AProceduralIntersection::GetLeftVertice()
{
	// Set the position of the first vertex if we are on the last vertice of the wall.
	if (loopAroundTunnelCurrentIndex == pointsInRoof) {
		firstVertice = latestVertice;
	}

	// Calculate the location of the vertice on the wall.
	float locationOnWall = (float)(loopAroundTunnelCurrentIndex - pointsInRoof + 1) / (float)pointsInLeftWall;

	// Calculate the amount of roundness and tunnel rounding.
	float roundnessAmount = roundnessCurve->GetFloatValue(locationOnWall);
	float tunnelRounding = FMath::Lerp(tunnelRoundValue, 0.0f, roundnessAmount);

	// Calculate the amount of deform.
	float maxValue = FMath::Lerp(0.0f, maxDeform, surfaceVariation.Y);
	float deform = GetPixelValue(forwarLoopIndex, loopAroundTunnelCurrentIndex) * maxValue;

	// Calculate the position of the vertice.
	FVector vertice = firstVertice - FVector(0.0f, 0.0f, wallVerticeSize * (loopAroundTunnelCurrentIndex - pointsInRoof + 1));
	vertice.Y -= tunnelRounding; // ADD ROUNDNESS
	vertice.Y -= deform;      // ADD DEFORM

	// Set the ZValue to the height of the last vertex of the left side of the intersection.
	/*if (locationOnWall == 1.0f)
	{
		vertice = FVector(vertice.X, vertice.Y, parentTunnel->lastLeftVertices[parentTunnel->lastLeftVertices.Num() - 1].Z);
	}*/

	// Return the position vector of the vertice.
	return vertice;
}