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
	}
	//parentTunnel = parent;
	isUpdate = update;

	groundVerticeSize = (localScale.X * 100.0f) / (float)(pointsInWidth - 1);
	wallVerticeSize = (localScale.Y * 100.0f) / (float)(pointsInHeight);
	loopAroundTunnelLastIndex = (pointsInWidth * 2) + (pointsInHeight * 2) + pointAdjustment;
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
void AProceduralIntersection::RightSideIntersection() 
{
	// Forward loop is the size of points in widht
	for (forwarLoopIndex = 0; forwarLoopIndex <= pointsInWidth; forwarLoopIndex++)
	{
		latestVertice = FVector((float)forwarLoopIndex * groundVerticeSize, 0.0f, 0.0f);
		// Around tunnel loop
		for (aroundLoopIndex = 1; aroundLoopIndex <= loopAroundTunnelLastIndex; aroundLoopIndex++)
		{
			GetSurfaceIndex();
			if (surfaceIndex != 3) // This is just lazynes. We could make loop shorter 
			{
				if (surfaceIndex != 2)
				{
					latestVertice = GetVertice();
					wallVertices.Add(latestVertice);
					if(aroundLoopIndex == 1)
					{
						lastRightRoofVertices.Add(latestVertice);
					}
					if(surfaceIndex == 1 && forwarLoopIndex == pointsInWidth)
					{
						lastLeftWallVertices.Add(latestVertice);
					}
					if(surfaceIndex == 0 && forwarLoopIndex == pointsInWidth)
					{
						lastStraightRoofVertices.Add(latestVertice);
					}
					float x = (float)forwarLoopIndex / (float)pointsInWidth;
					float y = (float)aroundLoopIndex / (float)((loopAroundTunnelLastIndex - pointsInHeight) - pointsInWidth);
					wallUV.Add(FVector2D(x, y));
				}
				else 
				{
					latestVertice = GetVertice();
					groundVertices.Add(latestVertice);
					if(aroundLoopIndex == pointsInWidth*2 + pointsInHeight + pointAdjustment)
					{
						lastRightFloorVertices.Add(latestVertice);
					}
					if(surfaceIndex == 2 && forwarLoopIndex == pointsInWidth)
					{
						lastStraightFloorVertices.Add(latestVertice);
					}

					float x = (float)forwarLoopIndex / (float)pointsInWidth;
					float y = (float)(aroundLoopIndex - (pointsInWidth + pointsInHeight)) / (float)((loopAroundTunnelLastIndex - pointsInHeight) - pointsInHeight - pointsInWidth);
					groundUV.Add(FVector2D(x, y));
				}
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

// Generate Intersection to left and straight
void AProceduralIntersection::LeftSideIntersection() 
{
	// Forward loop is the size of points in widht
	for (forwarLoopIndex = 0; forwarLoopIndex <= pointsInWidth; forwarLoopIndex++)
	{
		latestVertice = FVector((float)forwarLoopIndex * groundVerticeSize, 0.0f, 0.0f);
		// Around tunnel loop
		for (aroundLoopIndex = 1; aroundLoopIndex <= loopAroundTunnelLastIndex; aroundLoopIndex++)
		{
			GetSurfaceIndex();
			if (surfaceIndex != 3) // This is just lazynes. We could make loop shorter 
			{
				if (surfaceIndex != 0)
				{
					latestVertice = GetVertice();
					wallVertices.Add(latestVertice);
					if(aroundLoopIndex == (pointsInWidth*2+pointsInHeight+pointAdjustment))
					{
						lastLeftRoofVertices.Add(latestVertice);
					}
					if(surfaceIndex == 2 && forwarLoopIndex == pointsInWidth)
					{
						lastStraightRoofVertices.Add(latestVertice);
					}
					if(surfaceIndex == 1 && forwarLoopIndex == pointsInWidth)
					{
						lastRightWallVertices.Add(latestVertice);
					}
					float x = (float)forwarLoopIndex / (float)pointsInWidth;
					float y = (float)(aroundLoopIndex - pointsInWidth)/ (float)((loopAroundTunnelLastIndex - pointsInHeight) - pointsInWidth);
					wallUV.Add(FVector2D(x, y));
				}
				else 
				{
					latestVertice = GetVertice();
					groundVertices.Add(latestVertice);
					if(aroundLoopIndex == 1)
					{
						lastLeftFloorVertices.Add(latestVertice);
					}
					if(surfaceIndex == 0 && forwarLoopIndex == pointsInWidth)
					{
						lastStraightFloorVertices.Add(latestVertice);
					}
					float x = (float)forwarLoopIndex / (float)pointsInWidth;
					float y = (float)(aroundLoopIndex / pointsInWidth);
					groundUV.Add(FVector2D(x, y));
				}
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

// Generate Intersection to right and left 
void AProceduralIntersection::RightLeftIntersection() 
{
	// Forward loop is the size of points in widht
	for (forwarLoopIndex = 0; forwarLoopIndex <= pointsInWidth; forwarLoopIndex++)
	{
		bool xRounded = false;
		latestVertice = FVector((float)forwarLoopIndex * groundVerticeSize, 0.0f, 0.0f);
		loopAroundTunnelLastIndex = pointsInWidth * 2 + pointAdjustment;
		// If we are on the end of intersection generation
		// Then we do the last line of roof vertices 
		// And after that we do the wall in the end of 'T' intersection
		if (forwarLoopIndex == pointsInWidth)
		{
			loopAroundTunnelLastIndex = (pointsInWidth + 1) * (pointsInHeight + 2); 
		}
		for (aroundLoopIndex = 1; aroundLoopIndex <= loopAroundTunnelLastIndex; aroundLoopIndex++)
		{
			surfaceIndex = 1;
			if (aroundLoopIndex <= pointsInWidth + 1)
			{
				surfaceIndex = 0;
			}
			if (surfaceIndex == 1)
			{
				latestVertice = GetVertice();
				wallVertices.Add(latestVertice);
				if(forwarLoopIndex == pointsInWidth)
				{
					lastStraightRoofVertices.Add(latestVertice);
				}
				if(aroundLoopIndex == pointsInWidth * 2 + pointAdjustment)
				{
					lastLeftRoofVertices.Add(latestVertice);
				}
				if(aroundLoopIndex == pointsInWidth + 2)
				{
					lastRightRoofVertices.Add(latestVertice);
				}
				if((aroundLoopIndex - 1) % (pointsInWidth + 1) == 0 && forwarLoopIndex == pointsInWidth && aroundLoopIndex > 27)
				{
					lastRightWallVertices.Add(latestVertice);
				}
				if((aroundLoopIndex - 1) % (pointsInWidth + 1) == 25 && forwarLoopIndex == pointsInWidth && aroundLoopIndex > 52)
				{
					lastLeftWallVertices.Add(latestVertice);
				}
				float x = (float)forwarLoopIndex / (float)pointsInWidth;
				float y = (float)(aroundLoopIndex - pointsInWidth)/ (float)(pointsInWidth);
				wallUV.Add(FVector2D(x, y));
				xRounded = true;
			}
			else 
			{
				latestVertice = GetVertice();
				groundVertices.Add(latestVertice);
				if(forwarLoopIndex == pointsInWidth)
				{
					lastStraightFloorVertices.Add(latestVertice);
				}
				if(aroundLoopIndex == pointsInWidth + 1)
				{
					lastRightFloorVertices.Add(latestVertice);
				}
				if(aroundLoopIndex == 1)
				{
					lastLeftFloorVertices.Add(latestVertice);
				}
				float x = (float)forwarLoopIndex / (float)pointsInWidth;
				float y = (float)(aroundLoopIndex / pointsInWidth);
				groundUV.Add(FVector2D(x, y));
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

// Generate Intersection to right, left and straight
void AProceduralIntersection::AllSidesIntersection() 
{
	// Forward loop is the size of points in widht
	for (forwarLoopIndex = 0; forwarLoopIndex <= pointsInWidth; forwarLoopIndex++)
	{
		bool xRounded = false;
		latestVertice = FVector((float)forwarLoopIndex * groundVerticeSize, 0.0f, 0.0f);
		loopAroundTunnelLastIndex = pointsInWidth * 2 + pointAdjustment;
		// Around tunnel loop
		for (aroundLoopIndex = 1; aroundLoopIndex <= loopAroundTunnelLastIndex; aroundLoopIndex++)
		{
			surfaceIndex = 1;
			if (aroundLoopIndex <= pointsInWidth + 1)
			{
				surfaceIndex = 0;
			}
			if (surfaceIndex == 1)
			{
				latestVertice = GetVertice();
				wallVertices.Add(latestVertice);
				if(forwarLoopIndex == pointsInWidth)
				{
					lastStraightRoofVertices.Add(latestVertice);
				}
				if(aroundLoopIndex == loopAroundTunnelLastIndex)
				{
					lastLeftRoofVertices.Add(latestVertice);
				}
				if(aroundLoopIndex == pointsInWidth + 2)
				{
					lastRightRoofVertices.Add(latestVertice);
				}
				float x = (float)forwarLoopIndex / (float)pointsInWidth;
				float y = (float)(aroundLoopIndex - pointsInWidth)/ (float)(pointsInWidth);
				wallUV.Add(FVector2D(x, y));
				xRounded = true;
			}
			else 
			{
				latestVertice = GetVertice();
				groundVertices.Add(latestVertice);
				if(forwarLoopIndex == pointsInWidth)
				{
					lastStraightFloorVertices.Add(latestVertice);
				}
				if(aroundLoopIndex == pointsInWidth + 1)
				{
					lastRightFloorVertices.Add(latestVertice);
				}
				if(aroundLoopIndex == 1)
				{
					lastLeftFloorVertices.Add(latestVertice);
				}
				float x = (float)forwarLoopIndex / (float)pointsInWidth;
				float y = (float)(aroundLoopIndex / pointsInWidth);
				groundUV.Add(FVector2D(x, y));
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

// Get index that tells us in what surface we are
void AProceduralIntersection::GetSurfaceIndex()
{
	// If equal or less 26
	if (aroundLoopIndex <= pointsInWidth + 1)
	{
		surfaceIndex = 0;
	} // If under 47 and over 26
	else if (aroundLoopIndex < pointsInHeight + pointsInWidth + pointAdjustment && aroundLoopIndex > pointsInWidth + 1)
	{
		surfaceIndex = 1;
	} // If under 72 and over or equal to 47
	else if (aroundLoopIndex <= pointsInWidth * 2 + pointsInHeight + pointAdjustment && aroundLoopIndex >= pointsInHeight + pointsInWidth + pointAdjustment)
	{
		surfaceIndex = 2;
	}
	else 
	{
		surfaceIndex = 3;
	}
}

// Return vertice
FVector AProceduralIntersection::GetVertice() 
{
	// Check if this is the first loop index
	if (forwarLoopIndex == 0)
	{
		// Initialize variables
		int32 arrayIndex;
		FVector vertice;

		// Determine which surface this intersection belongs to
		switch (surfaceIndex)
		{
		case 0:
			arrayIndex = aroundLoopIndex - 1;
			if(intersectionType == IntersectionType::Right) 
			{
				arrayIndex = FMath::Clamp(arrayIndex, 0, parentTunnel->lastRoofVertices.Num() - 1);
				vertice = parentTunnel->lastRoofVertices[arrayIndex];
			}
			else if (intersectionType == IntersectionType::Left) 
			{
				arrayIndex = FMath::Clamp(arrayIndex, 0, parentTunnel->lastFloorVertices.Num() - 1);
				vertice = parentTunnel->lastFloorVertices[arrayIndex];
			}
			else if (intersectionType == IntersectionType::All || intersectionType == IntersectionType::RightLeft) 
			{
				arrayIndex = aroundLoopIndex - 1;
				arrayIndex = FMath::Clamp(arrayIndex, 0, parentTunnel->lastFloorVertices.Num() - 1);
				vertice = parentTunnel->lastFloorVertices[arrayIndex];
			}
			
			break;
		case 1:
			arrayIndex = aroundLoopIndex - (pointsInWidth + pointAdjustment);
			if(intersectionType == IntersectionType::Right) 
			{
				arrayIndex = FMath::Clamp(arrayIndex, 0, parentTunnel->lastLeftVertices.Num() - 1);
				vertice = parentTunnel->lastLeftVertices[arrayIndex];
			}
			else if (intersectionType == IntersectionType::Left) 
			{
				arrayIndex = FMath::Clamp(arrayIndex, 0, parentTunnel->lastRightVertices.Num() - 1);
				vertice = parentTunnel->lastRightVertices[arrayIndex];
			}
			else if (intersectionType == IntersectionType::All || intersectionType == IntersectionType::RightLeft) 
			{
				arrayIndex = aroundLoopIndex - (pointsInWidth + pointAdjustment);
				arrayIndex = FMath::Clamp(arrayIndex, 0, parentTunnel->lastRoofVertices.Num() - 1);
				vertice = parentTunnel->lastRoofVertices[arrayIndex];
			}
			break;
		case 2:
			arrayIndex = aroundLoopIndex - (pointsInWidth + pointsInHeight + pointAdjustment);
			if(intersectionType == IntersectionType::Right) 
			{
				arrayIndex = FMath::Clamp(arrayIndex, 0, parentTunnel->lastFloorVertices.Num() - 1);
				vertice = parentTunnel->lastFloorVertices[arrayIndex]; //this crashes
			}
			else if (intersectionType == IntersectionType::Left) 
			{
				arrayIndex = FMath::Clamp(arrayIndex, 0, parentTunnel->lastRoofVertices.Num() - 1);
				vertice = parentTunnel->lastRoofVertices[arrayIndex]; //this crashes
			}
			break;
		case 3:
			return FVector(0.0f,0.0f,0.0f);
			break;
		}
		vertice = UKismetMathLibrary::TransformLocation(parentTunnel->GetTransform(), vertice); 							   // FLIP FROM CONNECTED ACTORS TRANSFORM TO WORLD TRANSFORM
		vertice = UKismetMathLibrary::InverseTransformLocation(this->GetTransform(), vertice);	
		return vertice;
	}
	else 
	{
		FVector vertice;
		switch (surfaceIndex)
		{
		case 0:
			if(intersectionType == IntersectionType::Right) 
			{
				vertice = GetRoofVertice();
			}
			else if (intersectionType == IntersectionType::Left) 
			{
				vertice = GetFloorVertice();
			}
			else if (intersectionType == IntersectionType::All) 
			{
				vertice = GetFloorVertice();
			}
			else if (intersectionType == IntersectionType::RightLeft) 
			{
				vertice = GetFloorVertice();
			}
			break;
		case 1:
			if(intersectionType == IntersectionType::Right) 
			{
				vertice = GetLeftVertice();
			}
			else if (intersectionType == IntersectionType::Left) 
			{
				vertice = GetRightVertice();
			}
			else if (intersectionType == IntersectionType::All) 
			{
				vertice = GetRoofVertice();
			}
			else if (intersectionType == IntersectionType::RightLeft) 
			{
				vertice = GetRightLeftIntersectionRoof();
			}
			break;
		case 2:
			if(intersectionType == IntersectionType::Right) 
			{
				vertice = GetFloorVertice();
			}
			else if (intersectionType == IntersectionType::Left) 
			{
				vertice = GetRoofVertice();
			}
			break;
		case 3:
			vertice = FVector(0.0f,0.0f,0.0f);
			break;
		}
		return vertice;
	}
}

// Get the floor vertice for the intersection
FVector AProceduralIntersection::GetFloorVertice()
{
	float sideWaysMovementAmount = 0.0f;

	// Calculate the amount of sideways movement based on the intersection type.
	switch (intersectionType)
	{
	case IntersectionType::Right: 
		sideWaysMovementAmount = groundVerticeSize * (aroundLoopIndex - (pointsInWidth + pointsInHeight + pointAdjustment));
		// If this is the first vertice in the intersection, set it as the starting point.
		if(aroundLoopIndex == pointsInWidth + pointsInHeight + pointAdjustment)
		{
			firstVertice = latestVertice;
			return latestVertice;
		}
		break;
	case IntersectionType::Left: 
		sideWaysMovementAmount = groundVerticeSize * (aroundLoopIndex - 1);
		if(aroundLoopIndex == 1)
		{
			firstVertice = latestVertice - FVector(0.0f, (float)pointsInWidth / 2.0f * groundVerticeSize, 0.0f);
			return firstVertice;
		}
		break;
	case IntersectionType::All: 
		sideWaysMovementAmount = groundVerticeSize * (aroundLoopIndex - 1);
		if(aroundLoopIndex == 1)
		{
			firstVertice = latestVertice - FVector(0.0f, (float)pointsInWidth / 2.0f * groundVerticeSize, 0.0f);
			return firstVertice;
		}
		break;
	case IntersectionType::RightLeft: 
		sideWaysMovementAmount = groundVerticeSize * (aroundLoopIndex - 1);
		FVector x = FVector(forwarLoopIndex * groundVerticeSize, 0.0f, 0.0f);
		if (isUpdate && aroundLoopIndex == 2) 
		{
			firstVertice = FVector(x.X, latestVertice.Y, latestVertice.Z) + FVector(0.0f, groundVerticeSize, 0.0f);
			return firstVertice;
		} 
		else if (aroundLoopIndex == 1)
		{
			firstVertice = latestVertice - FVector(0.0f, (float)pointsInWidth / 2 * groundVerticeSize, 0.0f);
			return firstVertice;
		}
		break;
	}

	// Calculate the amount of deform based on the texture value at this point.
	float deformAmount = GetPixelValue(forwarLoopIndex, aroundLoopIndex);
	float amountOfDeform = FMath::Lerp(0.0f, deformAmount, surfaceVariation.X);
	float deform = FMath::RandRange(amountOfDeform * -20, amountOfDeform * 20);
	
	// Calculate the position of the vertice.
	FVector vertice = firstVertice + FVector(0.0f, sideWaysMovementAmount, 0.0f);
	vertice = vertice - FVector(0.0f, 0.0f, deform);
	return vertice;
}

// Get the right vertice for the left side intersection
FVector AProceduralIntersection::GetRightVertice()
{
	// If this is the first vertice in the intersection, set it as the starting point.
	if(aroundLoopIndex == (pointsInWidth + pointAdjustment))
	{
		firstVertice = latestVertice;
		return latestVertice;
	}

	// Calculate the location of the vertice along the wall and the amount of roundness based on that location.
	float locationOnWall = (float)(aroundLoopIndex - (pointsInWidth + pointAdjustment)) / (float)pointsInHeight;
	float roundnessAmount = roundnessCurve->GetFloatValue(locationOnWall);
	float tunnelRounding = FMath::Lerp(tunnelRoundValue, 0.0f, roundnessAmount);

	// Calculate the amount of deform based on the texture value at this point.
	float maxValue = FMath::Lerp(0.0f, maxDeform, surfaceVariation.Y);
	float deform = GetPixelValue(forwarLoopIndex, aroundLoopIndex) * maxValue;

	// Calculate the position of the vertice.
	FVector vertice = firstVertice + FVector(0.0f, 0.0f, wallVerticeSize * (float)(aroundLoopIndex - (pointsInWidth + pointAdjustment)));
	vertice += FVector(0.0f, tunnelRounding, 0.0f);
	vertice += FVector(0.0f, deform, 0.0f);

	return vertice;
}

// This function returns the position of the roof vertice in the intersection.
FVector AProceduralIntersection::GetRoofVertice()
{
	// Calculate the location of the vertice on the roof based on the intersection type.
	float locationOnRoof;
	switch (intersectionType) 
	{
	case IntersectionType::Right:
		locationOnRoof = (float)aroundLoopIndex / (float)(pointsInWidth + 1);
		break;
	case IntersectionType::Left:
		locationOnRoof = (float)(aroundLoopIndex - (pointsInWidth + pointsInHeight + 1) ) / (float)(pointsInWidth);
		break;
	case IntersectionType::All:
		locationOnRoof = (float)aroundLoopIndex / (float)(pointsInWidth + 1);
		break;
	}
	
	// Calculate the amount of roundness and tunnel rounding.
	float roundnessAmount = roundnessCurve->GetFloatValue(locationOnRoof); 
	float tunnelRounding = FMath::Lerp(tunnelRoundValue, 0.0f, roundnessAmount);

	// Calculate the position of the vertex based on the intersection type.
	if (intersectionType == IntersectionType::Right) 
	{ 
		FVector selectedVector;

		if (aroundLoopIndex == 1) 
		{
			// If we are at the start of the intersection, set the starting point.
			selectedVector = latestVertice + FVector(0.0f, ((float)pointsInWidth / 2.0f) * groundVerticeSize, (float)pointsInHeight * wallVerticeSize);
			firstVertice = selectedVector;
		}
		else {
			// Otherwise, move on the Y axis.
			selectedVector = firstVertice - FVector(0.0f, (float)((aroundLoopIndex - 1) * groundVerticeSize), 0.0f);
		}

		// Add roundness to the vertice position.
		selectedVector = selectedVector + FVector(0.0f, 0.0f, tunnelRounding); // ADD ROUNDNESS
		
		// Calculate the amount of deform and add it to the vertice position.
		float maxValue = FMath::Lerp(0.0f, maxDeform, surfaceVariation.Y); 
		float deform = GetPixelValue(forwarLoopIndex, aroundLoopIndex) * maxValue; 
		return selectedVector + FVector(0.0f,0.0f, deform); 

	} 
	else if (intersectionType == IntersectionType::Left) // LEFT SIDE INTERSECTION
	{

		FVector selectedVector = latestVertice - FVector(0.0f, groundVerticeSize, 0.0f);
		if(aroundLoopIndex == (pointsInWidth + pointsInHeight + pointAdjustment)) 
		{
			// If we are at the start of the intersection, set the starting point.
			selectedVector = latestVertice + FVector(0.0f,0.0f, wallVerticeSize);
			firstVertice = selectedVector;
		}
		else 
		{
			// Otherwise, move on the Y axis.
			selectedVector = firstVertice - FVector(0.0f, (float)((aroundLoopIndex - (pointsInWidth + pointsInHeight + pointAdjustment)) * groundVerticeSize), 0.0f);
		}

		// Add roundness to the vertex position.
		selectedVector = selectedVector + FVector(0.0f, 0.0f, tunnelRounding);
		
		// Calculate the amount of deform and add it to the vertex position.
		float maxValue = FMath::Lerp(0.0f, maxDeform, surfaceVariation.Y);
		float deform = GetPixelValue(forwarLoopIndex, aroundLoopIndex) * maxValue;
		return selectedVector + FVector(0.0f,0.0f, deform);

	} 
	else // ALL SIDE INTERSECTION
	{
		FVector selectedVector = latestVertice - FVector(0.0f, groundVerticeSize, 0.0f);
		if(aroundLoopIndex - (pointsInWidth + 1) == 1)
		{
			// If we are at the start of the intersection, set the starting point.
			selectedVector = FVector(latestVertice.X, latestVertice.Y, parentTunnel->lastRoofVertices[0].Z);
		}

		// Set the ZValue to the height of the middle vertex of the roof.
		selectedVector = FVector(selectedVector.X, selectedVector.Y, parentTunnel->lastRoofVertices[(parentTunnel->lastRoofVertices.Num()-1) / 2].Z);
		return selectedVector;
	}
}

// This function returns the position of the left vertice in the right side intersection.
FVector AProceduralIntersection::GetLeftVertice()
{
	// Set the position of the first vertex if we are on the last vertice of the wall.
	if (aroundLoopIndex == pointsInWidth + pointAdjustment) {
		firstVertice = latestVertice;
	}

	// Calculate the location of the vertice on the wall.
	float locationOnWall = (float)(aroundLoopIndex - (pointsInWidth + 1)) / (float)pointsInHeight;

	// Calculate the amount of roundness and tunnel rounding.
	float roundnessAmount = roundnessCurve->GetFloatValue(locationOnWall);
	float tunnelRounding = FMath::Lerp(tunnelRoundValue, 0.0f, roundnessAmount);

	// Calculate the amount of deform.
	float maxValue = FMath::Lerp(0.0f, maxDeform, surfaceVariation.Y);
	float deform = GetPixelValue(forwarLoopIndex, aroundLoopIndex) * maxValue;

	// Calculate the position of the vertice.
	FVector vertice = firstVertice - FVector(0.0f, 0.0f, wallVerticeSize * (aroundLoopIndex - (pointsInWidth + 1)));
	vertice -= FVector(0.0f, tunnelRounding, 0.0f); // ADD ROUNDNESS
	vertice -= FVector(0.0f, deform, 0.0f);      // ADD DEFORM

	// Set the ZValue to the height of the last vertex of the left side of the intersection.
	if(locationOnWall == 1.0f)
	{
		vertice = FVector(vertice.X, vertice.Y, parentTunnel->lastLeftVertices[parentTunnel->lastLeftVertices.Num() - 1].Z);
	}

	// Return the position vector of the vertice.
	return vertice;
}

// This function returns the roof position of the vertice at the intersection that turns right and left but doesnt go straight.
FVector AProceduralIntersection::GetRightLeftIntersectionRoof()
{
	// Initialize variables.
	const float MaxDeform = 1.0f;
	const float RoundingMin = 0.0f;
	const float RoundingMax = 100.0f;


	float deform = 0.0f; // Amount of deformation applied to the vertice
	float rounding = RoundingMin; // Amount of rounding applied to the vertex.

	// Check if we are on the last line of vertices at the end wall.
	const int32 endWallVerticesOffset = (pointsInHeight + 1) * (pointsInWidth + 1);
	int32 x = aroundLoopIndex - endWallVerticesOffset;
	if (x >= 1)
	{ 
		// If we are adding the last vertices of the wall, use floors end vertices to prevent caps between wall and floor.
		return lastStraightFloorVertices[lastStraightFloorVertices.Num() - x];
	} 	//^ Above is done in inversed order

	// Calculate deform.
	const float value = FMath::TruncToFloat((float)aroundLoopIndex / (float)(pointsInWidth + 2));
	if (1.0f < value && value < (float)pointsInHeight)
	{
		const float maxValue = FMath::Lerp(0.0f, MaxDeform, surfaceVariation.Y);
		deform = GetPixelValue(forwarLoopIndex, aroundLoopIndex) * maxValue;
	}	

	// Initialize the position vector of the vertice.
	FVector vector = latestVertice;
	
	// Calculate ZValue.
	float ZValue = latestVertice.Z;
	if (aroundLoopIndex - (pointsInWidth +1) == 1) 
	{
		// Get the height of the first index of the roof.
		ZValue = parentTunnel->lastRoofVertices[(float)(parentTunnel->lastRoofVertices.Num() - 1) / 2.0f].Z;
	} 
	else 
	{ 
		// Don't move sideways on the first vertice.
		vector -= FVector(0.0f, groundVerticeSize, 0.0f);
	}

	// Check if we are on the first end wall vertice.
	if((aroundLoopIndex - 1) % (pointsInWidth + 1) == 0 && aroundLoopIndex - (pointsInWidth + 1) != 1.0f && forwarLoopIndex == pointsInWidth)
	{
		// Calculate rounding.
		const float alpha = abs(((value / (float)pointsInHeight) - 0.5f) * 2.0f);
		rounding = FMath::Lerp(RoundingMax, RoundingMin, alpha);

		// Use the first vertex of the last straight roof.
		vector = lastStraightRoofVertices[0];

		// Calculate ZValue based on the vertex's position.
		const int32 roofVerticesMid = (parentTunnel->lastRoofVertices.Num() - 1) / 2.0f;
		ZValue = parentTunnel->lastRoofVertices[roofVerticesMid].Z - FMath::TruncToInt((float)aroundLoopIndex / (float)(pointsInWidth + 2)) * (parentTunnel->lastRoofVertices[roofVerticesMid].Z / (float)pointsInHeight);
	}
	
	// Calculate XValue and YValue.
	float XValue = vector.X + rounding - deform;
	float YValue = vector.Y;

	// Return the position vector of the vertice.
	return FVector(XValue, YValue, ZValue);
}