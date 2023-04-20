// Fill out your copyright notice in the Description page of Project Settings.


#include "MyTunnel.h"


AMyTunnel::AMyTunnel()
{
	PrimaryActorTick.bCanEverTick = false;
	ProceduralMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("ProceduralMesh"));
	RootComponent = ProceduralMesh;

	Spline = CreateDefaultSubobject<USplineComponent>(TEXT("Spline"));
	Spline->SetupAttachment(RootComponent);
	Spline->bEditableWhenInherited = true;
}

void AMyTunnel::BeginPlay()
{
	Super::BeginPlay();
}

void AMyTunnel::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	UpdateTunnel(Width, Height, 0.2f, nullptr, 50, Subsections); // Default values
}

void AMyTunnel::UpdateTunnel(float TunnelWidth, float TunnelHeight, float SurfaceVariation, UTexture2D* SurfaceTexture, int32 Resolution, int32 SubsectionCount)
{
	// Determine the starting spline point to update.
	int32 StartSplinePoint = FMath::Max(0, Spline->GetNumberOfSplinePoints() - 4);

	// Generate mesh sections for each spline segment with subsections.
	for (int32 i = StartSplinePoint; i < Spline->GetNumberOfSplinePoints() - 1; ++i)
	{
		for (int32 j = 0; j < Subsections; ++j)
		{
			// Calculate the start and end alpha for the current subsection.
			float StartAlpha = (static_cast<float>(i) + (static_cast<float>(j) / SubsectionCount));
			float EndAlpha = (static_cast<float>(i) + ((static_cast<float>(j) + 1) / SubsectionCount));

			// Generate a unique mesh section index using the spline point index and subsection index.
			int32 MeshSectionIndex = i * Subsections + j;

			GenerateMeshSection(MeshSectionIndex, i, TunnelWidth, TunnelHeight, SurfaceVariation, SurfaceTexture, Resolution + 1, StartAlpha, EndAlpha);
			SectionCount++;
		}
	}
}

void AMyTunnel::GenerateMeshSection(int32 MeshSectionIndex, int32 SplineIndex, float TunnelWidth, float TunnelHeight, float SurfaceVariation, UTexture2D* SurfaceTexture, int32 Resolution, float StartAlpha, float EndAlpha)
{
	// Generate vertices, normals, UVs, and tangents for the current spline segment.
	TArray<FVector> Vertices;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<FProcMeshTangent> Tangents;
	GenerateSectionVertices(SplineIndex, StartAlpha, EndAlpha, Width, Height, Resolution, Vertices, Normals, UVs, Tangents);

	// Generate triangles for the outer walls.
	TArray<int32> Triangles;
	for (int32 i = 0; i < Resolution; ++i)
	{
		int32 VertexIndex = i * 2;

		Triangles.Add(VertexIndex);
		Triangles.Add(VertexIndex + 1);
		Triangles.Add(VertexIndex + 2);

		Triangles.Add(VertexIndex + 1);
		Triangles.Add(VertexIndex + 3);
		Triangles.Add(VertexIndex + 2);
	}

	// Generate triangles for the bottom.
	int32 NumBottomVertices = (Resolution + 1) * 2;
	int32 Offset = (Resolution + 1) * 2;
	for (int32 i = 0; i < Resolution - 1; ++i)
	{
		int32 VertexIndex = i * 2;

		Triangles.Add(Offset + VertexIndex);
		Triangles.Add(Offset + VertexIndex + 2);
		Triangles.Add(Offset + VertexIndex + 1);

		Triangles.Add(Offset + VertexIndex + 1);
		Triangles.Add(Offset + VertexIndex + 2);
		Triangles.Add(Offset + VertexIndex + 3);
	}

	// Create the mesh section and enable collision.
	ProceduralMesh->CreateMeshSection(MeshSectionIndex, Vertices, Triangles, Normals, UVs, TArray<FColor>(), Tangents, true);
	// Set the material of the new section.
	ProceduralMesh->SetMaterial(MeshSectionIndex, TunnelMaterial);

}

void AMyTunnel::GenerateSectionVertices(int32 SplineIndex, float StartAlpha, float EndAlpha, float TunnelWidth, float TunnelHeight, int32 Resolution, TArray<FVector>& Vertices, TArray<FVector>& Normals, TArray<FVector2D>& UVs, TArray<FProcMeshTangent>& Tangents)
{
	// Get the starting and ending spline points using the alpha values.
	FVector StartLocation = Spline->GetLocationAtSplineInputKey(StartAlpha, ESplineCoordinateSpace::Local);
	FVector EndLocation = Spline->GetLocationAtSplineInputKey(EndAlpha, ESplineCoordinateSpace::Local);

	// Calculate the forward, right, and up vectors for both spline points.
	FVector StartForward = Spline->GetDirectionAtSplineInputKey(StartAlpha, ESplineCoordinateSpace::Local);
	FVector EndForward = Spline->GetDirectionAtSplineInputKey(EndAlpha, ESplineCoordinateSpace::Local);
	FVector StartRight = FVector::CrossProduct(StartForward, FVector::UpVector).GetSafeNormal();
	FVector EndRight = FVector::CrossProduct(EndForward, FVector::UpVector).GetSafeNormal();
	FVector StartUp = FVector::CrossProduct(StartForward, StartRight).GetSafeNormal();
	FVector EndUp = FVector::CrossProduct(EndForward, EndRight).GetSafeNormal();

	// Generate vertices for the tunnel based on the custom shape.
	for (int32 i = 0; i <= Resolution; ++i)
	{
		float Ratio = static_cast<float>(i) / Resolution;
		float VerticalAngle = FMath::Lerp(0.0f, PI, Ratio);

		// Calculate the height and width based on the vertical angle
		float CustomHeight = Height * FMath::Sin(VerticalAngle);
		float CustomWidth = Width * 0.5f * FMath::Cos(VerticalAngle);

		FVector StartRadialOffset = StartRight * CustomWidth - StartUp * CustomHeight;
		FVector EndRadialOffset = EndRight * CustomWidth - EndUp * CustomHeight;

		Vertices.Add(StartLocation + StartRadialOffset);
		Normals.Add(-StartRadialOffset.GetSafeNormal());
		UVs.Add(FVector2D(Ratio, 1.0f));

		Vertices.Add(EndLocation + EndRadialOffset);
		Normals.Add(-EndRadialOffset.GetSafeNormal());
		UVs.Add(FVector2D(Ratio, 0.0f));

		Tangents.Add(FProcMeshTangent(StartForward, false));
		Tangents.Add(FProcMeshTangent(EndForward, false));
	}

	// Add bottom vertices
	for (int32 i = 0; i <= Resolution; ++i)
	{
		float Ratio = static_cast<float>(i) / Resolution;
		float HorizontalAngle = Ratio * PI * 2.0f;

		FVector StartRadialOffset = StartRight * (Width * 0.5f * FMath::Cos(HorizontalAngle));
		FVector EndRadialOffset = EndRight * (Width * 0.5f * FMath::Cos(HorizontalAngle));

		Vertices.Add(StartLocation + StartRadialOffset);
		Normals.Add(-StartUp);
		UVs.Add(FVector2D(Ratio, 1.0f));
		Tangents.Add(FProcMeshTangent(StartForward, false));

		Vertices.Add(EndLocation + EndRadialOffset);
		Normals.Add(-EndUp);
		UVs.Add(FVector2D(Ratio, 0.0f));
		Tangents.Add(FProcMeshTangent(EndForward, false));
	}
}

void AMyTunnel::AddIntersection()
{
	float BranchingAngle = 0.0f; // Adjust this value as needed
	AMyTunnel* IntersectionTunnel = CreateIntersection(BranchingAngle);
}

AMyTunnel* AMyTunnel::CreateIntersection(float BranchingAngle)
{
    // Determine the spline point where the intersection should be added.
    int32 IntersectionSplineIndex = FMath::RandRange(1, Spline->GetNumberOfSplinePoints() - 3);

    // Calculate the intersection point and tangent at the chosen spline point.
    FVector IntersectionLocation = Spline->GetLocationAtSplinePoint(IntersectionSplineIndex, ESplineCoordinateSpace::World);
    FVector IntersectionTangent = Spline->GetTangentAtSplinePoint(IntersectionSplineIndex, ESplineCoordinateSpace::World);
    FVector IntersectionForward = Spline->GetDirectionAtSplinePoint(IntersectionSplineIndex, ESplineCoordinateSpace::World);

    // Calculate the forward and right vectors for the new tunnel.
    FVector NewForward = IntersectionForward.RotateAngleAxis(BranchingAngle, FVector::UpVector);
    FVector NewRight = FVector::CrossProduct(NewForward, FVector::UpVector).GetSafeNormal();

    // Get the transform at the chosen spline point.
    FTransform IntersectionTransform = Spline->GetTransformAtSplinePoint(IntersectionSplineIndex, ESplineCoordinateSpace::World);

    // Calculate the new rotation aligned with the NewForward direction.
    FRotator NewRotation = NewForward.Rotation();

    // Spawn a new tunnel instance and set its location and rotation.
    FActorSpawnParameters SpawnParams;
    AMyTunnel* IntersectionTunnel = GetWorld()->SpawnActor<AMyTunnel>(GetClass(), IntersectionTransform.GetLocation(), NewRotation, SpawnParams);

    // Clear the spline points of the new tunnel and add the intersection point.
    IntersectionTunnel->Spline->ClearSplinePoints();
    IntersectionTunnel->Spline->AddSplinePoint(IntersectionLocation, ESplineCoordinateSpace::World, false);

    // Add additional spline points for the new tunnel.
    for (int32 i = 1; i <= 3; ++i)
    {
        FVector NewLocation = IntersectionLocation + NewForward * (i * Spline->GetDefaultUpVector(ESplineCoordinateSpace::World).Size());
        IntersectionTunnel->Spline->AddSplinePoint(NewLocation, ESplineCoordinateSpace::World, false);
    }

    // Update the spline points and refresh the tunnel mesh.
    IntersectionTunnel->Spline->UpdateSpline();
    IntersectionTunnel->UpdateTunnel(Width, Height, 0.2f, nullptr, 50, Subsections);

    return IntersectionTunnel;
}