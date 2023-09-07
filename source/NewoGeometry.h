#ifndef NEWO_GEOMETRY_H
#define NEWO_GEOMETRY_H

#include "NewoCommon.h"
#include "NewoMath.h"
#include "NewoLinearMath.h"
#include "DynoDraw.h"

internal inline vec3
VecSphericalToCartesian(f32 Theta, f32 Phi)
{
    // NOTE: Using coordinate system ZXY; Theta = Angle from axis Z in direction of X (CCW);
    // Phi = Angle from axis Y in direction ZX plane (CW, from top to down)
    vec3 Result = {};

    f32 ThetaRads = DegreesToRadians(Theta);
    f32 PhiRads = DegreesToRadians(Phi);

    Result.X = SinF32(PhiRads) * SinF32(ThetaRads);
    Result.Y = CosF32(PhiRads);
    Result.Z = SinF32(PhiRads) * CosF32(ThetaRads);

    return Result;
}

struct plane
{
    vec3 Normal; // Plane normal. Points X on the plane satisfy DotProduct(Normal, X) = Distance
    f32 Distance; // Distance from origin; Distance = DotProduct(Normal, P) for a given point P on the plane
};

struct aabb
{
    vec3 Center;
    vec3 Extents;
};

struct sphere
{
    vec3 Center;
    f32 Radius;
};

struct obb
{
    vec3 Center;
    vec3 Axes[3];
    vec3 Extents;
};

struct capsule
{
    vec3 Start;
    vec3 End;
    f32 Radius;
};

struct lozenge
{
    vec3 Origin;
    vec3 Edges[2];
    f32 Radius;
};

// NOTE: Kay-Kajiya slab-based volumes. Ericson05 - p.116
struct slab
{
    vec3 Normal;
    f32 DistNear;
    f32 DistFar;
};

// NOTE: Discrete-orientation Polytopes (k-DOPs). Ericson - p.117-122
struct dop8
{
    f32 Min[4];
    f32 Max[4];
};

struct rect
{
    vec3 Center;
    vec3 Axes[2];
    vec2 Extents;
};

#define DEBUG_VIZ_DOTS 64
#define DEBUG_VIZ_VECTORS 64
// Don't know if this is a good idea
#define DEBUG_VIZ_NONE 0
struct debug_viz_data
{
    vec3 Dots[DEBUG_VIZ_DOTS];
    vec3 DotColors[DEBUG_VIZ_DOTS];
    u32 DotsUsed;

    vec3 VectorStarts[DEBUG_VIZ_VECTORS];
    vec3 VectorEnds[DEBUG_VIZ_VECTORS];
    vec3 VectorColors[DEBUG_VIZ_VECTORS];
    u32 VectorsUsed;
};

bool
TestAABBAABB(aabb A, aabb B, debug_viz_data *VizData);

aabb
GetAABBForPointSet(vec3 *Points, u32 PointCount, debug_viz_data *VizData);

// Transform A by Transform and Translation, calculate new axis-aligned
// extents and store in B
void
GetAABBForOrientedBox(aabb A, mat3 Transform, vec3 Translation, aabb *Out_B, debug_viz_data *VizData);

bool
TestSphereSphere(sphere A, sphere B, debug_viz_data *VizData);

// Calculate bounding sphere for a point set by finding a sphere from the 2 most separated
// points on AABB bounding a set of points and expanding it until it envelops all points in the set
// Ritter90 -- Ericson05 - p. 128
sphere
GetBoundingSphereForPointSetRitter(vec3 *Points, u32 PointCount, debug_viz_data *VizData);

// Calculate the bounding sphere for a point set by finding the axis of maximum spread using eigenvalues.
// Constructing a sphere on thathen expanding it until it envelops all points in the set.
// Aka Wu's algorithm. Also something more that I found: https://ep.liu.se/ecp/034/009/ecp083409.pdf
// Ericson05 - p.93
// NOTE: Further research on PCA (principal component analysis) - Jolliffe02
sphere
GetBoundingSphereForPointSetRitterEigen(vec3 *Points, u32 PointCount, debug_viz_data *VizData);

// Calculate bounding sphere for a point set by using the ritter method, then trying to minimize it by shrinking
// the sphere and re-extending it to fit all points in a number of iterations
// Ericson05 - p.99
sphere
GetBoundingSphereForPointSetRitterIterative(vec3 *Points, u32 PointCount, debug_viz_data *VizData);

// Exact minimum sphere using Welzl algorithm.
// Not implementing it now, as I don't know how to compute exact sphere for 0-4 support points.
// And the algorithm doesn't seem that useful for real-time game applications.
// Ericson05 - p.100
// sphere
// WelzlSphere(vec3 *Points, u32 PointCount, vec3 SetOfSupportPoints, u32 SetOfSupportPointCount);

bool
TestOBBOBB(obb A, obb B, debug_viz_data *VizData);

// NOTE: For more info on computing tight OBBs about polyhedra, other references, see Ericson05 p. 108-112

bool
TestSphereCapsule(sphere S, capsule C, debug_viz_data *VizData);

bool
TestCapsuleCapsule(capsule A, capsule B, debug_viz_data *VizData);

vec3
ClosestPointPointSegment(vec3 Point, vec3 Start, vec3 End, f32 *Out_T);

f32
DistSqPointSegment(vec3 Point, vec3 Start, vec3 End);

vec3
ClosestPointPointAABB(vec3 Point, aabb AABB);

f32
DistSqPointAABB(vec3 Point, aabb AABB);

vec3
ClosestPointPointOBB(vec3 Point, obb OBB);

f32
DistSqPointOBB(vec3 Point, obb OBB);

f32
ClosestPointSegmentSegment(vec3 AStart, vec3 AEnd, vec3 BStart, vec3 BEnd,
                               f32 *Out_S, f32 *Out_T, vec3 *Out_PointOnA, vec3 *Out_PointOnB);

vec3
ClosestPointPointRect(vec3 Point, rect Rect);

vec3
ClosestPointPointRect(vec3 Point, vec3 A, vec3 B, vec3 C);

vec3
ClosestPointPointTriangle(vec3 Point, vec3 A, vec3 B, vec3 C);

vec3
ClosestPointPointTetrahedron(vec3 Point, vec3 A, vec3 B, vec3 C, vec3 D);

// Test whether a plane intersects a sphere
bool
TestSpherePlane(sphere Sphere, plane Plane);

// Test whether a sphere is fully behind (inside negative halfspace of) a plane
bool
IsInsideSpherePlane(sphere Sphere, plane Plane);

// Test whether a sphere intersects negative halfspace of a plane
bool
TestSphereHalfspace(sphere Sphere, plane Plane);

// Test if an OBB intersects a plane
bool
TestOBBPlane(obb B, plane P);

// Test if an AABB intersects a plane
bool
TestAABBPlane(aabb B, plane P);

// Test if a sphere intersects an AABB and return the closest point on AABB
bool
TestSphereAABB(sphere S, aabb B, vec3 *Out_ClosestPointOnAABB);

// Test if a sphere intersects an OBB and return the closest point on OBB
bool
TestSphereOBB(sphere S, obb B, vec3 *Out_ClosestPointOnOBB);

// Test if a sphere intersects a triangle and return the closest point on triangle
bool
TestSphereTriangle(sphere S, vec3 A, vec3 B, vec3 C, vec3 *Out_ClosestPointOnTri);

// Test if a triangle intersects a box. Note, this is the most general SAT test, without any optimizations.
// NOTE: This expects non-degenerate primitives only for now.
// TODO: Write an optimized version for AABB (maybe even OBB can be optimized) as demonstrated in Ericson05 - 5.2.9
bool
TestTriangleBox(vec3 A, vec3 B, vec3 C, vec3 BoxCenter, vec3 BoxExtents, vec3 *BoxAxes, debug_viz_data *VizData);

// Test if two triangles intersect. Note, this is the most general unoptimized SAT test.
// NOTE: This expects non-coplanar and non-degenerate triangles only for now.
// TODO: Write an optimized and more robust version of this when needed.
// E.g. Moller's interval overlap method: http://web.archive.org/web/19990203013328/http://www.acm.org/jgt/papers/Moller97/tritri.html
bool
TestTriangleTriangle(vec3 A, vec3 B, vec3 C, vec3 D, vec3 E, vec3 F, debug_viz_data *VizData);

// Test if a line segment intersects with a plane, and return the point of intersection. Ericson05 - 5.3.1
bool
IntersectSegmentPlane(vec3 A, vec3 B, plane P, f32 *Out_T, vec3 *Out_Q);

// Test if a line segment intersects with a plane, and return the point of intersection. 
// Takes 3 non-coplanar points for a plane. Ericson05 - 5.3.1
bool
IntersectSegmentPlane(vec3 A, vec3 B, vec3 D, vec3 E, vec3 F, f32 *Out_T, vec3 *Out_Q);

// Test if a ray intersects with a sphere, and return the point of intersection.
// Ray direction (D) has to be normalized.
// Ericson05 - 5.3.2
bool
IntersectRaySphere(vec3 P, vec3 D, sphere S, f32 *Out_T, vec3 *Out_Q);

// Test if a ray intersects with a sphere.
// More optimized because it doesn't calculate the point of intersection, and has early exits.
// Ray direction (D) has to be normalized.
// Ericson05 - 5.3.2
bool
TestRaySphere(vec3 P, vec3 D, sphere S);

// Test if a ray intersects with an AABB.
// Ray direction (D) has to be normalized.
// Ericson05 - 5.3.3
bool
IntersectRayAABB(vec3 P, vec3 D, aabb A, f32 *Out_TMin, vec3 *Out_Q);

// Test if a line segment intersects with an AABB using SAT
// Ericson05 - 5.3.3
bool
TestSegmentAABB(vec3 P0, vec3 P1, aabb B);

// Test if a line intersects with a triangle ABC, and return the barycentric coordinates of the intersection
// Ericson05 - 5.3.4
bool
IntersectLineTriangle(vec3 P, vec3 Q, vec3 A, vec3 B, vec3 C, f32 *Out_U, f32 *Out_V, f32 *Out_W);


// Test if a line intersects with a quad ABCD, and return the intersection point
// Ericson05 - 5.3.5
bool
IntersectLineQuad(vec3 P, vec3 Q, vec3 A, vec3 B, vec3 C, vec3 D, vec3 *Out_IntersectionPoint);

// Test if a line segment PQ intersects with a triangle ABC, and return barycentric coordinates of the intersection
bool
IntersectSegmentTriangle(vec3 P, vec3 Q, vec3 A, vec3 B, vec3 C, f32 *Out_U, f32 *Out_V, f32 *Out_W, f32 *Out_T);

// Test if a ray intersects with triangle ABC, and return barycentric coordinates of the intersection
bool
IntersectRayTriangle(vec3 P, vec3 RayDir, vec3 A, vec3 B, vec3 C, f32 *Out_U, f32 *Out_V, f32 *Out_W, f32 *Out_T);

// Test if a segment intersects with cylinder, and return T at which the intersection occurs
bool
IntersectSegmentCylinder(vec3 SegA, vec3 SegB, vec3 CylP, vec3 CylQ, f32 CylR, f32 *Out_T);

#endif
