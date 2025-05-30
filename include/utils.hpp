#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <queue>
#include <tuple>
#include <algorithm>
#include <limits>
#include <map>

namespace geometryUtils
{

    struct Point3D
    {
        double x, y, z;
        Point3D(double x, double y, double z) : x(x), y(y), z(z) {}
        bool operator==(const Point3D &other) const
        {
            return std::abs(x - other.x) < 1e-8 && std::abs(y - other.y) < 1e-8 && std::abs(z - other.z) < 1e-8;
        }
    };

    struct Triangle
    {
        int a, b, c;
        Triangle(int a, int b, int c) : a(a), b(b), c(c) {}
    };

    Point3D cross(const Point3D &u, const Point3D &v)
    {
        return {
            u.y * v.z - u.z * v.y,
            u.z * v.x - u.x * v.z,
            u.x * v.y - u.y * v.x};
    }

    double dot(const Point3D &a, const Point3D &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    Point3D subtract(const Point3D &u, const Point3D &v)
    {
        return {u.x - v.x, u.y - v.y, u.z - v.z};
    }

    Point3D normal(const Point3D &a, const Point3D &b, const Point3D &c)
    {
        return cross(subtract(b, a), subtract(c, a));
    }

    Point3D normalize(const Point3D &v)
    {
        double length = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        if (length < 1e-10)
            return Point3D(0, 0, 0); // Avoid division by zero
        return Point3D(v.x / length, v.y / length, v.z / length);
    }

    bool allSameSide(const std::vector<Point3D> &points, const Triangle &t)
    {
        Point3D n = normal(points[t.a], points[t.b], points[t.c]);
        int sign = 0;
        const double epsilon = 1e-6; // Tolerance for floating-point comparisons

        for (int i = 0; i < points.size(); ++i)
        {
            if (i == t.a || i == t.b || i == t.c)
                continue;

            Point3D ap = subtract(points[i], points[t.a]);
            double dotProduct = dot(n, ap);

            if (std::abs(dotProduct) < epsilon)
                continue; // Point is on the plane

            if (sign == 0)
            {
                sign = (dotProduct > 0) ? 1 : -1;
            }
            else if ((dotProduct > 0 && sign < 0) || (dotProduct < 0 && sign > 0))
            {
                return false; // Points are on different sides
            }
        }
        return true;
    }

    std::tuple<int, int, int> findInitialTriangle(const std::vector<Point3D> &points)
    {
        int i0 = 0;
        int i1 = 0;
        int i2 = 0;

        for (int i = 0; i < points.size(); ++i)
        {
            if (points.at(i).z < points.at(i0).z)
            {
                i0 = i;
            }
        }

        double min_angle = std::numeric_limits<double>::max();
        for (int i = 0; i < points.size(); ++i)
        {
            if (i == i0)
                continue;
            Point3D v = subtract(points.at(i), points.at(i0));
            double angle = std::atan2(v.z, v.y);
            // dot(subtract(points.at(i), points.at(i0)), subtract(points.at(i), points.at(i0)));
            if (angle < min_angle)
            {
                min_angle = angle;
                i1 = i;
            }
        }

        double max_angle = -std::numeric_limits<double>::max();
        Point3D v1 = subtract(points.at(i1), points.at(i0));
        Point3D aux(0, 0, 1);
        Point3D ortho = cross(v1, aux);
        ortho = normalize(ortho);
        Point3D ref_normal = ortho;
        if (std::abs(v1.x) < 1e-8 && std::abs(v1.y) < 1e-8)
        {
            aux = Point3D(0, 1, 0);
        }
        for (int i = 0; i < points.size(); ++i)
        {
            if (i == i0 || i == i1)
                continue;
            Point3D candidate_normal = cross(v1, subtract(points.at(i), points.at(i0)));
            double norm = std::sqrt(dot(candidate_normal, candidate_normal));
            if (norm < 1e-10)
                continue; // Skip colinear points

            candidate_normal = normalize(candidate_normal);
            double angle = dot(ref_normal, candidate_normal);
            if (angle > max_angle)
            {
                max_angle = angle;
                i2 = i;
            }
        }
        return std::make_tuple(i0, i1, i2);
    }

    bool isFaceExplored(const Triangle &face, std::map<std::pair<int, int>, std::vector<int>> edgeToFaces)
    {
        std::vector<std::pair<int, int>> edges = {
            {std::min(face.a, face.b), std::max(face.a, face.b)},
            {std::min(face.b, face.c), std::max(face.b, face.c)},
            {std::min(face.c, face.a), std::max(face.c, face.a)}};

        for (auto &e : edges)
        {
            if (edgeToFaces[e].size() < 2)
                return false; // Ainda falta uma face adjacente nessa aresta
        }
        return true;
    }

    void printEdgesToFaces(const std::map<std::pair<int, int>, std::vector<int>> &edgeToFaces)
    {
        for (const auto &entry : edgeToFaces)
        {
            const auto &edge = entry.first;
            const auto &faces = entry.second;
            std::cout << "Edge (" << edge.first << ", " << edge.second << ") is shared by faces: ";
            for (int faceIdx : faces)
            {
                std::cout << faceIdx << " ";
            }
            std::cout << std::endl;
        }
    }

    std::vector<Triangle> giftWrapping(const std::vector<Point3D> &points)
    {
        std::vector<Triangle> faces;
        std::set<std::pair<int, int>> processedEdges;
        std::set<std::tuple<int, int, int>> uniqueFaces;
        std::queue<std::pair<int, int>> edgeQueue;
        std::map<std::pair<int, int>, std::vector<int>> edgeToFaces;

        // Step 1: Find the initial triangle
        std::tuple<int, int, int> initialFace = findInitialTriangle(points);

        int i0 = std::get<0>(initialFace);
        int i1 = std::get<1>(initialFace);
        int i2 = std::get<2>(initialFace);
        faces.push_back(Triangle(i0, i1, i2));
        int initialFaceIdx = 0;
        std::vector<std::pair<int, int>> initialEdges = {
            {i0, i1},
            {i1, i2},
            {i2, i0}};
        for (const auto &e : initialEdges)
        {
            edgeToFaces[e].push_back(initialFaceIdx);
        }

        edgeQueue.push({std::get<0>(initialFace), std::get<1>(initialFace)});
        edgeQueue.push({std::get<1>(initialFace), std::get<2>(initialFace)});
        edgeQueue.push({std::get<2>(initialFace), std::get<0>(initialFace)});

        while (!edgeQueue.empty())
        {
            auto edge = edgeQueue.front();
            edgeQueue.pop();
            // processedEdges.insert(edge);
            int a = edge.first;
            int b = edge.second;
            int bestCandidate = -1;

            Point3D p1p2 = subtract(points[b], points[a]);
            double min_cos_theta = 2.0;

            for (int i = 0; i < points.size(); ++i)
            {
                if (i == a || i == b)
                {
                    continue;
                }
                Triangle tri(a, b, i);
                if (isFaceExplored(tri, edgeToFaces))
                {
                    continue; // Skip if the face is already explored
                }
                Point3D p1p = subtract(points.at(i), points.at(a));
                Point3D np = cross(p1p, p1p2);
                auto currentFace = faces.at(faces.size() - 1);
                Point3D n = normal(points.at(currentFace.a), points.at(currentFace.b), points.at(currentFace.c));
                double cos_theta = -dot(np, n) / (std::sqrt(dot(np, np)) * std::sqrt(dot(n, n)));
                bool sameSide = allSameSide(points, tri);
                std::cout << "Edge (" << a << "," << b << "), candidate " << i
                          << ", cos_theta=" << cos_theta << ", allSameSide=" << sameSide << std::endl;
                if (cos_theta < min_cos_theta && sameSide)
                {
                    min_cos_theta = cos_theta;
                    bestCandidate = i;
                }
            }
            if (bestCandidate != -1)
            {

                std::vector<int> tri = {a, b, bestCandidate};

                std::sort(tri.begin(), tri.end());
                auto triTuple = std::make_tuple(tri[0], tri[1], tri[2]);
                if (uniqueFaces.find(triTuple) == uniqueFaces.end())
                {
                    faces.push_back(Triangle(a, b, bestCandidate));
                    uniqueFaces.insert(triTuple);

                    int newFaceIdx = faces.size() - 1;
                    std::vector<std::pair<int, int>> edgesOfNewFace = {
                        {std::min(a, b), std::max(a, b)},
                        {std::min(b, bestCandidate), std::max(b, bestCandidate)},
                        {std::min(bestCandidate, a), std::max(bestCandidate, a)}};

                    for (const auto &e : edgesOfNewFace)
                    {
                        edgeToFaces[e].push_back(newFaceIdx);
                        if (edgeToFaces[e].size() < 2)
                        {
                            edgeQueue.push(e);
                        }
                    }
                }
            }
        }

        printEdgesToFaces(edgeToFaces);

        return faces;
    }

} // namespace geometryUtils