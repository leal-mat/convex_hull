#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <queue>
#include <tuple>
#include <algorithm>

namespace geometryUtils
{

    struct Point3D
    {
        double x, y, z;
        Point3D(double x, double y, double z) : x(x), y(y), z(z) {}
    };

    struct Triangle
    {
        int a, b, c;
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

    bool allSameSide(const std::vector<Point3D> &points, const Triangle &t)
    {
        Point3D n = normal(points[t.a], points[t.b], points[t.c]);
        int sign = 0;
        const double epsilon = 1e-8; // Tolerance for floating-point comparisons

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
        int n = points.size();
        if (n < 3)
            throw std::runtime_error("Not enough points for a triangle.");

        // 1. Find the point with the lowest x coordinate
        int i0 = 0;
        for (int i = 1; i < n; ++i)
        {
            if (points[i].x < points[i0].x)
                i0 = i;
        }

        // 2. Find the farthest point from i0
        int i1 = (i0 == 0) ? 1 : 0;
        double maxDist = 0;
        for (int i = 0; i < n; ++i)
        {
            if (i == i0)
                continue;
            double dist = std::pow(points[i].x - points[i0].x, 2) +
                          std::pow(points[i].y - points[i0].y, 2) +
                          std::pow(points[i].z - points[i0].z, 2);
            if (dist > maxDist)
            {
                maxDist = dist;
                i1 = i;
            }
        }

        // 3. Find a third point not collinear with i0 and i1
        int i2 = -1;
        for (int i = 0; i < n; ++i)
        {
            if (i == i0 || i == i1)
                continue;
            Point3D nrm = cross(subtract(points[i1], points[i0]), subtract(points[i], points[i0]));
            if (dot(nrm, nrm) > 1e-8)
            {
                i2 = i;
                break;
            }
        }
        if (i2 == -1)
            throw std::runtime_error("All points are collinear.");

        // 4. Orient the triangle so the normal points outward
        Point3D centroid = {0, 0, 0};
        for (const auto &p : points)
        {
            centroid.x += p.x;
            centroid.y += p.y;
            centroid.z += p.z;
        }
        centroid.x /= n;
        centroid.y /= n;
        centroid.z /= n;

        Point3D nrm = normal(points[i0], points[i1], points[i2]);
        Point3D toCentroid = subtract(centroid, points[i0]);
        if (dot(nrm, toCentroid) > 0)
        {
            std::swap(i1, i2);
        }

        return std::make_tuple(i0, i1, i2);
    }

    std::vector<Triangle> giftWrapping(const std::vector<Point3D> &points)
    {
        std::vector<Triangle> faces;
        std::set<std::pair<int, int>> processedEdges;
        std::queue<std::pair<int, int>> edgeQueue;
        std::set<std::tuple<int, int, int>> uniqueTriangles;

        // Step 1: Find the initial triangle
        auto [i0, i1, i2] = findInitialTriangle(points);
        std::cout << "Initial triangle: (" << i0 << ", " << i1 << ", " << i2 << ")\n";

        // Step 2: Add the initial triangle's edges to the queue
        edgeQueue.push({i0, i1});
        edgeQueue.push({i1, i2});
        edgeQueue.push({i2, i0});

        // Step 3: Process edges to expand the hull
        while (!edgeQueue.empty())
        {
            auto edge = edgeQueue.front();
            edgeQueue.pop();
            int a = edge.first;
            int b = edge.second;

            std::cout << "Processing edge: (" << a << ", " << b << ")\n";

            // Skip if the edge has already been processed
            if (processedEdges.count(edge))
                continue;
            processedEdges.insert(edge);

            int c = -1;
            double maxDist = -1e9;

            // Find the point `c` that forms the most outward triangle with edge (a, b)
            for (int i = 0; i < points.size(); ++i)
            {
                if (i == a || i == b)
                    continue;

                Triangle t = {a, b, i};
                if (!allSameSide(points, t))
                {
                    std::cout << "Triangle (" << a << ", " << b << ", " << i << ") is not valid.\n";
                    continue;
                }

                Point3D n = normal(points[a], points[b], points[i]);
                double normLen = std::sqrt(dot(n, n));
                if (normLen < 1e-8)
                    continue;

                double dist = std::abs(dot(n, subtract(points[i], points[a]))) / normLen;
                if (dist > maxDist)
                {
                    maxDist = dist;
                    c = i;
                }
            }

            if (c == -1)
            {
                std::cout << "No valid third point found for edge (" << a << ", " << b << ").\n";
                continue;
            }

            // Add the triangle if it's unique
            std::vector<int> indices = {a, b, c};
            std::sort(indices.begin(), indices.end());
            auto key = std::make_tuple(indices[0], indices[1], indices[2]);

            if (uniqueTriangles.count(key) == 0)
            {
                uniqueTriangles.insert(key);
                faces.push_back({a, b, c});
                std::cout << "Adding triangle: (" << a << ", " << b << ", " << c << ")\n";

                // Add the new edges to the queue
                std::pair<int, int> edge1 = {std::min(a, c), std::max(a, c)};
                std::pair<int, int> edge2 = {std::min(b, c), std::max(b, c)};

                if (!processedEdges.count(edge1))
                {
                    std::cout << "Adding edge: (" << edge1.first << ", " << edge1.second << ")\n";
                    edgeQueue.push(edge1);
                }
                if (!processedEdges.count(edge2))
                {
                    std::cout << "Adding edge: (" << edge2.first << ", " << edge2.second << ")\n";
                    edgeQueue.push(edge2);
                }
            }
        }

        return faces;
    }

} // namespace geometryUtils