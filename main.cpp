#include "include/utils.hpp"
#include <fstream>
#include <sstream>

std::vector<geometryUtils::Point3D> loadObj(std::string filename)
{
    std::vector<geometryUtils::Point3D> points;
    std::ifstream file(filename);
    if (!file)
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return points;
    }
    std::string line;

    while (getline(file, line))
    {
        if (line.rfind("v ", 0) == 0)
        {
            std::istringstream iss(line.substr(2));
            double x, y, z;
            if (iss >> x >> y >> z)
            {
                points.push_back(geometryUtils::Point3D(x, y, z));
            }
        }
    }
    return points;
}

void writeObj(std::string filename, std::vector<geometryUtils::Point3D> hull_points, std::vector<geometryUtils::Triangle> hull_faces)
{
    std::ofstream file(filename);
    if (!file)
    {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    for (auto vertex_line : hull_points)
    {
        file << "v " << vertex_line.x << " " << vertex_line.y << " " << vertex_line.z << "\n";
    }

    for (auto faces_line : hull_faces)
    {

        file << "f " << (faces_line.a + 1) << " " << (faces_line.b + 1) << " " << (faces_line.c + 1) << "\n";
    }
}

int main()
{

    std::string inputFile = "../input_files/cube.obj";
    std::string outputFile = "../output_files/cube_hull.obj";

    auto points = loadObj(inputFile);
    if (points.size() < 4)
    {
        std::cerr << "Not enough points to build a 3D convex hull.\n";
        return 1;
    }

    auto faces = geometryUtils::giftWrapping(points);
    std::cout << "Number of faces in the convex hull: " << faces.size() << "\n";
    writeObj(outputFile, points, faces);

    std::cout << "Convex hull written to " << outputFile << "\n";
    return 0;
}