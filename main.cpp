#include "include/utils.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <random>

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

void generatePointCloudObj()
{
    const int numPoints = 100;
    const float rangeMin = -1.0f;
    const float rangeMax = 1.0f;

    std::ofstream objFile("../input_files/cloud.obj");
    if (!objFile.is_open())
    {
        std::cerr << "Erro ao abrir o arquivo para escrita.\n";
        return;
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(rangeMin, rangeMax);

    for (int i = 0; i < numPoints; ++i)
    {
        float x = dist(gen);
        float y = dist(gen);
        float z = dist(gen);
        objFile << "v " << x << " " << y << " " << z << "\n";
    }

    objFile.close();
    std::cout << "Arquivo 'pontos.obj' gerado com sucesso.\n";
    return;
}

int main()
{

    // std::string inputFile = "../input_files/cube.obj";
    // std::string outputFile = "../output_files/cube_hull.obj";

    // std::string inputFile = "../input_files/tetrahedron2.obj";
    // std::string outputFile = "../output_files/tetrahedron2_hull.obj";

    // std::string inputFile = "../input_files/tetrahedron.obj";
    // std::string outputFile = "../output_files/tetrahedron_hull.obj";

    // generatePointCloudObj();
    // std::string inputFile = "../input_files/cloud.obj";
    // std::string outputFile = "../output_files/cloud_hull.obj";

    std::string inputFile = "../input_files/canopy.obj";
    std::string outputFile = "../output_files/canopy_hull.obj";

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