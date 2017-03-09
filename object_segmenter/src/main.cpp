#include <iostream>
#include <vector>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/filters/extract_indices.h>

using namespace std;

typedef pcl::PointXYZRGB PointInT;
typedef pcl::PointCloud<PointInT>::Ptr PointInTPtr;


// Parameters
std::string indices_file;
std::string input_cloud_pcd;

void parseCommandLine(int argc, char** argv);

void readFromFile(const std::string file_name, std::vector<int>& data);

void segmentObject(PointInTPtr in, PointInTPtr& out, pcl::PointIndices::Ptr indices);

// Methods actual

void parseCommandLine(int argc, char **argv)
{
    pcl::console::parse_argument(argc, argv, "--indices", indices_file);
}

void readFromFile(const string file_name, std::vector<int>& data)
{
    std::ifstream in;
    in.open(file_name.c_str(), std::ifstream::in);

    if(!in)
    {
        std::cout << "Cannot open file.\n";
        exit(0);
    }

    if(in.is_open())
    {
        while(in.good())
        {
            std::string line;
            getline(in, line);

            if(atoi(line.c_str()))
            {
                int idx = atoi(line.c_str());
                data.push_back(idx);
            }
        }
        in.close();
    }

}

void segmentObject(PointInTPtr in, PointInTPtr& out, pcl::PointIndices::Ptr indices)
{
    pcl::ExtractIndices<PointInT> eifilter(true);
    eifilter.setInputCloud(in);
    eifilter.setIndices(indices);
    eifilter.filter(*out);
}

int main(int argc, char** argv)
{
    PointInTPtr scene_cloud;
    PointInTPtr object_cloud;

    scene_cloud.reset(new pcl::PointCloud<PointInT>());
    object_cloud.reset(new pcl::PointCloud<PointInT>());

    parseCommandLine(argc, argv);
    pcl::io::loadPCDFile(argv[1], *scene_cloud);

    PCL_INFO("Scene cloud has %d points\n", (int)scene_cloud->points.size());

    std::vector<int> indices;
    readFromFile(indices_file, indices);

    for(size_t i = 0; i < indices.size(); i++)
    {
        PCL_INFO("index at %d is %d\n", (int)i, indices.at(i));
    }

    pcl::PointIndices::Ptr point_indices (new pcl::PointIndices());
    point_indices->indices.resize(indices.size());

    for(size_t i = 0; i < indices.size(); i++)
    {
        point_indices->indices[i] = indices[i];
    }

    for(size_t i = 0; i < indices.size(); i++)
    {
        PCL_INFO("point index at %d is %d\n", (int)i, point_indices->indices[i]);
    }

    segmentObject(scene_cloud, object_cloud, point_indices);

    PCL_INFO("Object cloud has %d points\n", (int)object_cloud->points.size());

    std::stringstream ss;

    std::vector<std::string> strs;
    boost::split(strs, argv[1], boost::is_any_of("_"));
    ss << "object_" << strs[1];
    std::string object_pcd = ss.str();

    pcl::io::savePCDFileBinary(object_pcd.c_str(), *object_cloud);

    PCL_INFO("Object cloud was saved in %s\n", object_pcd.c_str());

    return 0;
}
