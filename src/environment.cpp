
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors
// for socket communication
#include <unistd.h> 
#include <stdio.h> 
#include <sys/socket.h> 
#include <stdlib.h> 
#include <netinet/in.h> 
#define PORT 8080 
#include <typeinfo>

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <fstream>
#include <iostream>
#include <string> 
int numb =0;
const int MAX = 3;
ofstream fout;  // Create Object of Ofstream
ifstream fin;
// extern int main_socket;
// extern int new_socket;
using namespace std;
int valread;
int  var[MAX]={};
//initialize the socket 
int init_socket(int port_number);
int init_socket(int port_number)
{	int server_fd, new_socket, valread; 
    struct sockaddr_in address; 
    int opt = 1; 
    int addrlen = sizeof(address); 
    char buffer[1024] = {0}; 
    // char *hello = "Hello from server"; 

    // Creating socket file descriptor 
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
    { 
        perror("socket failed"); 
        exit(EXIT_FAILURE); 
    } 
    
    // Forcefully attaching socket to the port 8080 
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, 
                                                  &opt, sizeof(opt))) 
    { 
        perror("setsockopt"); 
        exit(EXIT_FAILURE); 
    } 
    address.sin_family = AF_INET; 
    address.sin_addr.s_addr = INADDR_ANY; 
    address.sin_port = htons( PORT ); 
       
    // Forcefully attaching socket to the port 8080 
    if (bind(server_fd, (struct sockaddr *)&address,  
                                 sizeof(address))<0) 
    { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
    if (listen(server_fd, 3) < 0) 
    { 
        perror("listen"); 
        exit(EXIT_FAILURE); 
    } 

    if ((new_socket = accept(server_fd, (struct sockaddr *)&address,  
                       (socklen_t*)&addrlen))<0) 
    { 
        perror("accept"); 
        exit(EXIT_FAILURE); 
    } 
    cout <<"Connected to server"<<endl;
    // return server_fd;
    
    // return(server_fd);       
}

// send a value through socket communication
// void send(int size, int value)
// {	// valread = read( new_socket , buffer, 1024); 
//     // printf("%s",buffer ); 
//     send(main_socket , &value , value , 0 ); 
//     // printf("Hello message sent");
//     // printf("%i ",&value ); 
// }



/* cityBlock function is the main function that does operations to identify objects in the scene,
 * following operations are performed:
 * Filter: Crop the scene to requested dimensions and remove the points from the roof top
 * Segmentation: Segment the scene to create to clouds one for road and another for objects.
 * Clustering: Identify the clusters from objects
 * Bounding Box: Append a bounding box for each of the clustered object
 * */
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
	// char *hello = "Hello from server";
    // ofstream fout;  // Create Object of Ofstream
    // ifstream fin;
    //std::cout<<"count ="<<numb<<std::endl;
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
	/*Filter: Crop the scene to requested dimensions and remove the points from the roof top*/
	filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.03, Eigen::Vector4f (7, -11, -2, 1), Eigen::Vector4f ( 30, 15, 20, 1));
	/*Segmentation: Segment the scene to create to clouds one for road and another for objects.*/
	std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud,100, 0.09);
	// std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI> *p = &segmentCloud;
	std::cerr << "Model inliers: " << &segmentCloud.first << std::endl;
    // std::cerr << "Model inliers: " << segmentCloud.second->points.size() << std::endl;
    
    for(std::size_t i = 0; i < segmentCloud.first->points.size (); ++i)
    {	
    	// int  var[MAX] = ;
    	// cout<<"size of first line" <<sizeof(segmentCloud.first->points[i])<<std::endl;
    	// int len =sizeof(segmentCloud.first->points[i]);
    	// char buffer[1024] = {0}; 
    	// valread = read( new_socket , buffer, 1024); 
    	// cout<<"%s"<<buffer<<endl ;
    	// send(new_socket , p,len , 0 ); 
    	// cout<<"size of first line" <<sizeof(segmentCloud.first->points[i])<<std::endl;
    //    std::cerr<<"inside loop"<<segmentCloud.first->points[i] << std::endl;
       // fin.open("/media/tejal/Academic/grid_on_image/obst_point/"+std::to_string(numb)+".txt");
       // fout.open("/media/tejal/Academic/grid_on_image/obst_point/"+std::to_string(numb)+".txt",ios::app);
       // if(fin.is_open())
         // fout<< segmentCloud.first->points[i] << std::endl;
    	// var[MAX] = segmentCloud.first->points[i];
    	// cout<<" data save into the array"<< var<<std::endl;
    	cout<< typeid(segmentCloud.first->points[i]).name() << std::endl;

    //    cout<<"\n Data has been appended to file";
       // fin.close();
       // fout.close();
       
    }
    // for(std::size_t i = 0; i < segmentCloud.second->points.size (); ++i)
    // {  
    // //    std::cerr<<"inside loop"<<segmentCloud.second->points[i] << std::endl;
    //    fin.open("/media/tejal/Academic/grid_on_image/road_plane/"+std::to_string(numb)+".txt");
    //    fout.open("/media/tejal/Academic/grid_on_image/road_plane/"+std::to_string(numb)+".txt",ios::app);
    //    if(fin.is_open())
    //    { fout<< segmentCloud.second->points[i] << std::endl;
    //     //  cout<<"\n Data has been appended to file";
    //    }
    //    fin.close();
    //    fout.close();
       
    // }
    /*Render the road to the viewer*/
	renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,1));
	/*Clustering: Identify the clusters from objects*/
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering_euclideanCluster(segmentCloud.second, 0.07,40, 3000);

	int clusterId = 0;
	std::vector<Color> colors = {Color(0,1,1), Color(0,1,1), Color(0,1,1)};
	/*Render the objects along with bounding boxes to the viewer*/
	for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
	{
	    // std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);

        Box box = pointProcessorI->BoundingBox(cluster);
        // std::cout << "Max x: " << box.x_min << std::endl;
        
        // fin.open("/media/tejal/Academic/grid_on_image/road_box/"+std::to_string(numb)+".txt");
        // fout.open("/media/tejal/Academic/grid_on_image/road_box/"+std::to_string(numb)+".txt",ios::app);
        // // std::cout<<" ++++++++++";
        // if(fin.is_open())
        // { 
        //   fout<<box.x_min<<" "<<box.y_max<<" "<<box.z_min<<" ";
        //   fout<<box.x_max<<" "<<box.y_min<<" "<<box.z_max<<std::endl;
        //   std::cout << "Max x: " << box.x_min << std::endl;
        // }
        // fin.close();
        // fout.close(); // Closing the file
        
        renderBox(viewer,box,clusterId);

        ++clusterId;
	}
	++numb;
	auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // cout<<"length of hello:"<<strlen(hello)<<endl;
    	//std::cout << "total fps for one frame " << elapsedTime.count() << " milliseconds and found "<< std::endl;
    
}
//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{   
	int num_porta = 8080;                                
	// init_socket(PORT);
	
    //std::cout << "starting enviroment" << std::endl;
    std::cout << PCL_VERSION << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("/media/taju/Academic/lidar_obstacle/velo_pcd_sync/");
    // fin.open("file_names.txt");
    // fout.open ("file_names.txt",ios::app);
    
    // if(fin.is_open())
    //     for(int i=0;i<(int)stream.size();i++)
    //         fout<<stream[i]<< std::endl;
    // fin.close();
    // fout.close(); // Closing the file
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI(new pcl::PointCloud<pcl::PointXYZI>) ;

    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    int i=0;
    
    while (i<=107)
    {
      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();
      //delete pointProcessorI;

      // Load pcd and run obstacle detection process
      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      //renderPointCloud(viewer,inputCloudI,"Example");

      /*Call cityBlock to identify the objects*/
      cityBlock(viewer, pointProcessorI, inputCloudI);
      // socket()
      
      // Increment to next scene and if the last scene is reached loop back to first scene.
      streamIterator++;
      if(streamIterator == stream.end())
        streamIterator = stream.begin();
      viewer->spinOnce ();
      //delete inputCloudI;
      i++;
    }

}
