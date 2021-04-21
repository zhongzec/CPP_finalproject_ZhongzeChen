#include "trojanmap.h"
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <algorithm>
#include <fstream>
#include <locale>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
// #include <bits/stdc++.h>
#include <cmath>
#include <iostream>
#include <cctype>
#include <unordered_set>
#include <stack>
#include <chrono>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

//-----------------------------------------------------
// TODO (Students): You do not and should not change the following functions:
//-----------------------------------------------------

/**
 * PrintMenu: Create the menu
 * 
 */
void TrojanMap::PrintMenu() {

  std::string menu =
      "**************************************************************\n"
      "* Select the function you want to execute.                    \n"
      "* 1. Autocomplete                                             \n"
      "* 2. Find the position                                        \n"
      "* 3. CalculateShortestPath                                    \n"
      "* 4. Travelling salesman problem                              \n"
      "* 5. Cycle Detection                                          \n"
      "* 6. Topological Sort                                         \n"
      "* 7. Exit                                                     \n"
      "**************************************************************\n";
  std::cout << menu << std::endl;
  std::string input;
  getline(std::cin, input);
  char number = input[0];
  switch (number)
  {
  case '1':
  {
    menu =
        "**************************************************************\n"
        "* 1. Autocomplete                                             \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a partial location:";
    std::cout << menu;
    getline(std::cin, input);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = Autocomplete(input);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '2':
  {
    menu =
        "**************************************************************\n"
        "* 2. Find the position                                        \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a location:";
    std::cout << menu;
    getline(std::cin, input);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = GetPosition(input);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.first != -1) {
      std::cout << "Latitude: " << results.first
                << " Longitude: " << results.second << std::endl;
      PlotPoint(results.first, results.second);
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '3':
  {
    menu =
        "**************************************************************\n"
        "* 3. CalculateShortestPath                                    \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the start location:";
    std::cout << menu;
    std::string input1;
    getline(std::cin, input1);
    menu = "Please input the destination:";
    std::cout << menu;
    std::string input2;
    getline(std::cin, input2);
    auto start = std::chrono::high_resolution_clock::now();
    auto results = CalculateShortestPath_Dijkstra(input1, input2);
    
    //you can uncomment it to test Bellman. It will take 3-5 min to show the result!!!!!!!!!
    //auto results = CalculateShortestPath_Bellman_Ford(input1, input2);  
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
      std::cout << "The distance of the path is:" << CalculatePathLength(results) << " miles" << std::endl;
      PlotPath(results);
    } else {
      std::cout << "No route from the start point to the destination."
                << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '4':
  {
    menu =
        "**************************************************************\n"
        "* 4. Travelling salesman problem                              \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "In this task, we will select N random points on the map and you need to find the path to travel these points and back to the start point.";
    std::cout << menu << std::endl << std::endl;
    menu = "Please input the number of the places:";
    std::cout << menu;
    getline(std::cin, input);
    int num = std::stoi(input);
    std::vector<std::string> keys;
    for (auto x : data) {
      keys.push_back(x.first);
    }
    std::vector<std::string> locations;
    srand(time(NULL));
    for (int i = 0; i < num; i++)
      locations.push_back(keys[rand() % keys.size()]);
    PlotPoints(locations);
    std::cout << "Calculating ..." << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    auto results = TravellingTrojan(locations);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    CreateAnimation(results.second);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.second.size() != 0) {
      for (auto x : results.second[results.second.size()-1]) std::cout << x << std::endl;
      menu = "**************************************************************\n";
      std::cout << menu;
      std::cout << "The distance of the path is:" << results.first << " miles" << std::endl;
      PlotPath(results.second[results.second.size()-1]);
    } else {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
           "You could find your animation at src/lib/output.avi.          \n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '5':
  {
    menu =
        "**************************************************************\n"
        "* 5. Cycle Detection                                          \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the left bound longitude(between -118.299 and -118.264):";
    std::cout << menu;
    getline(std::cin, input);
    std::vector<double> square;
    square.push_back(atof(input.c_str()));

    menu = "Please input the right bound longitude(between -118.299 and -118.264):";
    std::cout << menu;
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));

    menu = "Please input the upper bound latitude(between 34.011 and 34.032):";
    std::cout << menu;
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));

    menu = "Please input the lower bound latitude(between 34.011 and 34.032):";
    std::cout << menu;
    getline(std::cin, input);
    square.push_back(atof(input.c_str()));

    auto start = std::chrono::high_resolution_clock::now();
    auto results = CycleDetection(square);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results == true)
      std::cout << "there exists cycle in the subgraph " << std::endl;
    else
      std::cout << "there exist no cycle in the subgraph " << std::endl;
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '6':
  {
    menu =
        "**************************************************************\n"
        "* 6. Topological Sort                                         \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the locations filename:";
    std::cout << menu;
    std::string locations_filename;
    getline(std::cin, locations_filename);
    menu = "Please input the dependencies filename:";
    std::cout << menu;
    std::string dependencies_filename;
    getline(std::cin, dependencies_filename);
    
    // Read location names from CSV file
    std::vector<std::string> location_names;
    if (locations_filename == "") 
      location_names = {"Cardinal Gardens", "Coffee Bean1","CVS"};
    else
      location_names = ReadLocationsFromCSVFile(locations_filename);
    
    // Read dependencies from CSV file
    std::vector<std::vector<std::string>> dependencies;
    if (dependencies_filename == "")
      dependencies = {{"Coffee Bean1","Cardinal Gardens"}, {"CVS","Cardinal Gardens"}, {"CVS","Coffee Bean1"}};
    else
      dependencies = ReadDependenciesFromCSVFile(dependencies_filename);

    // std::vector<std::string> location_names = {"Cardinal Gardens", "Coffee Bean1","CVS"};
    // std::vector<std::vector<std::string>> dependencies = {{"Coffee Bean1","Cardinal Gardens"}, {"CVS","Cardinal Gardens"}, {"CVS","Coffee Bean1"}};
    auto start = std::chrono::high_resolution_clock::now();
    auto result = DeliveringTrojan(location_names, dependencies);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    menu = "*************************Results******************************";
    std::cout << menu << std::endl;
    std::cout << "Topological Sorting Reults:" << std::endl;
    for (auto x : result) std::cout << x << std::endl;
    std::vector<std::string> node_ids;
    for (auto x: result) {
      Node node = GetNode(x);
      node_ids.push_back(node.id);
    }
    PlotPointsOrder(node_ids);
    menu = "**************************************************************\n";
    std::cout << menu;
    std::cout << "Time taken by function: " << duration.count() << " microseconds" << std::endl << std::endl;
    PrintMenu();
    break;
  }
  case '7':
    break;
  default:
  {
    std::cout << "Please select 1 - 7." << std::endl;
    PrintMenu();
    break;
  }
  }
}


/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  std::fstream fin;
  fin.open("src/lib/map.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '['), word.end());
      word.erase(std::remove(word.begin(), word.end(), ']'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}

/**
 * PlotPoint: Given a location id, plot the point on the map
 * 
 * @param  {std::string} id : location id
 */
void TrojanMap::PlotPoint(std::string id) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(data[id].lat, data[id].lon);      //在CSV中根据ID定位data的lat和long
  cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}
/**
 * PlotPoint: Given a lat and a lon, plot the point on the map
 * 
 * @param  {double} lat : latitude
 * @param  {double} lon : longitude
 */
void TrojanMap::PlotPoint(double lat, double lon) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(lat, lon);
  cv::circle(img, cv::Point(int(result.first), int(result.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPath: Given a vector of location ids draws the path (connects the points)
 * 
 * @param  {std::vector<std::string>} location_ids : path
 */
void TrojanMap::PlotPath(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < location_ids.size(); i++) {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::line(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPoints(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points inside square
 * @param  {std::vector<double>} square : boundary
 */
void TrojanMap::PlotPointsandEdges(std::vector<std::string> &location_ids, std::vector<double> &square) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto upperleft = GetPlotLocation(square[2], square[0]);
  auto lowerright = GetPlotLocation(square[3], square[1]);
  cv::Point pt1(int(upperleft.first), int(upperleft.second));
  cv::Point pt2(int(lowerright.first), int(lowerright.second));
  cv::rectangle(img, pt2, pt1, cv::Scalar(0, 0, 255));
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    for(auto y : data[x].neighbors) {
      auto start = GetPlotLocation(data[x].lat, data[x].lon);
      auto end = GetPlotLocation(data[y].lat, data[y].lon);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              LINE_WIDTH);
    }
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPointsOrder: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPointsOrder(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::putText(img, data[x].name, cv::Point(result.first, result.second), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 0, 0), 2);
  }
  // Plot dots and lines
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < location_ids.size(); i++) {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::arrowedLine(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}


/**
 * CreateAnimation: Create the videos of the progress to get the path
 * 
 * @param  {std::vector<std::vector<std::string>>} path_progress : the progress to get the path
 */
void TrojanMap::CreateAnimation(std::vector<std::vector<std::string>> path_progress){
  cv::VideoWriter video("src/lib/output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(1248,992));
  for(auto location_ids: path_progress) {
    std::string image_path = cv::samples::findFile("src/lib/input.jpg");
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
    cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
              cv::Scalar(0, 0, 255), cv::FILLED);
    for (auto i = 1; i < location_ids.size(); i++) {
      auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
      auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
      cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
                cv::Scalar(0, 0, 255), cv::FILLED);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              LINE_WIDTH);
    }
    video.write(img);
    cv::imshow("TrojanMap", img);
    cv::waitKey(1);
  }
	video.release();
}
/**
 * GetPlotLocation: Transform the location to the position on the map
 * 
 * @param  {double} lat         : latitude 
 * @param  {double} lon         : longitude
 * @return {std::pair<double, double>}  : position on the map
 */
std::pair<double, double> TrojanMap::GetPlotLocation(double lat, double lon) {
  std::pair<double, double> bottomLeft(34.01001, -118.30000);
  std::pair<double, double> upperRight(34.03302, -118.26502);
  double h = upperRight.first - bottomLeft.first;
  double w = upperRight.second - bottomLeft.second;
  std::pair<double, double> result((lon - bottomLeft.second) / w * 1248,
                                   (1 - (lat - bottomLeft.first) / h) * 992);
  return result;
}

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(std::string id) {
    return data[id].lat;    //返回对应id的latitude
}


/**
 * GetLon: Get the longitude of a Node given its id. 
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(std::string id) { 
    return data[id].lon;   //返回对应id的lon
}

/**
 * GetName: Get the name of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(std::string id) { 
    return data[id].name;
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(std::string id) {
    return data[id].neighbors;  //返回对应id的neighbors
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {Node} a  : node a
 * @param  {Node} b  : node b
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const Node &a, const Node &b) { //node指的是data中string ID对应的node
  // Do not change this function                                    //如：data["123"] 代表id=123对应的node
  // TODO: Use Haversine Formula:
  // dlon = lon2 - lon1;
  // dlat = lat2 - lat1;
  // a = (sin(dlat / 2)) ^ 2 + cos(lat1) * cos(lat2) * (sin(dlon / 2)) ^ 2;
  // c = 2 * arcsin(min(1, sqrt(a)));
  // distances = 3961 * c;

  // where 3961 is the approximate radius of the earth at the latitude of
  // Washington, D.C., in miles
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2),2.0) + cos(a.lat * M_PI / 180.0) * cos(b.lat * M_PI / 180.0) * pow(sin(dlon / 2),2.0);
  double c = 2 * asin(std::min(1.0,sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) {
  double sum = 0;
  int numofplace = path.size();    //Num of places between two places
  if (numofplace<=1)    
  return 0;

  else
  {
    for(int i=0;i<numofplace-1;i++)
    sum = sum + CalculateDistance(data[path[i]],data[path[i+1]]); 
    //分别计算相邻的string ID的distance，然后累加
    //path[i] = string ID. data[path[i]]指data中对应的string ID的Node
  }

  return sum;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */

//用于autocomplete，忽略大小写，比较两个string
bool iequals(const std::string& a, const std::string& b)
{
    unsigned int sz = a.size();
    if (b.size() != sz)
        return false;
    for (unsigned int i = 0; i < sz; ++i)
        if (tolower(a[i]) != tolower(b[i]))
            return false;
    return true;
}

std::vector<std::string> TrojanMap::Autocomplete(std::string name){
  std::vector<std::string> results;
  int prefix_size = name.size();  //prefix length
  
  //first,check if input is empty
  if(name.empty())
  {
  return results;
  std::cout<<"empty result"<<std::endl;
  }

  //else,check the prefix
  else
  {   
    //auto it type -> map<string, Node> data in .h file
    for(auto it=data.begin();it!=data.end();it++)
    {
    //在csv的name中截取和input长度相同的字母，放入temp中。
    //ex:input=ch,则截取full name中的前两个字母
    std::string temp = it->second.name.substr(0,prefix_size);//store prefix into temp
    //std::cout<<"current value is "<<temp<<std::endl; //用来测试

    if (iequals(temp, name))    //调用函数，忽略大小写
    {
    //std::cout<<"find the match"<<std::endl;
    results.push_back(it->second.name);
    }
    }
    return results;
  }
  
}

/**
 * GetPosition: Given a location name, return the position.
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);  //initialization
  //std::map<std::string, Node> it; create iterator with the same type of data
  for(auto it=data.begin();it!=data.end();it++)
  {
    if(it->second.name == name)//if data node has the name matches input parameter
    {   //pass the Node lat and lon to the output
      results.first=it->second.lat;
      results.second=it->second.lon;
    } 
  
  }

  return results;
}

/**
 * GetNode: Given a location name, return the node.
 *
 * @param  {std::string} name          : location name
 * @return {Node}  : node
 */
// Node::Node()
// {
//   id=""; name="";lat=0;lon=0;neighbors={""};
// }

Node TrojanMap::GetNode(std::string name) {
  Node n;
  for(auto it = data.begin();it!=data.end();it++)
  {
    if(it->second.name == name)   //如果csv中的name=输入，则将信息返回给node
    {
       n.id = it->second.id;
       n.lat = it->second.lat;
       n.lon = it->second.lon;
       n.name = it->second.name;
       n.neighbors = it->second.neighbors;
    }
  }
 
  return n;
}






//step3: Dijstra
int FindMinInDButNotInVisited(std::vector<double> &d,std::unordered_set<int> &visited, bool &uncheck)
{
  int min_node;
  double min_dis =INT_MAX;
  for(int i=0;i<d.size();i++)
  {
      auto it = visited.find(i);
      if(it==visited.end())   //if node has not visited,else回到for loop
      {                       
        double cur_dis = d[i];
        if(cur_dis < min_dis) //如果source到该node的distance比之前的dis小，则更新
        {                     //else,min不变
          min_dis = cur_dis;
          min_node = i;
          uncheck = false;
        }
      }     
  }
  return min_node;    //如果返回-1，则说明source和dest没有任何路径
}




//step3: Dijstra
void TrojanMap::weight_matrix(){
  weight =std::vector<std::vector<double> >(data.size(),std::vector<double>(data.size(),INT_MAX)); 
  //std::cout<<"check2"<<std::endl;
  for(int i=0;i<data.size();i++) //weight bewteen the node itself = 0
  {
    weight[i][i] = 0;   

//find the weight of each node between all its neigbour
    for(auto j: data[temp_id[i]].neighbors) 
    { 
      weight[i][temp_index[j]] = CalculateDistance(data[temp_id[i]],data[j]); //datatemp_id[i]=当前node，data[j]=其所有neighbour
      weight[temp_index[j]][i] = weight[i][temp_index[j]];  //adjacency matrix中两个node之间undirected，A->B=B->A
    }
  }
}



void create_path(std::vector<int> &path,std::vector<int> &parent,int node)
{
  if (parent[node] == -1)
  {
    return;
  }
  path.push_back(parent[node]);
  create_path(path, parent, parent[node]);
}



/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path which is a
 * list of id.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
  std::string location1_name, std::string location2_name) 
{
//First, check if two input loactions exist
Node n1 = GetNode(location1_name);
Node n2 = GetNode(location2_name);
//std::cout<<"n1 is "<<n1.id<<" n2 is "<<n2.id<<std::endl;  //测试
if(n1.id==""||n2.id==""||n1.id==n2.id)
{
std::cout<<"invalid location"<<std::endl;   //测试
return {""};
}

//If locations exist: 
//create adjacency matrix:outer vector is num of nodes, 
//inner vector are the weights of a node between all its neigbours

auto it =data.begin();
for(int i=0;i<data.size();i++)
{
  temp_id[i] = it->first;  //每个序号i对应的id:ex temp[0]=id1,temp[1]=id2...
  temp_index[it->first] = i;  //每个id的序号
  it++;       //iterator定位到下一个id位置
}
 weight_matrix();  //相邻node的weight。不相邻的node：weight=max
std::vector<int> parent(data.size(), -1); //存最短path中destination node的所有parent node
std::unordered_set<int> visited;
std::vector<double> d(weight.size());
bool uncheck = false;   //initialize bool value,判断source node到任意一个node存不存在path
std::vector<int> path;  //按dest到source顺序存最短路径的node
  std::cout<<"weight size is"<<weight.size()<<std::endl;
for(int i=0;i<weight.size();i++)
  {    
  d[i] = weight[temp_index[n1.id]][i];  //source node到每个node的distance。此处，和source不相邻的node的weight=max
  }

visited.insert(temp_index[n1.id]);  
while(visited.size()<weight.size() && !uncheck && visited.find(temp_index[n2.id])==visited.end())
  {
  
    uncheck = true;
    //访问unvisited的node，比较source到每个node的距离，找到最短距离的node
    int min_node = FindMinInDButNotInVisited(d,visited,uncheck);  
    visited.insert(min_node);
    for(int i=0;i<weight.size();i++)  //check if source node can reach new node through its neighbor node
    {
      //if so,then update the distance to d
      if(d[min_node] + weight[min_node][i] < d[i]){  
        d[i] = d[min_node]+weight[min_node][i];
        parent[i] = min_node;   //将该node的parent node存起来
      }
//ex: lec7图:if minnode=6,if node0->node3的距离(因为不相邻所以初始=max）小于node0->5->6->3，则更新node0->3的距离  
    }
  }
  //check if the destination node has been visited
  if(visited.find(temp_index[n2.id]) == visited.end()) //if not visited, then there is no path btw two nodes
  return {""};
  //if found dest node,search for the path btw source an dest node
  path.push_back(temp_index[n2.id]);  //首先将dest node传进去
  create_path(path,parent,temp_index[n2.id]);   //按照dest->source顺序存parent nodes
  path.push_back(temp_index[n1.id]);    //最后加上source node

  //当前path vector是{dest,....source}，所以要reverse
  reverse(path.begin(), path.end());
  //path存着node的序号，转化为对应的string id
  std::vector<std::string> path_output;
  for(int i=0;i<path.size();i++)
  {  
    path_output.push_back(temp_id[path[i]]);
  }
    

  return path_output;
} 








/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest path which is a
 * list of id.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name){
  //std::vector<std::string> path;

//First, check if two input loactions exist
Node n1 = GetNode(location1_name);
Node n2 = GetNode(location2_name);
//std::cout<<"n1 is "<<n1.id<<" n2 is "<<n2.id<<std::endl;  //测试
if(n1.id==""||n2.id==""||n1.id==n2.id)
{
std::cout<<"invalid location"<<std::endl;   //测试
return {""};
}

//If locations exist: 
std::cout<<"Running Bellman will take 3-5 minutes to show the result!Please wait"<<std::endl;
auto it =data.begin();
for(int i=0;i<data.size();i++)
{
  temp_id[i] = it->first;  
  temp_index[it->first] = i;  
  it++;       //iterator定位到下一个id位置
}
 weight_matrix();  //相邻node的weight。不相邻的node：weight=max
std::vector<int> parent(data.size(), -1); //存最短path中destination node的所有parent node
std::vector<double> d(weight.size()); //1D table
std::vector<int> path;  //按dest到source顺序存最短路径的node

//std::cout<<"weight size is"<<weight.size()<<std::endl;
for(int i=0;i<weight.size();i++)
  {    
  d[i] = weight[temp_index[n1.id]][i];  //source node到每个node的distance。此处，和source不相邻的node的weight=max
  }
// visited.insert(temp_index[n1.id]);  //先存source node
d[temp_index[n1.id]] = 0;   //source node to itself distance = 0 

for(int i=0;i<weight.size()-1;i++)  //iterate 0->n-1 edges情况 EX:n=2 nodes,so only 1 edge exist
{ 
  //find total number of edges of certain nodes
  for(int v=0;v<weight.size();v++)  //iterate all nodes,then find the incoming edges of these nodes
  {
    for(int u=0;u<weight.size();u++)
    {
      //  if(i<5)
      //   std::cout<<"check 1"<<std::endl;   //测试
      if(d[v]+weight[v][u] < d[u])
      {
        d[u] = d[v]+weight[v][u];
        parent[u] = v;
      }
    }            
  }
}
  if(d[temp_index[n2.id]]==INT_MAX)
  return {""};

  path.push_back(temp_index[n2.id]);  //首先将dest node传进去
  create_path(path,parent,temp_index[n2.id]);   //按照dest->source顺序存parent nodes
  path.push_back(temp_index[n1.id]);    //最后加上source node

  //当前path vector是{dest,....source}，所以要reverse
  reverse(path.begin(), path.end());
  //path存着node的序号，转化为对应的string id
   std::vector<std::string> path_output;
  for(int i=0;i<path.size();i++)
  {  
    path_output.push_back(temp_id[path[i]]);
  }
  
 return path_output;

}




//step4: Brute force helper function
void TrojanMap::permute(std::vector<std::string> &location_ids, std::vector<std::vector<std::string> > &result,
std::vector<std::string> &curResult)           //curRersult = curpath
{
  //First, check if curpath amount = location_ids amount
  if(curResult.size()==location_ids.size())
  {
    result.push_back(curResult);
    return;
  }

//else, generate the curpath
for(int i=0;i<location_ids.size();i++)
{
  //if current id was found before, then find the next id
  if(find(curResult.begin(),curResult.end(),location_ids[i]) != curResult.end())
  continue;

  //else,add this id into the curpath
  curResult.push_back(location_ids[i]);   
  //if the curpath is smaller, then keep searching new node
  if(CalculatePathLength(curResult) < CalculatePathLength(location_ids))    
  {
  permute(location_ids,result,curResult);
  }
  //after finished the path, or the curpath is bigger, then pop out the curnode, then goes to the next node
  curResult.pop_back();
  } 
}
/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan(
                                    std::vector<std::string> &location_ids) {
  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<std::string> curResult;
  double minpath = INT_MAX;
  int min = 0;
  permute(location_ids,results.second,curResult);
  //find the min path
  for(int i=0;i<results.second.size();i++)
  {
    results.second[i].push_back(location_ids[0]);   //存每条path终点=source node
    if(CalculatePathLength(results.second[i]) < minpath && results.second[i][0] == location_ids[0])
    {
      minpath = CalculatePathLength(results.second[i]);
      min = i;
    }
  }
  results.first = minpath;
  std::cout<<"sadfsda"<<std::endl;
  results.second[min].swap(results.second[results.second.size()-1]);     //for TA's test 
  return results;
}








std::pair<double, std::vector<std::vector<std::string>>> TravellingTrojan_2opt(
      std::vector<std::string> &location_ids){
  std::pair<double, std::vector<std::vector<std::string>>> results;
  return results;
}









/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 *
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations 
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(std::string locations_filename){
  std::vector<std::string> location_names_from_csv;
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 *
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(std::string dependencies_filename){
  std::vector<std::vector<std::string>> dependencies_from_csv;
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a sorting of nodes
 * that satisfies the given dependencies.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(std::vector<std::string> &locations,
                                                     std::vector<std::vector<std::string>> &dependencies){
  std::vector<std::string> result;
  return result;                                                     
}






/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<double> &square) {
  return false;
}