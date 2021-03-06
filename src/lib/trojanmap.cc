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
      "* 4. Travelling salesman problem - Brute Force                \n"
      "* 5. Travelling salesman problem - 2Opt                       \n"
      "* 6. Cycle Detection                                          \n"
      "* 7. Topological Sort                                         \n"
      "* 8. Exit                                                     \n"
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
        "* 3. CalculateShortestPath                                            "
        "      \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    std::cout << "If you select bellman-ford please expect around 4 minute runtime\n" << std::endl;
    menu = "Please select function: 1 = dijkstra, 2 = bellman-ford: ";
    std::cout << menu;
    getline(std::cin, input);
    int selFtn = std::stoi(input);
    if(selFtn < 1 || selFtn > 2) {
      std::cout << "Invalid input\n";
      break;
    }
    menu = "Please input the start location:";
    std::cout << menu;
    std::string input1;
    getline(std::cin, input1);
    menu = "Please input the destination:";
    std::cout << menu;
    std::string input2;
    getline(std::cin, input2);
    //used to calculate the time of the function
    auto start = std::chrono::high_resolution_clock::now(); 
    std::vector<std::string> results;
    if(selFtn == 1) {
      results = CalculateShortestPath_Dijkstra(input1, input2);
    }
    else {
      results = CalculateShortestPath_Bellman_Ford(input1, input2);
    }
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start); 

    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0)
    {
      for (auto x : results)
        std::cout << x << std::endl;
        std::cout << "The distance of the path is:" << CalculatePathLength(results) << " miles" << std::endl;
      PlotPath(results);
    }
      
    else
    {
      std::cout << "No route from the start point to the destination."
                << std::endl;
    }
          std::cout << "Time taken by function: " << duration.count() << " milliseconds" << std::endl;

    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }

  case '4':
  {
    menu =
        "**************************************************************\n"
        "* 4. Travelling salesman problem - Brute Force                \n"
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
    //auto results = TravellingTrojan_2opt(locations);
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
        "* 5. Travelling salesman problem - 2Opt                       \n"
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
    auto results = TravellingTrojan_2opt(locations);
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

  case '6':
  {
    menu =
        "**************************************************************\n"
        "* 6. Cycle Detection                                          \n"
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
  case '7':
  {
    menu =
        "**************************************************************\n"
        "* 7. Topological Sort                                         \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the locations filename: ???type any random letter to access our file)";
    std::cout << menu;
    std::string locations_filename;
    getline(std::cin, locations_filename);
    menu = "Please input the dependencies filename: (type any random letter to access our file)";
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
  case '8':
    break;
  default:
  {
    std::cout << "Please select 1 - 8." << std::endl;
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
  auto result = GetPlotLocation(data[id].lat, data[id].lon);      //???CSV?????????ID??????data???lat???long
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
    return data[id].lat;    //????????????id???latitude
}


/**
 * GetLon: Get the longitude of a Node given its id. 
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(std::string id) { 
    return data[id].lon;   //????????????id???lon
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
    return data[id].neighbors;  //????????????id???neighbors
}

/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {Node} a  : node a
 * @param  {Node} b  : node b
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const Node &a, const Node &b) { //node?????????data???string ID?????????node
  // Do not change this function                                    //??????data["123"] ??????id=123?????????node
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
  if (numofplace < 2)    
  return 0;

  else
  {
    for(int i=0;i<numofplace-1;i++)
    sum = sum + CalculateDistance(data[path[i]],data[path[i+1]]); 
    //?????????????????????string ID???distance???????????????
    //path[i] = string ID. data[path[i]]???data????????????string ID???Node
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

//??????autocomplete?????????????????????????????????string
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
    //???csv???name????????????input??????????????????????????????temp??????
    //ex:input=ch,?????????full name?????????????????????
    std::string temp = it->second.name.substr(0,prefix_size);//store prefix into temp
    //std::cout<<"current value is "<<temp<<std::endl; //????????????

    if (iequals(temp, name))            //??????????????????????????????
    {
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
    if(it->second.name == name)                     //if data node has the name matches input parameter
    {                                               //pass the Node lat and lon to the output
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
    if(it->second.name == name)   //??????csv??????name=??????????????????????????????node
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
  for(int i=0;i<d.size();i++)   //O(n)
  {
      auto it = visited.find(i);
      if(it==visited.end())   //if node has not visited,else??????for loop
      {                       
        double cur_dis = d[i];
        if(cur_dis < min_dis) //??????source??????node???distance????????????dis???????????????
        {                     //else,min??????
          min_dis = cur_dis;
          min_node = i;
          uncheck = false;
        }
      }     
  }
  return min_node;    //????????????-1????????????source???dest??????????????????
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
      weight[i][temp_index[j]] = CalculateDistance(data[temp_id[i]],data[j]); //datatemp_id[i]=??????node???data[j]=?????????neighbour
      weight[temp_index[j]][i] = weight[i][temp_index[j]];  //adjacency matrix?????????node??????undirected???A->B=B->A
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
if(n1.id==""||n2.id==""||n1.id==n2.id)
{
std::cout<<"invalid location"<<std::endl;   //test
return {""};
}

//If locations exist: 
//create adjacency matrix:outer vector is num of nodes, 
//inner vector are the weights of a node between all its neigbours

auto it =data.begin();
for(int i=0;i<data.size();i++)
{
  temp_id[i] = it->first;  //????????????i?????????id:ex temp[0]=id1,temp[1]=id2...
  temp_index[it->first] = i;  //??????id?????????
  it++;       //iterator??????????????????id??????
}
 weight_matrix();  //??????node???weight???????????????node???weight=max
std::vector<int> parent(data.size(), -1); //?????????path???destination node?????????parent node
std::unordered_set<int> visited;
std::vector<double> d(weight.size());
bool uncheck = false;   //initialize bool value,??????source node???????????????node????????????path
std::vector<int> path;  //???dest???source????????????????????????node
  //std::cout<<"weight size is"<<weight.size()<<std::endl;
for(int i=0;i<weight.size();i++)
  {    
  d[i] = weight[temp_index[n1.id]][i];  //source node?????????node???distance???????????????source????????????node???weight=max
  }

visited.insert(temp_index[n1.id]);  
while(visited.size()<weight.size() && !uncheck && visited.find(temp_index[n2.id])==visited.end())
  {
    uncheck = true;
    //??????unvisited???node?????????source?????????node?????????????????????????????????node
    int min_node = FindMinInDButNotInVisited(d,visited,uncheck);  
    visited.insert(min_node);
    for(int i=0;i<weight.size();i++)  //check if source node can reach new node through its neighbor node
    {
      //if so,then update the distance to d
      if(d[min_node] + weight[min_node][i] < d[i]){  
        d[i] = d[min_node]+weight[min_node][i];
        parent[i] = min_node;   //??????node???parent node?????????
      }
//ex: lec7???:if minnode=6,if node0->node3?????????(???????????????????????????=max?????????node0->5->6->3????????????node0->3?????????  
    }
  }
  //check if the destination node has been visited
  if(visited.find(temp_index[n2.id]) == visited.end()) //if not visited, then there is no path btw two nodes
  return {""};
  //if found dest node,search for the path btw source an dest node
  path.push_back(temp_index[n2.id]);  //?????????dest node?????????
  create_path(path,parent,temp_index[n2.id]);   //??????dest->source?????????parent nodes
  path.push_back(temp_index[n1.id]);    //????????????source node

  //??????path vector???{dest,....source}????????????reverse
  reverse(path.begin(), path.end());
  //path??????node??????????????????????????????string id
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
//std::cout<<"n1 is "<<n1.id<<" n2 is "<<n2.id<<std::endl;  //??????
if(n1.id==""||n2.id==""||n1.id==n2.id)
{
std::cout<<"invalid location"<<std::endl;   //??????
return {""};
}

//If locations exist: 
std::cout<<"Running Bellman will take 3-5 minutes to show the result!Please wait"<<std::endl;
auto it =data.begin();
for(int i=0;i<data.size();i++)
{
  temp_id[i] = it->first;  
  temp_index[it->first] = i;  
  it++;       //iterator??????????????????id??????
}
 weight_matrix();  //??????node???weight???????????????node???weight=max
std::vector<int> parent(data.size(), -1); //?????????path???destination node?????????parent node
std::vector<double> d(weight.size()); //1D table
std::vector<int> path;  //???dest???source????????????????????????node

//std::cout<<"weight size is"<<weight.size()<<std::endl;
for(int i=0;i<weight.size();i++)
  {    
  d[i] = weight[temp_index[n1.id]][i];  //source node?????????node???distance???????????????source????????????node???weight=max
  }
// visited.insert(temp_index[n1.id]);  //??????source node
d[temp_index[n1.id]] = 0;   //source node to itself distance = 0 

for(int i=0;i<weight.size()-1;i++)  //iterate 0->n-1 edges?????? EX:n=2 nodes,so only 1 edge exist
{ 
  //find total no of edges of certain nodes. no of edges = m
  for(int v=0;v<weight.size();v++)  //iterate all nodes
  {
    for(int u=0;u<weight.size();u++)  //then find the incoming edges of these nodes
    {
      //  if(i<5)
      //   std::cout<<"check 1"<<std::endl;   //??????
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

  path.push_back(temp_index[n2.id]);  //?????????dest node?????????
  create_path(path,parent,temp_index[n2.id]);   //??????dest->source?????????parent nodes
  path.push_back(temp_index[n1.id]);    //????????????source node

  //??????path vector???{dest,....source}????????????reverse
  reverse(path.begin(), path.end());
  //path??????node??????????????????????????????string id
   std::vector<std::string> path_output;
  for(int i=0;i<path.size();i++)
  {  
    path_output.push_back(temp_id[path[i]]);
  }
  
 return path_output;

}




//step4: Brute force helper function: permuate all the path
void TrojanMap::permute(std::vector<std::string> &location_ids, std::vector<std::vector<std::string> > &result,
std::vector<std::string> &curResult)           //curRersult = curpath
{
  double curpath = CalculatePathLength(location_ids);

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
  //if the curpath is equal or smaller, then keep searching new node
  if(CalculatePathLength(curResult) <= curpath)    
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
  if(location_ids.size() == 0)    //if place = 0, then no shortest path
    return {0, {}};
  if(location_ids.size() == 1)    //if place = itself, then return itself
    return {0, {location_ids}};

//else, find all the path from source to places and go back to source
  permute(location_ids,results.second,curResult);
  //find the min path
  for(int i=0;i<results.second.size();i++)
  {
    results.second[i].push_back(location_ids[0]);   //?????????path??????=source node
    if(CalculatePathLength(results.second[i]) < minpath && results.second[i][0] == location_ids[0])
    {
      minpath = CalculatePathLength(results.second[i]);
      min = i;
    }
  }
  results.first = minpath;
  results.second[min].swap(results.second[results.second.size()-1]);     //for TA's test 
  return results;
}







std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan_2opt(
      std::vector<std::string> &location_ids){
    std::pair<double, std::vector<std::vector<std::string> > > results;
    if(location_ids.size() == 0)    //if place = 0, then no shortest path
      return {0, {}};
    if(location_ids.size() == 1)    //if place = itself, then return itself
      return {0, {location_ids}};

    bool update = true;
    std::vector<std::string> exist_route = location_ids;
    exist_route.push_back(location_ids[0]);   //??????path??????=source node
    std::vector<std::string>  current_best_route = exist_route;
    results.second.push_back(exist_route);
    double best_distance = CalculatePathLength(exist_route);

    while (update)
    {
      update = false;
        for ( int i = 1; i < location_ids.size() - 2; i++ )
        {
            for ( int k = i + 1; k < location_ids.size() - 1; k++)
            {
              std::vector<std::string> nextResult = current_best_route;
              std::reverse(nextResult.begin()+i,nextResult.begin()+k+1);
                double new_distance = CalculatePathLength(nextResult);
 
                if ( new_distance < best_distance )
                {
                    // Improvement found so reset
                    update = true;
                    best_distance = new_distance;
                    current_best_route = nextResult;
                    results.second.push_back(current_best_route); //????????????smaller path??????cur best route??????result???
                }
            }
        }
        exist_route = current_best_route;
    }
    results.first = best_distance;    //????????????min distance??????result

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
  std::fstream fs;
  std::string line, word;
  fs.open(locations_filename,std::ios::in);
  getline(fs,line);
  while(getline(fs,line)){
    std::stringstream s(line);
    while(getline(s,word,',')){
      location_names_from_csv.push_back(word);
    }
  }
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
  std::fstream fs;
  std::string line;
  std::string word;
  fs.open(dependencies_filename,std::ios::in);
  getline(fs,line);
  while(getline(fs,line)){
    std::stringstream s(line);
    std::vector<std::string> temp;
    while(getline(s,word,',')){
      temp.push_back(word);
    }
    dependencies_from_csv.push_back(temp);
  }
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
  // ????????????location????????????????????????
  std::unordered_map<std::string, std::vector<std::string>> outgoing_nodes;
  // ??????location?????????
  std::unordered_map<std::string, int> degree;
  // ???????????????outgoing_nodes

std::string s1 = "/Users/chongkunlin/Desktop/USC_Courses/EE599/final-project-ChongkunLin/input/topologicalsort_dependencies.csv";
std::string s2 = "/Users/chongkunlin/Desktop/USC_Courses/EE599/final-project-ChongkunLin/input/topologicalsort_locations.csv";
if(locations.empty() || dependencies.empty())
{
locations = ReadLocationsFromCSVFile(s2);
dependencies = ReadDependenciesFromCSVFile(s1);}

  for(int i = 0; i < dependencies.size(); i++) {
    outgoing_nodes[dependencies[i][0]].push_back(dependencies[i][1]);
    degree[dependencies[i][1]]++;
  }
  // ????????????0????????????result
  for(int i = 0; i < locations.size(); i++) {
    if(degree.find(locations[i]) == degree.end()) result.push_back(locations[i]);
  }
  int start = 0, end = result.size();
  while(result.size() < locations.size()) {
    // ??????????????????0??????????????????????????????0??????
    while(start < end) {
      std::vector<std::string> nodes = outgoing_nodes[result[start]];
      for(int i = 0; i < nodes.size(); i++) {
        degree[nodes[i]]--;
        if(degree[nodes[i]] == 0) {
          result.push_back(nodes[i]);
          degree.erase(nodes[i]);
        }
      }
      start++;
    }
    // ?????????result????????????index??????
    start = end;
    end = result.size();
  }
  if(degree.size()>0)
  {
    std::vector<std::string> temp;
    return temp;
  }
  return result;                                                     
}









/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true if there
 * is a cycle path inside the square, false otherwise.
 * 
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
void dfs(
std::map<std::string,Node>& data, 
std::string cur,
std::unordered_map<std::string,int>&met,
std::unordered_map<std::string,std::string>& bef,
std::vector<std::string>& cyc){
  met[cur] = 1;
  for (std::string next : data[cur].neighbors)
  {
    if (cyc.size()>0){
      return;
    }
    if (met.find(next) == met.end())
    {
      continue;
    }
    if (met[next] == 0)
    {
      bef[next] = cur;
      dfs(data,next,met,bef,cyc);
    } 
    else if (met[next]==1 && next != bef[cur])
    {
      std::string temp =cur;
      while(temp!=next){
        cyc.push_back(temp);
        temp=bef[temp];
      }
      cyc.push_back(temp);
      return;
    }
  }
  met[cur] = 2;
}

bool TrojanMap::CycleDetection(std::vector<double> &square) {
  std::unordered_map<std::string,int> visited;
  std::vector<std::string> cycle;
  std::unordered_map<std::string,std::string> prev;
  
  for(auto it = data.begin();it!=data.end();it++)
  {
    if(it->second.lon >= square[0] && it->second.lon <= square[1] && it->second.lat<=square[2] && it->second.lat>=square[3])   //??????????????????????????????????????????node??????visited
    {
      visited[it->first] = 0;           //visited[id]???????????????0
    }
  }
  for(auto it1 = visited.begin();it1!=visited.end();it1++){
    if (visited[it1->first] == 0){
      dfs(data,it1->first,visited,prev,cycle);
      if(cycle.size()>0)
      {
        PlotPointsandEdges(cycle,square);
        return true;
      }
    }
  }
  
  return false;
}