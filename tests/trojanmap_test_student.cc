#include <map>
#include <vector>

#include "gtest/gtest.h"
#include "src/lib/trojanmap.h"

TEST(TrojanMapStudentTest, autocomplete) {
TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test the simple case
  auto names = m.Autocomplete("Ch");
  std::vector<std::string> gt1 = {"ChickfilA", "Chipotle Mexican Grill"}; // groundtruth for "Ch"
  EXPECT_EQ(names, gt1);
  // Test the lower case
   names = m.Autocomplete("ch");
  std::vector<std::string> gt2 = {"ChickfilA", "Chipotle Mexican Grill"}; // groundtruth for "ch"
  EXPECT_EQ(names, gt2);

  // Test the lower case
   names = m.Autocomplete("DI");
  std::vector<std::string> gt3 = {"Divine Providence Convent", "Divine Providence Kindergarten and Day Nursery"}; // groundtruth for "ch"
  EXPECT_EQ(names, gt3);
  

   // Test the lower case
   names = m.Autocomplete("di");
  std::vector<std::string> gt4 = {"Divine Providence Convent", "Divine Providence Kindergarten and Day Nursery"}; // groundtruth for "ch"
  EXPECT_EQ(names, gt4);

   // Test the mixed case
   names = m.Autocomplete("dI");
  std::vector<std::string> gt5 = {"Divine Providence Convent", "Divine Providence Kindergarten and Day Nursery"}; // groundtruth for "ch"
  EXPECT_EQ(names, gt5);
}





// Test FindPosition function
TEST(TrojanMapTest, FindPosition) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test ChickfilA
  auto position = m.GetPosition("ChickfilA");
  std::pair<double, double> gt1(34.0167334, -118.2825307); // groundtruth for "ChickfilA"
  EXPECT_EQ(position, gt1);
  // Test Ralphs
  position = m.GetPosition("Ralphs");
  std::pair<double, double> gt2(34.0317653, -118.2908339); // groundtruth for "Ralphs"
  EXPECT_EQ(position, gt2);
  // Test Target
  position = m.GetPosition("Target");
  std::pair<double, double> gt3(34.0257016, -118.2843512); // groundtruth for "Target"
  EXPECT_EQ(position, gt3);
}






TEST(TrojanMapStudentTest, getName) {
TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto names = m.GetName("358850006");
  std::string expected1 = "Korean Presbyterian Church";
  EXPECT_EQ(names, expected1);

  names = m.GetName("441891112");
  std::string expected2 = "";
  EXPECT_EQ(names, expected2);
}



TEST(TrojanMapStudentTest, GetNeighborIDs) {
TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto names = m.GetNeighborIDs("368172344");
  std::vector<std::string> expected1 = {"6512287152", "4399693646"};
  EXPECT_EQ(names, expected1);
}



TEST(TrojanMapStudentTest, GetLat) {
TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto lat = m.GetLat("368172344");
  double expected1 = 34.0225143;
  EXPECT_EQ(lat, expected1);
}


TEST(TrojanMapStudentTest, GetLon) {
TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto lon = m.GetLon("368172344");
  double expected1 = -118.2911925;
  EXPECT_EQ(lon, expected1);
}






// Test CalculateShortestPath_Dijkstra function 1
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Test from Ralphs to ChickfilA
  auto path = m.CalculateShortestPath_Dijkstra("Ralphs", "ChickfilA");
  std::vector<std::string> gt{
      "2578244375", "5559640911", "6787470571", "6808093910", "6808093913", "6808093919", "6816831441",
      "6813405269", "6816193784", "6389467806", "6816193783", "123178876", "2613117895", "122719259",
      "2613117861", "6817230316", "3642819026", "6817230310", "7811699597", "5565967545", "123318572",
      "6813405206", "6813379482", "544672028", "21306059", "6813379476", "6818390140", "63068610", 
      "6818390143", "7434941012", "4015423966", "5690152766", "6813379440", "6813379466", "21306060",
      "6813379469", "6813379427", "123005255", "6807200376", "6807200380", "6813379451", "6813379463",
      "123327639", "6813379460", "4141790922", "4015423963", "1286136447", "1286136422", "4015423962",
      "6813379494", "63068643", "6813379496", "123241977", "4015372479", "4015372477", "1732243576",
      "6813379548", "4015372476", "4015372474", "4015372468", "4015372463", "6819179749", "1732243544",
      "6813405275", "348121996", "348121864", "6813405280", "1472141024", "6813411590", "216155217", 
      "6813411589", "1837212103", "1837212101", "6820935911", "4547476733"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Ralphs to ChickfilA
  path = m.CalculateShortestPath_Dijkstra("ChickfilA", "Ralphs");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

// Test CalculateShortestPath_Dijkstra function 2
TEST(TrojanMapTest, CalculateShortestPath_Dijkstra2) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto path = m.CalculateShortestPath_Dijkstra("Target", "Popeyes Louisiana Kitchen");
  // Test from Target to Popeyes Louisiana Kitchen
  std::vector<std::string> gt{
      "5237417650", "6813379479", "5237381975", "4399698012", "4399698013", "4399698011", "4399698010", 
      "123044712", "4399698009", "4399698008", "123005253", "6813379513", "6813379517", "6813379521", 
      "123327627", "4399697999", "6813565290", "122719210", "6813379407", "2613117879", "6813379406", 
      "6807905595", "6787803635", "2613117867", "4835551110", "6813565296", "122719205", "6813565294", "4835551232", 
      "4835551104", "4012842272", "4835551103", "123178841", "6813565313", "122814435", "6813565311", "4835551228", 
      "6813513565", "4835551090", "4835551081", "6813513564", "20400292", "5556117120", "5556117115", "4835551064", 
      "4012842277", "6813565326", "123241961", "6813565322", "4835551070", "5695236164"}; // Expected path
  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  // Reverse the input from Popeyes Louisiana Kitchen to Target
  path = m.CalculateShortestPath_Dijkstra("Popeyes Louisiana Kitchen", "Target");
  std::reverse(gt.begin(),gt.end()); // Reverse the path

  // Print the path lengths
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

