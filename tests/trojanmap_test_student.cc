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




