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
  
}

