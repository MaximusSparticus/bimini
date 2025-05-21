#include "gtest/gtest.h"

#include "bimini/BIMInterface.hpp"

TEST(BIMInterface, loadIFC_badPath) {
    bimini::BIMInterface bimi;

    EXPECT_FALSE(bimi.loadIFC("doesn't exist"));
}

TEST(BIMInterface, loadIFC) {
    bimini::BIMInterface bimi;

    ASSERT_TRUE(bimi.loadIFC("test/test.ifc"));
}

TEST(BIMInterface, dumpEntities) {
    bimini::BIMInterface bimi;

    ASSERT_TRUE(bimi.loadIFC("test/test.ifc"));
    bimi.dumpEntities();
}
