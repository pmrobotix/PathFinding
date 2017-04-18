/*
 * Playground_test.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: gmo
 */

#include <math.h>
#include "../src/pmr_tools.h"
#include "../src/pmr_playground.h"
#include "../src/pmr_edge.h"
#include "../src/pmr_zone.h"
#include "gtest/gtest.h"

namespace {

// The fixture for testing struct Node.
class NodeTest : public ::testing::Test {
protected:
    Node * n1;
    Node * n2;
    Node * n3;
    Node * n4;
    Node * n5;

  NodeTest() {
    n1 = node_new(1.0, 0.0);
    n1->f_score = n1->x;
    n2 = node_new(2.0, 0.0);
    n2->f_score = n2->x;
    n3 = node_new(3.0, 0.0);
    n3->f_score = n3->x;
    n4 = node_new(4.0, 0.0);
    n4->f_score = n4->x;
    n5 = node_new(1.0, 0.0);
    n5->f_score = n5->x;
  }

  virtual ~NodeTest() {
    node_free(n1);
    node_free(n2);
    node_free(n3);
    node_free(n4);
    node_free(n5);
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case.
};
// The fixture for testing struct Edge.
class EdgeTest : public ::testing::Test {
protected:
    Node * n1;
    Node * n2;
    Node * n3;
    Node * n4;
    Node * n5;
    Node * n6;
    Node * n7;
    Node * n8;
    Node * n9;
    Node * n10;
    Node * n11;
    Node * n12;
    Edge * e1;
    Edge * e2;
    Edge * e3;
    Edge * e4;
    Edge * e5;
    Edge * e6;
    Edge * e7;
    Edge * e8;

  EdgeTest() {
    n1 = node_new(-1.0, 0.0);
    n2 = node_new(1.0, 0.0);
    n3 = node_new(0.0, -1.0);
    n4 = node_new(0.0, 1.0);
    n5 = node_new(-1.0, -1.0);
    n6 = node_new(1.0, 1.0);
    n7 = node_new(-1.0, -1.0);
    n8 = node_new(1.0, 1.0);
    n9 = node_new(-1.0, 1.0);
    n10 = node_new(1.0, -1.0);
    n11 = node_new(10.0, 10.0);
    n12 = node_new(12.0, 12.0);
    e1 = edge_new(n1, n2);
    e2 = edge_new(n3, n4);
    e3 = edge_new(n5, n6);
    e4 = edge_new(n7, n8);
    e5 = edge_new(n9, n6);
    e6 = edge_new(n10, n6);
    e7 = edge_new(n1, n4);
    e8 = edge_new(n11, n12);
  }

  virtual ~EdgeTest() {
    node_free(n1);
    node_free(n2);
    node_free(n3);
    node_free(n4);
    node_free(n5);
    node_free(n6);
    node_free(n7);
    node_free(n8);
    node_free(n9);
    node_free(n10);
    node_free(n11);
    node_free(n12);
    edge_free(e1);
    edge_free(e2);
    edge_free(e3);
    edge_free(e4);
    edge_free(e5);
    edge_free(e6);
    edge_free(e7);
    edge_free(e8);
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case.
};

// The fixture for testing struct Node.
class ZoneTest : public ::testing::Test {
protected:
    Zone * z1;
    std::vector<Point> zone_points_list;
    Point p1 = Point();
    Point p2 = Point();
    Point p3 = Point();
    Point p4 = Point();
    Edge * e1;
    Edge * e2;

  ZoneTest() {
      p1.x = -1.0;
      p1.y = -1.0;
      p2.x = 1.0;
      p2.y = -1.0;
      p3.x = 1.0;
      p3.y = 1.0;
      p4.x = -1.0;
      p4.y = 1.0;
      zone_points_list.push_back(p1);
      zone_points_list.push_back(p2);
      zone_points_list.push_back(p3);
      zone_points_list.push_back(p4);
      z1 = zone_new(0, zone_points_list);
      e1 = edge_new(z1->nodes[0], z1->nodes[1]);
      e2 = edge_new(z1->nodes[0], z1->nodes[2]);
      edge_update(e1);
      edge_update(e2);
  }

  virtual ~ZoneTest() {
    node_free(z1->nodes[0]);
    node_free(z1->nodes[1]);
    node_free(z1->nodes[2]);
    node_free(z1->nodes[3]);
    zone_free(z1);
    edge_free(e1);
    edge_free(e2);
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  // Objects declared here can be used by all tests in the test case.
};


TEST_F(NodeTest, CheckCreation) {
    EXPECT_EQ(n1->f_score, n1->x);
    EXPECT_EQ(n2->f_score, n2->x);
    EXPECT_EQ(n3->f_score, n3->x);
    EXPECT_EQ(n4->f_score, n4->x);
}

TEST_F(NodeTest, CheckEdges) {
    Edge e;
    node_create_edges_array(n1, 3);
    node_add_edge(n1, &e);
    node_add_edge(n1, &e);
    node_add_edge(n1, &e);
    EXPECT_EQ(3, n1->edges_count);
}

TEST_F(NodeTest, CheckCoords) {
    int result = node_coords_equal(n1, n5);
    EXPECT_NE(0, result);
}

TEST_F(NodeTest, CheckSortedListInsert) {
    Node * my_sorted_list = NULL;
    my_sorted_list = node_list_insert_sorted(my_sorted_list, n2);
    my_sorted_list = node_list_insert_sorted(my_sorted_list, n3);
    my_sorted_list = node_list_insert_sorted(my_sorted_list, n1);
    my_sorted_list = node_list_insert_sorted(my_sorted_list, n4);
    Node * current = my_sorted_list;
    EXPECT_EQ(1.0, current->f_score);
    current = current->next;
    EXPECT_EQ(2.0, current->f_score);
    current = current->next;
    EXPECT_EQ(3.0, current->f_score);
    current = current->next;
    EXPECT_EQ(4.0, current->f_score);
}

TEST_F(NodeTest, CheckSortedListDelete) {
    Node * my_sorted_list = NULL;
    my_sorted_list = node_list_insert_sorted(my_sorted_list, n2);
    my_sorted_list = node_list_insert_sorted(my_sorted_list, n3);
    my_sorted_list = node_list_insert_sorted(my_sorted_list, n1);
    my_sorted_list = node_list_insert_sorted(my_sorted_list, n4);
    Node * current = my_sorted_list;
    EXPECT_EQ(1.0, current->f_score);
    current = node_list_pop(current);
    EXPECT_EQ(2.0, current->f_score);
    current = node_list_pop(current);
    EXPECT_EQ(3.0, current->f_score);
    current = node_list_pop(current);
    EXPECT_EQ(4.0, current->f_score);
    EXPECT_EQ(NULL, n1->next);
    EXPECT_EQ(NULL, n2->next);
    EXPECT_EQ(NULL, n3->next);
    EXPECT_EQ(NULL, n4->next);
}


TEST_F(EdgeTest, CheckLinks) {
    int result = edge_links(e1, n1, n2);
    EXPECT_NE(0, result);
    result = edge_links(e1, n2, n1);
    EXPECT_NE(0, result);
    result = edge_links(e2, n3, n4);
    EXPECT_NE(0, result);
    result = edge_links(e2, n4, n3);
    EXPECT_NE(0, result);
    result = edge_links(e1, n1, n3);
    EXPECT_EQ(0, result);
    result = edge_links(e1, n1, n4);
    EXPECT_EQ(0, result);
    result = edge_links(e1, n2, n3);
    EXPECT_EQ(0, result);
    result = edge_links(e1, n2, n4);
    EXPECT_EQ(0, result);
    result = edge_links(e1, n3, n4);
    EXPECT_EQ(0, result);
    result = edge_links(e1, n4, n3);
    EXPECT_EQ(0, result);
    result = edge_links(e2, n3, n1);
    EXPECT_EQ(0, result);
    result = edge_links(e2, n3, n2);
    EXPECT_EQ(0, result);
    result = edge_links(e2, n4, n1);
    EXPECT_EQ(0, result);
    result = edge_links(e2, n4, n2);
    EXPECT_EQ(0, result);
    result = edge_links(e2, n1, n2);
    EXPECT_EQ(0, result);
    result = edge_links(e2, n2, n1);
    EXPECT_EQ(0, result);
}

TEST_F(EdgeTest, CheckOtherNode) {
    Node * result;
    result = edge_other_node(e1, n1);
    EXPECT_EQ(n2, result);
    result = edge_other_node(e1, n2);
    EXPECT_EQ(n1, result);
    result = edge_other_node(e1, n3);
    EXPECT_EQ(NULL, result);
}

TEST_F(EdgeTest, CheckUpdate) {
    edge_update(e1);
    edge_update(e2);
    edge_update(e3);
    edge_update(e7);
    EXPECT_EQ(0.0, e1->a);
    EXPECT_EQ(0.0, e1->b);
    EXPECT_EQ(2.0, e1->length);
    EXPECT_EQ(INFINITY, e2->a);
    EXPECT_EQ(INFINITY, e2->b);
    EXPECT_EQ(2.0, e2->length);
    EXPECT_EQ(1.0, e3->a);
    EXPECT_EQ(0.0, e3->b);
    EXPECT_NE(0, tools_quasi_equal(2.0*sqrt(2.0), e3->length));
    EXPECT_NE(0, tools_quasi_equal(sqrt(2.0), e7->length));
}

TEST_F(EdgeTest, CheckContains) {
    int result;
    edge_update(e1);
    edge_update(e2);
    edge_update(e3);
    result = edge_contains(e1, 0.0, 0.0);
    EXPECT_NE(0, result);
    result = edge_contains(e2, 0.0, 0.0);
    EXPECT_NE(0, result);
    result = edge_contains(e3, 0.0, 0.0);
    EXPECT_NE(0, result);
    result = edge_contains(e3, 1.0, 1.0);
    EXPECT_NE(0, result);
    result = edge_contains(e1, 0.0, 1.0);
    EXPECT_EQ(0, result);
    result = edge_contains(e2, 1.0, 0.0);
    EXPECT_EQ(0, result);
    result = edge_contains(e3, 0.0, 1.0);
    EXPECT_EQ(0, result);
}

TEST_F(EdgeTest, CheckIntersection) {
    int result;
    edge_update(e1);
    edge_update(e2);
    edge_update(e3);
    edge_update(e4);
    edge_update(e5);
    edge_update(e6);
    edge_update(e7);
    result = edge_intersects(e1, e1);
    EXPECT_EQ(0, result);
    result = edge_intersects(e1, e2);
    EXPECT_NE(0, result);
    result = edge_intersects(e1, e3);
    EXPECT_NE(0, result);
    result = edge_intersects(e2, e2);
    EXPECT_EQ(0, result);
    result = edge_intersects(e2, e3);
    EXPECT_NE(0, result);
    result = edge_intersects(e3, e3);
    EXPECT_EQ(0, result);
    result = edge_intersects(e3, e4);
    EXPECT_EQ(0, result); // segments with equal coordinates
    result = edge_intersects(e6, e5);
    EXPECT_EQ(0, result); // segments with common endpoint
    result = edge_intersects(e1, e5);
    EXPECT_EQ(0, result); // parallel horizontal
    result = edge_intersects(e2, e6);
    EXPECT_EQ(0, result); // parallel vertical
    result = edge_intersects(e4, e8);
    EXPECT_EQ(0, result); // on same straight line
}

TEST_F(ZoneTest, CheckZoneSize) {
    EXPECT_EQ(4, z1->nodes_count);
}

TEST_F(ZoneTest, CheckZoneUpdate) {
    Point p1 = Point();
    Point p2 = Point();
    Point p3 = Point();
    Point p4 = Point();
    Point p5 = Point();
    std::vector<Point> points_list;
    Node * current;
    unsigned int num_exceptions = 0;

    p1.x = -2.0;
    p1.y = -2.0;
    p2.x =  2.0;
    p2.y = -2.0;
    p3.x =  2.0;
    p3.y =  2.0;
    p4.x = -2.0;
    p4.y =  2.0;
    p5.x = -2.0;
    p5.y =  0.0;
    points_list.push_back(p1);
    points_list.push_back(p2);
    points_list.push_back(p3);
    points_list.push_back(p4);
    zone_update(z1, points_list);
    EXPECT_EQ(4, z1->nodes_count);
    current = z1->nodes[0];
    EXPECT_EQ(-2.0, current->x);
    EXPECT_EQ(-2.0, current->y);
    current = z1->nodes[1];
    EXPECT_EQ( 2.0, current->x);
    EXPECT_EQ(-2.0, current->y);
    current = z1->nodes[2];
    EXPECT_EQ( 2.0, current->x);
    EXPECT_EQ( 2.0, current->y);
    current = z1->nodes[3];
    EXPECT_EQ(-2.0, current->x);
    EXPECT_EQ( 2.0, current->y);

    // now try to update with one point more than expected.
    points_list.push_back(p5);
    try {
        zone_update(z1, points_list);
    }
    catch (std::exception & e) {
        num_exceptions++;
    }
    EXPECT_EQ(1, num_exceptions);
}

TEST_F(ZoneTest, CheckZoneIsInternal) {
    int result;
    result = zone_is_internal_edge(z1, e1);
    EXPECT_EQ(0, result);
    result = zone_is_internal_edge(z1, e2);
    EXPECT_NE(0, result);
}


}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
