/*
 * Playground_test.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: gmo
 */

#include "../src/pmr_playground.h"
#include "../src/pmr_edge.h"
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
// The fixture for testing struct Node.
class EdgeTest : public ::testing::Test {
protected:
    Node * n1;
    Node * n2;
    Node * n3;
    Node * n4;
    Edge * e1;
    Edge * e2;

  EdgeTest() {
    n1 = node_new(-1.0, 0.0);
    n2 = node_new(1.0, 0.0);
    n3 = node_new(0.0, -1.0);
    n4 = node_new(0.0, 1.0);
    e1 = edge_new(n1, n2);
    e2 = edge_new(n3, n4);
  }

  virtual ~EdgeTest() {
    node_free(n1);
    node_free(n2);
    node_free(n3);
    node_free(n4);
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

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
