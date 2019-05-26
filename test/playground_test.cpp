/*
 * Playground_test.cpp
 *
 *  Created on: Apr 8, 2017
 *      Author: gmo
 */


#include <gtest/gtest.h>
#include <gtest/internal/gtest-internal.h>
#include <simple_svg_1.0.0.hpp>
#include <cmath>
#include <exception>
#include <iterator>
#include <string>
#include <vector>

#include "../src/pmr_edge.h"
#include "../src/pmr_node.h"
#include "../src/pmr_path_result.h"
#include "../src/pmr_pathfinding.h"
#include "../src/pmr_symmetrical_pg.h"
#include "../src/pmr_point.h"
#include "../src/pmr_tools.h"
#include "../src/pmr_zone.h"

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
    Node * n13;
    Node * n14;
    Edge * e1;
    Edge * e2;
    Edge * e3;
    Edge * e4;
    Edge * e5;
    Edge * e6;
    Edge * e7;
    Edge * e8;
    Edge * e9;

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
    n13 = node_new(10.001, 9.99);
    n14 = node_new(12.0, 12.0);
    e1 = edge_new(n1, n2);
    e2 = edge_new(n3, n4);
    e3 = edge_new(n5, n6);
    e4 = edge_new(n7, n8);
    e5 = edge_new(n9, n6);
    e6 = edge_new(n10, n6);
    e7 = edge_new(n1, n4);
    e8 = edge_new(n11, n12);
    e9 = edge_new(n13, n14);
  }

  virtual ~EdgeTest() {
      edge_free(e1);
      edge_free(e2);
      edge_free(e3);
      edge_free(e4);
      edge_free(e5);
      edge_free(e6);
      edge_free(e7);
      edge_free(e8);
      edge_free(e9);
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
      node_free(n13);
      node_free(n14);
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
    Node * n1;
    Node * n2;
    Node * n3;
    Node * n4;

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
      n1 = node_new(-0.5, 1.0);
      n2 = node_new( 0.0, 0.0);
      n3 = node_new( 2.0, 0.0);
      n4 = node_new(-1.0, 0.5);
  }

  virtual ~ZoneTest() {
    node_free(z1->nodes[0]);
    node_free(z1->nodes[1]);
    node_free(z1->nodes[2]);
    node_free(z1->nodes[3]);
    node_free(n1);
    node_free(n2);
    node_free(n3);
    node_free(n4);
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

class SVGTest {
public:
    virtual void toSVG(Playground * p, FoundPath * found_path, std::string file_name) {
        PathFinder * pf = p->get_path_finder();
        std::vector<Zone*>::iterator zones_it;
        std::vector<Edge*>::iterator edges_it;
        std::vector<Node*>::iterator nodes_it;
        unsigned int i;
        svg::Dimensions dimensions(pf->field_x2, pf->field_y2);

        svg::Layout lay(dimensions, svg::Layout::TopLeft);

        svg::Document doc(file_name, lay);

        doc << svg::elemStart("g") << svg::attribute("transform","translate(200,2200) scale(1,-1)") << svg::emptyElemEnd(false) ;

        // Red image border.
        svg::Polygon border(svg::Fill(svg::Color::White), svg::Stroke(5, svg::Color::Red));
        border << svg::Point(pf->field_x1, pf->field_y1) << svg::Point(dimensions.width, pf->field_y1)
            << svg::Point(dimensions.width, dimensions.height) << svg::Point(pf->field_x1, dimensions.height);
        doc << border;

        // Display all zones
        for (zones_it = pf->zones.begin(); zones_it < pf->zones.end(); zones_it++) {
            Zone* zone = *zones_it;
            if (zone->nodes_count > 0) {
                svg::Polygon zone_poly(svg::Fill(svg::Color::Aqua), svg::Stroke(0, svg::Color::Aqua));
                for (i = 0; i < zone->nodes_count; i++) {
                    Node* node = zone->nodes[i];
                    zone_poly << svg::Point(node->x, node->y);


                }
                doc << zone_poly;
            }

            std::string s = "Zone " + std::to_string(zone_getId(zone)) ; //;
            float x, y;
            zone_get_center(zone, &x, &y);
            std::string t = "translate(0,"+ std::to_string(y*2) +") scale(1,-1)";
            doc << svg::Text(svg::Point(x, y), s, svg::Color::Black, svg::Font(50, "Verdana"), svg::Stroke(), t);
        }

        // Display all enabled edges
        for (edges_it = pf->edges.begin(); edges_it < pf->edges.end(); edges_it++) {
            Edge* edge = *edges_it;
            if (edge->enabled) {
                svg::Point p1(edge->node1->x, edge->node1->y);
                svg::Point p2(edge->node2->x, edge->node2->y);
                svg::Line line(p1, p2, svg::Stroke(0.5, svg::Color(238,238,238)));
                doc << line;
            }
        }

        // Display found path
        if (found_path != NULL) {
            bool node_found = false;
            svg::Polyline path_polyline(svg::Stroke(5, svg::Color::Green));

            for (nodes_it = found_path->path.begin(); nodes_it < found_path->path.end(); nodes_it++) {
                Node* node = *nodes_it;
                if (!node_found) {
                    node_found = true;
                }
                path_polyline << svg::Point(node->x, node->y);
            }
            if (node_found) {
                doc << path_polyline;
            }
        }

        doc << svg::elemEnd("g");
        doc.save();
    }
    virtual ~SVGTest() {
    }
};

class SimplePlaygroundTest : public ::testing::Test, public SVGTest {
protected:
    Playground * p;
    PlaygroundObjectID me = Playground::INVALID;

SimplePlaygroundTest() {
   p = new Playground(0.0, 0.0, 3000.0, 2100.0, 0.02, 1.0);
   p->add_circle(me, 400.0, 525.0, 180.0, 3)
            ->add_rectangle(1050.0, 1300.0, 1200.0, 30.0, -3.1415926/4.0);
}

virtual ~SimplePlaygroundTest() {
    delete p;
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

class PlaygroundTest : public ::testing::Test, public SVGTest {
protected:
    Playground * p;
    PlaygroundObjectID me = Playground::INVALID;
    PlaygroundObjectID teammate = Playground::INVALID;
    PlaygroundObjectID opponent_1 = Playground::INVALID;
    PlaygroundObjectID opponent_2 = Playground::INVALID;
    PlaygroundObjectID obj = Playground::INVALID;

PlaygroundTest() {
    p = new Playground(0.0, 0.0, 3000.0, 2000.0, 0.02, 1.0);
    p->add_circle(me, 400.0, 525.0, 180.0, 3)
		->add_rectangle(obj,1500.0, 1050.0, 200.0, 400.0, 0)
            ->add_circle(teammate, 400.0, 1575.0, 180.0, 8)
            ->add_circle(opponent_1, 2600.0, 525.0, 180.0, 4)
            ->add_circle(opponent_2, 2000.0, 1375.0, 250.0, 8);

}

virtual ~PlaygroundTest() {
    delete p;
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



class PlaygroundTestImpossiblePlaces : public ::testing::Test, public SVGTest {
protected:
    Playground * p;
    PlaygroundObjectID opponent_1 = Playground::INVALID;
    PlaygroundObjectID opponent_2 = Playground::INVALID;
    PlaygroundObjectID oarea_cube1;
    PlaygroundObjectID oarea_cube2high;
    PlaygroundObjectID oarea_cube3left = Playground::INVALID;
    PlaygroundObjectID oarea_distribadverse;
    PlaygroundObjectID garea_cube1;
    PlaygroundObjectID garea_cube2high = Playground::INVALID;
    PlaygroundObjectID garea_cube3left = Playground::INVALID;
    PlaygroundObjectID garea_distribadverse;
    PlaygroundObjectID area_stations;

    PlaygroundTestImpossiblePlaces() {
    p = new Playground(0.0, 0.0, 3400.0, 2500.0, 0.5, 1.0);

    /*p->add_circle(me, 400.0, 525.0, 180.0, 3)
                    ->add_rectangle(obj,1500.0, 1050.0, 200.0, 400.0, 0)
            ->add_circle(teammate, 400.0, 1575.0, 180.0, 8)
            ->add_circle(opponent_1, 2600.0, 525.0, 180.0, 4)
            ->add_circle(opponent_2, 2000.0, 1375.0, 250.0, 8);
            */
            //bordure terrain
       p->add_rectangle(1500, 0, 3000, 140, 0); //bottom
       p->add_rectangle(1500, 2000, 3000, 140, 0); //top
       p->add_rectangle(0, 1000, 140, 2000, 0); //left
       p->add_rectangle(3000, 1000, 140, 2000, 0); //right

       //orange parts
       //cubes
       p->add_circle(oarea_cube1, 850.0, 530.0, 300.0, 6);
       p->add_circle(oarea_cube2high, 1100.0, 1500.0, 300.0, 6);
       p->add_circle(oarea_cube3left, 300.0, 1200.0, 300.0, 6);

       //distributeurs
       p->add_rectangle(oarea_distribadverse, 600, 1900, 200.0, 200.0, 0);

       //Green parts
       //cubes
       p->add_circle(garea_cube1, 2150.0, 530.0, 300.0, 6);
       p->add_circle(garea_cube2high, 1900.0, 1500.0, 300.0, 6);
       p->add_circle(garea_cube3left, 2700.0, 1200.0, 300.0, 6);

       //distributeurs
       p->add_rectangle(garea_distribadverse, 2400, 1900, 200.0, 200.0, 0);

       //stations d'épuration
       p->add_rectangle(area_stations, 1500.0, 1850.0, 1400.0, 300.0, 0);
       p->add_circle(opponent_1, 1200.0, 1200.0, 600.0, 10);


}

virtual ~PlaygroundTestImpossiblePlaces() {
    delete p;
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

class PlaygroundSizeTest : public ::testing::Test, public SVGTest {
protected:
    Playground * p;
    PlaygroundObjectID me = Playground::INVALID;

PlaygroundSizeTest() {
    p = new Playground(190.0, 230.0, 3000.0, 2100.0, 0.02, 1.0);
}

virtual ~PlaygroundSizeTest() {
    delete p;
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


class SymCircleTest : public ::testing::Test, public SVGTest {
protected:
    SymmetricalPlayground * sp;
    PlaygroundObjectID me = Playground::INVALID;
    PlaygroundObjectID teammate = Playground::INVALID;
    PlaygroundObjectID opponent_1 = Playground::INVALID;
    PlaygroundObjectID opponent_2 = Playground::INVALID;

SymCircleTest() {
    sp = new SymmetricalPlayground(0.0, 0.0, 3000.0, 2000.0, 0.02, 1.0, 1500.0);
    sp->add_circle_symmetrical(me, teammate, 400.0, 525.0, 150.0, 5)
                    ->add_circle_symmetrical(opponent_1, opponent_2, 400.0, 1000.0, 180.0, 8);

}

virtual ~SymCircleTest() {
    delete sp;
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


class SymHalfCircleTest : public ::testing::Test, public SVGTest {
protected:
    SymmetricalPlayground * sp;
    PlaygroundObjectID me = Playground::INVALID;
    PlaygroundObjectID teammate = Playground::INVALID;
    PlaygroundObjectID opponent_1 = Playground::INVALID;
    PlaygroundObjectID opponent_2 = Playground::INVALID;

SymHalfCircleTest() {
    sp = new SymmetricalPlayground(0.0, 0.0, 3000.0, 2000.0, 0.02, 1.0, 1000.0);
    sp->add_half_circle_symmetrical(me, teammate, 400.0, 525.0, 150.0, 0.25f*((float)M_PI), 5)
                    ->add_half_circle_symmetrical(opponent_1, opponent_2, 400.0, 1000.0, 180.0, 0.25f*((float)M_PI), 8);

}

virtual ~SymHalfCircleTest() {
    delete sp;
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


class SymQuarterCircleTest : public ::testing::Test, public SVGTest {
protected:
    SymmetricalPlayground * sp;
    PlaygroundObjectID me = Playground::INVALID;
    PlaygroundObjectID teammate = Playground::INVALID;
    PlaygroundObjectID opponent_1 = Playground::INVALID;
    PlaygroundObjectID opponent_2 = Playground::INVALID;

SymQuarterCircleTest() {
    sp = new SymmetricalPlayground(0.0, 0.0, 3000.0, 2000.0, 0.02, 1.0, 1500.0);
    sp->add_quarter_circle_symmetrical(me, teammate, 400.0, 525.0, 150.0, 0.25f*((float)M_PI), 5)
                    ->add_quarter_circle_symmetrical(opponent_1, opponent_2, 400.0, 1000.0, 180.0, -0.25f*((float)M_PI), 8);

}

virtual ~SymQuarterCircleTest() {
    delete sp;
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


class SymSegmentTest : public ::testing::Test, public SVGTest {
protected:
    SymmetricalPlayground * sp;
    PlaygroundObjectID me = Playground::INVALID;
    PlaygroundObjectID teammate = Playground::INVALID;
    PlaygroundObjectID opponent_1 = Playground::INVALID;
    PlaygroundObjectID opponent_2 = Playground::INVALID;

SymSegmentTest() {
    sp = new SymmetricalPlayground(0.0, 0.0, 3000.0, 2000.0, 0.02, 1.0, 1500.0);
    sp->add_segment_symmetrical(me, teammate, 400.0, 400.0, 500.0, 500.0, 20)
                    ->add_segment_symmetrical(opponent_1, opponent_2, 400.0, 1000.0, 500.0, 900.0, 20);

}

virtual ~SymSegmentTest() {
    delete sp;
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


class SymRectangleTest : public ::testing::Test, public SVGTest {
protected:
    SymmetricalPlayground * sp;
    PlaygroundObjectID me = Playground::INVALID;
    PlaygroundObjectID teammate = Playground::INVALID;
    PlaygroundObjectID opponent_1 = Playground::INVALID;
    PlaygroundObjectID opponent_2 = Playground::INVALID;

SymRectangleTest() {
    sp = new SymmetricalPlayground(0.0, 0.0, 3000.0, 2000.0, 0.02, 1.0, 1500.0);
    sp->add_rectangle_symmetrical(me, teammate, 400.0, 400.0, 100.0, 200.0, 0.25f*((float)M_PI))
                    ->add_rectangle_symmetrical(opponent_1, opponent_2, 400.0, 1000.0, 100.0, 200.0, -0.25f*((float)M_PI));

}

virtual ~SymRectangleTest() {
    delete sp;
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


class SymRectangleLowerLeftTest : public ::testing::Test, public SVGTest {
protected:
    SymmetricalPlayground * sp;
    PlaygroundObjectID me = Playground::INVALID;
    PlaygroundObjectID teammate = Playground::INVALID;
    PlaygroundObjectID opponent_1 = Playground::INVALID;
    PlaygroundObjectID opponent_2 = Playground::INVALID;

SymRectangleLowerLeftTest() {
    sp = new SymmetricalPlayground(0.0, 0.0, 3000.0, 2000.0, 0.02, 1.0, 1500.0);
    sp->add_rectangle_lower_left_symmetrical(me, teammate, 400.0, 400.0, 100.0, 200.0, 0.25f*((float)M_PI))
                    ->add_rectangle_lower_left_symmetrical(opponent_1, opponent_2, 400.0, 1000.0, 100.0, 200.0, -0.25f*((float)M_PI));

}

virtual ~SymRectangleLowerLeftTest() {
    delete sp;
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


class SymBodyTest : public ::testing::Test, public SVGTest {
protected:
    SymmetricalPlayground * sp;
    PlaygroundObjectID me = Playground::INVALID;
    PlaygroundObjectID teammate = Playground::INVALID;
    PlaygroundObjectID opponent_1 = Playground::INVALID;
    PlaygroundObjectID opponent_2 = Playground::INVALID;

SymBodyTest() {
    std::vector<Point*> relative_point_refs = std::vector<Point*>(3);
    Point p1;
    Point p2;
    Point p3;

    p1.x = 0.0f;
    p1.y = 0.0f;
    relative_point_refs[0] = &(p1);
    p2.x = 200.0f;
    p2.y = 0.0f;
    relative_point_refs[1] = &(p2);
    p3.x = 100.0f;
    p3.y = 100.0f;
    relative_point_refs[2] = &(p3);
    sp = new SymmetricalPlayground(0.0, 0.0, 3000.0, 2000.0, 0.02, 1.0, 1500.0);
    sp->add_convex_body_symmetrical(me, teammate, 400.0, 400.0, relative_point_refs, 0.25f*((float)M_PI))
                    ->add_convex_body_symmetrical(opponent_1, opponent_2, 400.0, 1000.0, relative_point_refs, -0.25f*((float)M_PI));

}

virtual ~SymBodyTest() {
    delete sp;
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
    EXPECT_EQ(2.0, e1->length);
    EXPECT_EQ(2.0, e2->length);
    EXPECT_NE(0, tools_quasi_equal(2.0*sqrt(2.0), e3->length));
    EXPECT_NE(0, tools_quasi_equal(sqrt(2.0), e7->length));
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
    EXPECT_EQ(1, result); // segments with equal coordinates
    result = edge_intersects(e6, e5);
    EXPECT_EQ(0, result); // segments with common endpoint
    result = edge_intersects(e1, e5);
    EXPECT_EQ(0, result); // parallel horizontal
    result = edge_intersects(e2, e6);
    EXPECT_EQ(0, result); // parallel vertical
    result = edge_intersects(e4, e8);
    EXPECT_EQ(0, result); // on same straight line
    result = edge_intersects(e8, e9);
    EXPECT_EQ(1, result); // segments with quasi equal coordinates

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

TEST_F(ZoneTest, CheckZoneContainsNode) {
    int result;

    result = zone_contains_node(z1, z1->nodes[3]);
    EXPECT_EQ(0, result); // nodes of the border are not contained in the zone
    result = zone_contains_node(z1, n1);
    EXPECT_EQ(0, result); // nodes of the border are not contained in the zone
    result = zone_contains_node(z1, n2);
    EXPECT_NE(0, result); // (0, 0) is in the zone
    result = zone_contains_node(z1, n3);
    EXPECT_EQ(0, result); // (2, 0) is not in the zone
    result = zone_contains_node(z1, n4);
    EXPECT_EQ(0, result); // nodes of the border are not contained in the zone

}

TEST_F(ZoneTest, CheckZoneCenter) {
    float x;
    float y;

    zone_get_center(z1, &x, &y);
    EXPECT_NE(0, tools_quasi_equal(0.0, x));
    EXPECT_NE(0, tools_quasi_equal(0.0, y));
}

TEST_F(ZoneTest, CheckZoneVisibility) {
    EXPECT_EQ(true, zone_is_enabled(z1));
    EXPECT_EQ(true, zone_is_detected(z1));
    zone_set_is_detected(z1, false);
    zone_set_is_enabled(z1, false);
    EXPECT_EQ(false, zone_is_enabled(z1));
    EXPECT_EQ(false, zone_is_detected(z1));
}

TEST_F(SimplePlaygroundTest, CheckPlaygroundEdges) {
    FoundPath * path = NULL;
    std::vector<Point> * my_points;
    Point p_end;

    p->compute_edges();
    toSVG(p, path, "simpletest0.svg");
    // Initial position
    p->get_shape(my_points, me);
    p_end.x = 615.129;
    p_end.y = 1713.66;
    p->find_path(path, (*my_points)[0], p_end);
    toSVG(p, path, "simpletest1.svg");
    delete my_points;
    delete path;
    // Move the robots and compute again the path
    p->move(me, 200.0, 0.0)->synchronize();
    p->get_shape(my_points, me);
    p_end.x = 1484.87;
    p_end.y = 886.343;
    p->find_path(path, (*my_points)[0], p_end);
    toSVG(p, path, "simpletest2.svg");
    delete my_points;
    delete path;
}

TEST_F(PlaygroundTest, CheckPlayground) {
    FoundPath * path = NULL;
    std::vector<Point> * my_points;
    std::vector<Point> * opponent_points;

    //PlaygroundObjectID opponent_2 = 3;
    p->enable(opponent_2,0);

    p->compute_edges();
    toSVG(p, path, "test0.svg");
    // Initial position
    p->get_shape(my_points, me);
    p->get_shape(opponent_points, opponent_2);


    p->find_path(path, (*my_points)[6], (*opponent_points)[3]);
    toSVG(p, path, "test1.svg");
    delete my_points;
    delete opponent_points;
    delete path;
    // Move the robots and compute again the path
    p->move(me, 100.0, 0.0);
    p->move(opponent_2, 0.0, -100.0)->synchronize();
    p->get_shape(my_points, me);
    p->get_shape(opponent_points, opponent_2);
    p->find_path(path, (*my_points)[6], (*opponent_points)[3]);
    toSVG(p, path, "test2.svg");
    delete path;
    p->find_path(path, (*my_points)[4], (*opponent_points)[3]);
    toSVG(p, path, "test3.svg");
    delete my_points;
    delete opponent_points;
    delete path;
}

TEST_F(PlaygroundTest, SampleRobotMove) {
    FoundPath * path = NULL;

    //PlaygroundObjectID opponent_2 = 2;

    Point startPoint = {x : 250.0, y : 300.0 };
    Point secondPosition = {x : 1700.0, y : 1050.0 };
    Point finalPosition  = {x : 2800.0, y : 1800.0 };

    p->compute_edges();
    toSVG(p, path, "testRobot0.svg");
    // First move
    p->find_path(path, startPoint, secondPosition);
    toSVG(p, path, "testRobot1.svg");
    delete path;
    p->find_path(path, startPoint, finalPosition);
    toSVG(p, path, "testRobot2.svg");
    delete path;
    // Move the robots and compute again the path
    p->move(obj, 0.0, 300.0)->synchronize();
    p->find_path(path, startPoint, finalPosition);
    toSVG(p, path, "testRobot3.svg");
    delete path;

    p->enable(opponent_2,0);
    p->enable(obj,0);

    p->find_path(path, startPoint, finalPosition);
    toSVG(p, path, "testRobot4.svg");
    delete path;

}


TEST_F(PlaygroundTestImpossiblePlaces, SampleRobotMoveImpossiblePlace) {
    FoundPath * path = NULL;

    Point startPoint = {x : 250.0, y : 300.0 };
    Point secondPosition = {x : 1650.0, y : 1650.0 };
    //Point secondPosition = {x : 2000.0, y : 1150.0 };

    //Point finalPosition  = {x : 200.0, y : 1600.0 };

    p->enable(opponent_1,1);
    p->enable(garea_cube2high,1);

    p->compute_edges();


    toSVG(p, path, "testImpossible0.svg");
    // First move
    p->find_path(path, startPoint, secondPosition);
    toSVG(p, path, "testImpossible1.svg");
    // No path found, node (0, 0) is returned
    EXPECT_EQ(1, path->path.size());
    EXPECT_FLOAT_EQ(0.0, path->path[0]->x);
    EXPECT_FLOAT_EQ(0.0, path->path[0]->y);
    delete path;
    /*
    p->find_path(path, secondPosition, finalPosition);
    toSVG(p, path, "testImpossible2.svg");
    delete path;

    p->find_path(path, startPoint, finalPosition);
    toSVG(p, path, "testImpossible3.svg");
    delete path;
*/
}

TEST_F(PlaygroundSizeTest, CheckPlaygroundSizes) {

	PlaygroundObjectID teammate = Playground::INVALID;
	PlaygroundObjectID opponent_1 = Playground::INVALID;
	PlaygroundObjectID opponent_2 = Playground::INVALID;

    EXPECT_EQ(2, p->get_nodes_count());
    EXPECT_EQ(0, p->get_edges_count());
    EXPECT_EQ(0, p->get_zones_count());
    p->add_circle(me, 400.0, 525.0, 180.0, 8);
    EXPECT_EQ(10, p->get_nodes_count());
    EXPECT_EQ(0, p->get_edges_count());
    EXPECT_EQ(1, p->get_zones_count());
    EXPECT_EQ(8, p->get_nodes_count(me));
    p->add_circle(teammate, 400.0, 1575.0, 180.0, 8);
    EXPECT_EQ(18, p->get_nodes_count());
    EXPECT_EQ(0, p->get_edges_count());
    EXPECT_EQ(2, p->get_zones_count());
    EXPECT_EQ(8, p->get_nodes_count(teammate));
    p->add_circle(opponent_1, 2600.0, 525.0, 180.0, 8);
    EXPECT_EQ(26, p->get_nodes_count());
    EXPECT_EQ(0, p->get_edges_count());
    EXPECT_EQ(3, p->get_zones_count());
    EXPECT_EQ(8, p->get_nodes_count(opponent_1));
    p->add_circle(opponent_2, 2600.0, 1575.0, 180.0, 8);
    EXPECT_EQ(34, p->get_nodes_count());
    EXPECT_EQ(0, p->get_edges_count());
    EXPECT_EQ(4, p->get_zones_count());
    EXPECT_EQ(8, p->get_nodes_count(opponent_2));
    p->add_rectangle(1500.0, 1050.0, 200.0, 400.0, 0);
    EXPECT_EQ(38, p->get_nodes_count());
    EXPECT_EQ(0, p->get_edges_count());
    EXPECT_EQ(5, p->get_zones_count());
    p->compute_edges();
    EXPECT_EQ(38, p->get_nodes_count());
    EXPECT_EQ(38*37/2, p->get_edges_count());
    toSVG(p, NULL, "sizetest1.svg");
}

TEST_F(SymCircleTest, CheckCircle) {
    FoundPath * path = NULL;

    sp->compute_edges();
    toSVG(sp, path, "symCircle.svg");
    delete path;
}

TEST_F(SymHalfCircleTest, CheckHalfCircle) {
    FoundPath * path = NULL;

    sp->compute_edges();
    toSVG(sp, path, "symHalfCircle.svg");
    delete path;
}

TEST_F(SymQuarterCircleTest, CheckQuarterCircle) {
    FoundPath * path = NULL;

    sp->compute_edges();
    toSVG(sp, path, "symQuarterCircle.svg");
    delete path;
}

TEST_F(SymSegmentTest, CheckSegment) {
    FoundPath * path = NULL;

    sp->compute_edges();
    toSVG(sp, path, "symSegment.svg");
    delete path;
}

TEST_F(SymRectangleTest, CheckRectangle) {
    FoundPath * path = NULL;

    sp->compute_edges();
    toSVG(sp, path, "symRectangle.svg");
    delete path;
}

TEST_F(SymRectangleLowerLeftTest, CheckRectangleLowerLeft) {
    FoundPath * path = NULL;

    sp->compute_edges();
    toSVG(sp, path, "symRectangleLowerLeft.svg");
    delete path;
}

TEST_F(SymBodyTest, CheckconvexBody) {
    FoundPath * path = NULL;

    sp->compute_edges();
    toSVG(sp, path, "symBody.svg");
    delete path;
}

}  // namespace

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
