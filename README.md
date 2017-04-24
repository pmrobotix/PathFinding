# PM_Robotix path finding

This is a very simple C++ path finding library for robots.

The world is modelled as a graph where the nodes have physical locations. The nodes form convex zones. The inside of a zone cannot be reached by a robot. Zones can be enabled or disabled.

When disabled, they become transparent and are ignored by the path finding algorithm. Zones can move, rotate and even change shape (they must stay convex).

Once a world (playground) has been described in terms of nodes, all the edges between the nodes are computed except for those that are within a zone.

This project is still work in progress. The low-level API has been taken from the [BHTeam](https://bitbucket.org/bhteam/bhware-open/overview) and should be stable.  The high level API will certainly change over time.

The unit tests use the Google Testing Framework.

These tests also use https://github.com/adishavit/simple-svg.git. Clone that repository if you want to compile the tests.

The project compiles, but most probably it has a lot of issues.

This code is released under the [GNU General Public License version 3](https://www.gnu.org/licenses/licenses.en.html#GPL).