#include "mas/core/base.h"
#include "mas/core/math.h"

#include "mas/mesh/mesh.h"
#include "mas/mesh/io.h"
#include "mas/mesh/meshbv.h"

#include "mas/csg/bsp.h"
#include "mas/bvtree/bvtree.h"
#include "mas/csg/csg.h"

#include <iostream>
#include <stdlib.h>
#include <typeinfo>

// timings
#include "mas/core/time.h"

bool doNearestTest(const char filename[], double pnt[]) {

   mas::mesh::io::SimpleObjReader reader;
   reader.read(filename);
   using mas::mesh::PolygonMesh;
   using mas::Point3d;
   using mas::mesh::PPolygon;

   PolygonMesh mesh = reader.getPolygonMesh();

   Point3d p;
   p.x = pnt[0];
   p.y = pnt[1];
   p.z = pnt[2];

   Point3d nearestPoint;
   PPolygon nearest = mas::mesh::nearest_polygon(p, mesh, nearestPoint);
   int inside = mas::mesh::is_inside(p, mesh);

   printf("Polygon: %ld %ld %ld, Point: (%lf, %lf, %lf), ",
         nearest->verts[0]->idx, nearest->verts[1]->idx, nearest->verts[2]->idx,
         nearestPoint.x, nearestPoint.y, nearestPoint.z);

   if (inside == MESH_INSIDE_TRUE) {
      printf("Inside: true\n");
   } else if (inside == MESH_INSIDE_FALSE) {
      printf("Inside: false\n");
   } else {
      printf("Inside: unsure\n");
   }


}

bool doInsideTest(const char filename[], double pnt[], double dx[], int nx[]) {

   mas::mesh::io::SimpleObjReader reader;
   reader.read(filename);
   using mas::mesh::PolygonMesh;
   using mas::Point3d;
   using mas::mesh::PPolygon;
   using namespace mas::bvtree;

   PolygonMesh mesh = reader.getPolygonMesh();

   Point3d pStart;
   pStart.x = pnt[0];
   pStart.y = pnt[1];
   pStart.z = pnt[2];

   POBBTree obbt = get_obb_tree(mesh);

   Point3d p;
   for (int i=0; i<nx[0]; i++) {
      p.x = pStart.x + i*dx[0];
      for (int j=0; j<nx[1]; j++) {
         p.y = pStart.y + j*dx[1];
         for (int k=0; k<nx[2]; k++) {

            p.z = pStart.z + k*dx[2];
            int inside = mas::mesh::is_inside(p, mesh, obbt);

//            printf("Point: (%.5lf, %.5lf, %.5lf), Inside: ", p.x, p.y, p.z);
//            if (inside == MESH_INSIDE_TRUE) {
//               printf("true\n");
//            } else if (inside == MESH_INSIDE_FALSE) {
//               printf("false\n");
//            } else {
//               printf("unsure\n");
//            }

         }
      }
   }

   return true;
}

bool doInsideTestFaster(const char filename[], double pnt[], double dx[], int nx[]) {

   mas::mesh::io::SimpleObjReader reader;
   reader.read(filename);
   using mas::mesh::PolygonMesh;
   using mas::Point3d;
   using mas::mesh::PPolygon;
   using namespace mas::bvtree;

   PolygonMesh mesh = reader.getPolygonMesh();

   Point3d pStart;
   pStart.x = pnt[0];
   pStart.y = pnt[1];
   pStart.z = pnt[2];

   POBBTree obbt = get_obb_tree(mesh);

   Point3d p;
   for (int i=0; i<nx[0]; i++) {
      p.x = pStart.x + i*dx[0];
      for (int j=0; j<nx[1]; j++) {
         p.y = pStart.y + j*dx[1];
         for (int k=0; k<nx[2]; k++) {
            mas::mesh::InsideMeshQueryData data;
            bool inside2 = mas::mesh::is_inside(p, obbt, data);

//            printf("Point: (%.5lf, %.5lf, %.5lf), Inside: ", p.x, p.y, p.z);
//
//            if (data.unsure) {
//               printf("unsure\n");
//            } else if (inside2) {
//               printf("true\n");
//            } else {
//               printf("false\n");
//            }

         }
      }
   }

   return true;
}

int main( int argc, const char* argv[] ) {

   double pnt[3];
   double dx[3] = {0.001,0.001,0.001};
   int dn[3] = {10,10,10};

   for (int i=0; i<3; i++) {
      sscanf(argv[i+2], "%lf", &pnt[i]);
   }

   mas::time::Timer timer;

   timer.start();
   doInsideTestFaster(argv[1], pnt, dx, dn);
   timer.stop();
   double micros2 = timer.getMicroseconds();
   printf("Took %f us\n", micros2);

   timer.reset();
   timer.start();
   doInsideTest(argv[1], pnt, dx, dn);
   timer.stop();
   double micros1 = timer.getMicroseconds();
   printf("Took %f us\n", micros1);

   return 0;
}

