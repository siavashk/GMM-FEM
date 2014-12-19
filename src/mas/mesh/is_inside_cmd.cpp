#include "mas/mesh/io.h"
#include "mas/mesh/meshbv.h"

#include <iostream>
#include <stdlib.h>
#include <typeinfo>

// timings
#include "mas/core/time.h"
#include "mas/mesh/simplification.h"
#include "mas/core/exception.h"

bool doNearestTest(const char filename[], double pnt[]) {

   mas::mesh::io::SimpleObjReader reader;

   using mas::mesh::PolygonMesh;
   using mas::Point3d;
   using mas::mesh::SharedPolygon;

   std::unique_ptr<PolygonMesh> mesh = std::unique_ptr<PolygonMesh>(reader.read(filename));
   mesh->connect(); // build connectivity

   Point3d p;
   p.x = pnt[0];
   p.y = pnt[1];
   p.z = pnt[2];

   Point3d nearestPoint;
   SharedPolygon nearest = mas::mesh::nearest_polygon(p, *mesh, nearestPoint);
   bool inside = mas::mesh::is_inside(p, *mesh);

   printf("Polygon: %ld %ld %ld, Point: (%lf, %lf, %lf), ",
         nearest->verts[0]->idx, nearest->verts[1]->idx, nearest->verts[2]->idx,
         nearestPoint.x, nearestPoint.y, nearestPoint.z);

   if (inside) {
      printf("Inside: true\n");
   } else if (inside = false) {
      printf("Inside: false\n");
   } else {
      printf("Inside: unsure\n");
   }


}

bool doInsideTest(const char filename[], double pnt[], double dx[], int nx[]) {

   mas::mesh::io::SimpleObjReader reader;
   using mas::mesh::PolygonMesh;
   using mas::Point3d;
   using mas::mesh::SharedPolygon;
   using namespace mas::bvtree;

   std::unique_ptr<PolygonMesh> mesh = std::unique_ptr<PolygonMesh>(reader.read(filename));
   mesh->connect(); // build connectivity

   mas::mesh::edge_collapse(*mesh, 0.5, [](){});

   Point3d pStart;
   pStart.x = pnt[0];
   pStart.y = pnt[1];
   pStart.z = pnt[2];

   UniqueOBBTree obbt(get_obb_tree(*mesh));

   Point3d p;
   for (int i=0; i<nx[0]; i++) {
      p.x = pStart.x + i*dx[0];
      for (int j=0; j<nx[1]; j++) {
         p.y = pStart.y + j*dx[1];
         for (int k=0; k<nx[2]; k++) {
			p.z = pStart.z + k*dx[2];
            mas::mesh::InsideMeshQueryData data;
            bool inside2 = mas::mesh::is_inside(p, *obbt, data);

            printf("Point: (%.5lf, %.5lf, %.5lf), Inside: ", p.x, p.y, p.z);
            if (data.unsure) {
                printf("unsure\n");
            } else if (inside2) {
                printf("true\n");
            } else {
                printf("false\n");
            }

            //            printf("  nearest face: ");
            //            if (data.nearestFace != nullptr) {
            //                for (mas::mesh::SharedVertex3d& vtx : data.nearestFace->polygon->verts) {
            //                    printf("%d ", vtx->idx );
            //                }
            //                printf(", dist = %.5lf", data.nearestPoint.distance(p));
            //                printf("'\n");
            //            } else {
            //                printf("null\n");
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

   if (argc == 7 ) {
       sscanf(argv[5], "%lf", &dx[0]);
       dx[1] = dx[0];
       dx[2] = dx[0];
       sscanf(argv[6], "%d", &dn[0]);
       dn[1] = dn[0];
       dn[2] = dn[0];
   } else if (argc == 11 ) {
       for (int i=0; i<3; i++) {
           sscanf(argv[i+5], "%lf", &dx[i]);
       }
       for (int i=0; i<3; i++) {
           sscanf(argv[i+8], "%d", &dn[i]);
       }
   }


	printf("Starting Point: (%.5lf, %.5lf, %.5lf)\n\n", pnt[0], pnt[1], pnt[2]);

   mas::time::Timer timer;
   timer.start();
   doInsideTest(argv[1], pnt, dx, dn);
   timer.stop();
   double micros = timer.getMicroseconds();
   printf("Took %f us\n", micros);


   return 0;
}

