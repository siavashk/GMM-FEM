#include "mas/mesh/io.h"
#include "mas/mesh/meshbv.h"

#include <iostream>
#include <stdlib.h>

// timings
#include "mas/core/time.h"
#include "mas/mesh/simplification.h"
#include "mas/core/exception.h"

bool doEdgeCollapseTest(const char filename[], const char out[], int reduce) {

   mas::mesh::io::SimpleObjReader reader;
   mas::mesh::io::SimpleObjWriter writer;

   using mas::mesh::PolygonMesh;
   using mas::Point3d;
   using mas::mesh::SharedPolygon;
   using mas::mesh::SharedHalfEdge;
   using mas::mesh::HalfEdge;

   std::unique_ptr<PolygonMesh> mesh = std::unique_ptr<PolygonMesh>(reader.read(filename));
   if (mesh == nullptr) {
       std::cout << "Cannot find mesh " << filename << std::endl;
       return false;
   }
   mesh->connect(); // build connectivity

   // check connectivity
   for (SharedPolygon& face : mesh->faces) {
       HalfEdge* he0 = face->he0.get();
       HalfEdge* he = he0;
       do {
           if (he->opposite == nullptr) {
               std::cout << "BROKEN!!!" << std::endl;
           }
           he = he->next.get();
       } while (he != he0);
   }

   mas::mesh::edge_collapse(*mesh, mesh->numFaces()-reduce);

   writer.write(*mesh, out);

   return true;
}

int main( int argc, const char* argv[] ) {

   int reduce = 1;
   if (argc > 3) {
       sscanf(argv[3], "%d", &reduce);
   }

   mas::time::Timer timer;
   timer.start();
   doEdgeCollapseTest(argv[1], argv[2], reduce);
   timer.stop();
   double micros = timer.getMicroseconds();
   printf("Took %f us\n", micros);


   return 0;
}

