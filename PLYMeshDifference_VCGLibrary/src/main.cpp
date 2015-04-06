#include <iostream>
#include <wrap/io_trimesh/import.h>
#include <vcg/simplex/face/component_ep.h>

class CFace;
class CVertex;
struct UsedTypes:public vcg::UsedTypes< vcg::Use<CFace>::AsFaceType, vcg::Use<CVertex>::AsVertexType>{};
class CVertex   : public vcg::Vertex<UsedTypes,vcg::vertex::Coord3d,vcg::vertex::Qualityf,vcg::vertex::Normal3d,vcg::vertex::Color4b,vcg::vertex::BitFlags> {};
class CFace     : public vcg::Face< UsedTypes,vcg::face::VertexRef, vcg::face::Normal3d, vcg::face::EdgePlane,vcg::face::Color4b,vcg::face::Mark,vcg::face::BitFlags> {};
class CMesh     : public vcg::tri::TriMesh< std::vector<CVertex>, std::vector<CFace> > {};

bool LoadMesh(std::string filename, CMesh& mesh) {
  int errorCode = vcg::tri::io::Importer<CMesh>::Open(mesh, filename.c_str());
  if(errorCode) {
    printf("Error in reading %s: '%s'\n", filename, vcg::tri::io::Importer<CMesh>::ErrorMsg(errorCode));
    if(vcg::tri::io::Importer<CMesh>::ErrorCritical(errorCode)) {
      return false;
    }
  }
  return true;
}

int main()
{
  CMesh meshA, meshB;
  std::string filenameA = "../data/human_1.ply";
  std::string filenameB = "../data/human_2.ply";
  // open meshes
  if (!LoadMesh(filenameA, meshA) || !LoadMesh(filenameB, meshB)) {
    return EXIT_FAILURE;
  }
  
  std::cout << "Test." << std::endl;
}
