#include <iostream>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export.h>
#include <vcg/complex/complex.h>
#include <vcg/simplex/face/component_ep.h>
#include <vcg/complex/algorithms/update/component_ep.h>
#include <vcg/complex/algorithms/update/bounding.h>
#include "sampling.h"

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
  int ColorMin = 0, ColorMax = 0;
  CMesh meshA, meshB;
  std::string filenameA = "../data/human_1.ply";
  std::string filenameB = "../data/human_2.ply";

  int flags = vcg::SamplingFlags::VERTEX_SAMPLING |
              vcg::SamplingFlags::EDGE_SAMPLING |
              vcg::SamplingFlags::FACE_SAMPLING |
              vcg::SamplingFlags::SIMILAR_SAMPLING |
              vcg::SamplingFlags::SAVE_ERROR |
              vcg::SamplingFlags::USE_STATIC_GRID;

  // open meshes
  if (!LoadMesh(filenameA, meshA) || !LoadMesh(filenameB, meshB)) {
    return EXIT_FAILURE;
  }
  
  int n_samples_target = 10 * max(meshA.fn, meshB.fn);

  vcg::tri::UpdateComponentEP<CMesh>::Set(meshA);
  vcg::tri::UpdateComponentEP<CMesh>::Set(meshB);

  vcg::tri::UpdateBounding<CMesh>::Box(meshA);
  vcg::tri::UpdateBounding<CMesh>::Box(meshB);

  // set Bounding Box.
  vcg::Box3<CMesh::ScalarType> bbox,
                          tmp_bbox_M1 = meshA.bbox, 
                          tmp_bbox_M2 = meshB.bbox;
  bbox.Add(meshA.bbox);
  bbox.Add(meshB.bbox);
  bbox.Offset(bbox.Diag() * 0.02);
  meshA.bbox = bbox;
  meshB.bbox = bbox;

  vcg::Sampling<CMesh> ForwardSampling(meshA, meshB);
  ForwardSampling.SetFlags(flags);
  ForwardSampling.SetSamplesPerAreaUnit(1.0f);
  ForwardSampling.Hausdorff();

  // save diff file
  vcg::tri::io::PlyInfo p;
  p.mask|=vcg::tri::io::Mask::IOM_VERTCOLOR | vcg::tri::io::Mask::IOM_VERTQUALITY /* | vcg::ply::PLYMask::PM_VERTQUALITY*/ ;
  //p.mask|=vcg::ply::PLYMask::PM_VERTCOLOR|vcg::ply::PLYMask::PM_VERTQUALITY;
  if(ColorMax != 0 || ColorMin != 0){
    vcg::tri::UpdateColor<CMesh>::PerVertexQualityRamp(meshA, ColorMin, ColorMax);
  }
  vcg::tri::io::ExporterPLY<CMesh>::Save(meshA, "../data/output.ply", false, p);

  std::cout << "DistMax: " << ForwardSampling.GetDistMax() << std::endl;
}
