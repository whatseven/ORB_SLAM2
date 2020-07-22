#ifndef ORBVOCABULARY_H
#define ORBVOCABULARY_H

#include <DBoW2/FORB.h>
#include <DBoW2/TemplatedVocabulary.h>

namespace MONOCULAR_ORB_SLAM2
{

typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
  ORBVocabulary;

} //namespace ORB_SLAM

#endif // ORBVOCABULARY_H
