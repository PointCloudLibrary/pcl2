/** \file correspondence_identification.h 
 * \brief Declares classes for finding corresponding points in two clouds
 */

#ifndef PCL2_CORRESPONDENCE_IDENTIFICATION_H
#define PCL2_CORRESPONDENCE_IDENTIFICATION_H

#include "pcl2/cloud.h"
#include "pcl2/typed_matrix.h"

#include <assert.h>

namespace pcl2
{

class PointCorrespondences
{
public:
  PointCorrespondences (Cloud source, Cloud target); // source_indices and target_indices are initialized to 1,2,...,n
  PointCorrespondences (Cloud source, // source_indices is initialized to 1,2,...,n
                        Cloud target, MatI target_indices); 
  PointCorrespondences (Cloud source, MatI source_indices, 
                        Cloud target, MatI target_indices);
  PointCorrespondences (Cloud source, MatI source_indices, MatF source_distances, 
                        Cloud target, MatI target_indices, MatF target_distances);

  inline Cloud & getSource () { return (source_); }
  inline Cloud & getTarget () { return (target_); }

  inline void setCorrespondenceIndices (MatI source_indices, MatI target_indices);
  inline void setCorrespondenceDistances (MatF source_distances, MatF target_distances);
  inline void setWeights (MatF weights) { weights_ = weights; };

protected:
  Cloud source_, target_;
  MatI source_indices_, target_indices_; // source_(source_indices_) corresponds to target_(target_indices)
  MatF distances_, weights_;
};

class CorrespondenceIdentification
{
public:
  typedef boost::shared_ptr<CorrespondenceIdentification> Ptr;

  class Matching 
  {
  public:
    typedef boost::shared_ptr<Matching> Ptr;
    virtual PointCorrespondences match (const Cloud & source, const Cloud & target) const = 0; 
  };

  class Weighting
  {
  public:
    typedef boost::shared_ptr<Weighting> Ptr;
    virtual MatF weight (const PointCorrespondences & correspondences) const = 0;
  };

  class Rejection
  {
  public:
    typedef boost::shared_ptr<Rejection> Ptr;
    virtual PointCorrespondences reject (const PointCorrespondences & correspondences) const = 0;
  };

  CorrespondenceIdentification ()
  {}

  inline void setMatchingFunction (const Matching::Ptr & matching)
  { 
    matching_ = matching; 
  }
  
  inline void setWeightingFunction (const Weighting::Ptr & weighting) { weighting_ = weighting; }
  inline void addRejectionFunction (const Rejection::Ptr & rejection) { rejection_.push_back (rejection); }

  inline PointCorrespondences identifyCorrespondences (const Cloud source, const Cloud target) const
  {
    // Find matches
    PointCorrespondences correspondences = matching_->match (source, target);

    // Apply rejection criteria
    std::vector<Rejection::Ptr>::const_iterator itr;
    for (itr = rejection_.begin (); itr != rejection_.end (); ++itr)
    {
      correspondences = (*itr)->reject (correspondences);
    }

    // Weight the remaining correspondences
    if (weighting_)
      correspondences.setWeights (weighting_->weight (correspondences));

    return (correspondences);
  }

protected:
  Matching::Ptr matching_;
  Weighting::Ptr weighting_;
  std::vector<Rejection::Ptr> rejection_;
};


class NearestNeighborMatching : public CorrespondenceIdentification::Matching
{
  virtual PointCorrespondences match (const Cloud & source, const Cloud & target) const
  {
    /*
    MatI indices (source.size (), 1);
    for (int i = 0; i < source.size (); ++i)
    {
      MatI i_in_mat (1, 1); i_in_mat (0, 0) = i; // ick!
      MatF query = source (i_in_mat);
      indices (i, 0) = findNearestNeighbor (target, query);
    }

    MatF::Row query = source.getRow (0);
    MatF::Row idx = indices.getRow (0);
    while (query.getIndex () != source.size ())
    {
      idx.fill (target, query);
      query.next ();
      idx.next ();
    }
    
    return (PointCorrespondences correspondences (source, target, indices));
    */
    assert (false);
  }
};


}

#endif // PCL2_CORRESPONDENCE_IDENTIFICATION_H
