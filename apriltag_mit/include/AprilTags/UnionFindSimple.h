#ifndef APRILTAGS_UNIONFINDSIMPLE_H_
#define APRILTAGS_UNIONFINDSIMPLE_H_

#include <vector>

namespace AprilTags {

//! Implementation of disjoint set data structure using the union-find algorithm
class UnionFindSimple {
  //! Identifies parent ids and sizes.
  struct Data {
    int id;
    int size;
  };

 public:
  explicit UnionFindSimple(int maxId) : data(maxId) {
    init();
  };

  int getSetSize(int thisId) { return data[getRepresentative(thisId)].size; }

  int getRepresentative(int thisId);

  //! Returns the id of the merged node.
  /** @param aId
   *  @param bId
   */
  int connectNodes(int aId, int bId);

  void printDataVector() const;

 private:
  void init();

  std::vector<Data> data;
};

}  // namespace AprilTags

#endif  // APRILTAGS_UNIONFINDSIMPLE_H_
