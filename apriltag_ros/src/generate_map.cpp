#include "tag_yaml.hpp"
#include <vector>
#include <iostream>

using namespace apriltag_ros;

int main(int argc, char **argv) {

  if (argc != 2) {
    printf("usage: generate_map <output path>\n");
    return -1;
  }

  double x = 0, y = 0;
  int tag_id = 0;

  Tag tag;

  //  encode into map
  YAML::Emitter emitter;
  emitter << YAML::Comment("MRSL Floor Map, 9x12 tags");
  emitter << YAML::BeginSeq;
  emitter << YAML::Block;

  for (int i = 0; i < 9; i++) {
    x = 0.0;

    for (int j = 0; j < 12; j++) {
      tag.id = tag_id++;
      tag.p[0] = cv::Point3d(x + 0.152, y, 0);
      tag.p[1] = cv::Point3d(x + 0.152, y + 0.152, 0);
      tag.p[2] = cv::Point3d(x, y + 0.152, 0);
      tag.p[3] = cv::Point3d(x, y, 0);

      emitter << tag;

      x += 0.152 * 2;
    }

    if (i == 2 || i == 5) {
      y += 0.178 + 0.152;
    } else {
      y += 0.152 * 2;
    }
  }
  emitter << YAML::EndSeq;
  emitter << YAML::Comment("End of map");

  //  save to disk
  FILE *output = fopen(argv[1], "w");
  if (!output) {
    printf("Failed to open %s for writing\n", argv[1]);
    return -1;
  }

  fprintf(output, "%s", emitter.c_str());
  fclose(output);

  printf("Wrote to %s\n", argv[1]);
  return 0;
}
